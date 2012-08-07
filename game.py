import subprocess
from subprocess import PIPE
import time
import random

import selectvisual
view = selectvisual.get_view_module()

import world
from world import box2d

import stats
import conf


class Game(object):
    def __init__(self, testmode=False, tournament=None):
        self.testmode = testmode
        self.tournament = tournament

        self.models = {}
        self.procs = {}
        self.results = {}
        self.timeouts = {}
        self.rnd = 0

        self.w = world.World()
        self.cl = world.CL()
        self.w.w.SetContactListener(self.cl)
        self.cl.w = self.w

    def run(self):
        self.load_tanks()
        while ((self.testmode and not self.tournament)
                    or len(self.procs) > 1) and not self.w.v.quit:
            if self.rnd > 60 * conf.maxtime:
                break
            self.tick()
        self.finish()

    def load_tanks(self):
        tanks = conf.tanks
        for tank in tanks:
            tankname = tank
            while tankname in self.w.tanks:
                tankname += '_'
            print 'STARTING', tankname,
            proc = subprocess.Popen([conf.subproc_python,
                                        conf.subproc_main,
                                        tank, tankname,
                                        str(int(self.testmode))],
                                        stdin=PIPE, stdout=PIPE)
            result = proc.stdout.readline().strip()

            if result in ['ERROR', 'END']:
                print 'ERROR!'
            else:
                print 'STARTED'
                model = self.w.maketank(tank, tankname)
                self.models[tankname] = model
                self.procs[tankname] = proc
                self.timeouts[tankname] = 0

        self.ntanks = len(self.models)
        self.t0 = int(time.time())

    def tick(self):
        procs = self.procs
        ntanks = self.ntanks
        timeouts = self.timeouts
        w = self.w
        rnd = self.rnd
        result = ''

        items = self.models.items()
        random.shuffle(items)
        for tankname, model in items:
            if tankname not in self.procs:
                continue

            health = model.health
            body = model.body
            pos = body.position
            possens = '%s;%s' % (int(pos.x*10), int(pos.y*10))
            tur = model.get_turretangle()
            ping = '%s;%s;%s' % (model._pingtype,
                                    model._pingangle,
                                    model._pingdist)
            gyro = model.gyro()
            heat = int(model._cannonheat)
            loading = int(model._cannonreload)
            pinged = int(model._pinged == rnd - 1)
            line = 'TICK:%s|HEALTH:%s|POS:%s|TUR:%s|PING:%s|GYRO:%s|HEAT:%s|LOADING:%s|PINGED:%s\n' % (rnd, health, possens, tur, ping, gyro, heat, loading, pinged)
            

            proc = procs[tankname]

            if not model.alive:
                model._kills = ntanks - len(procs)
                del procs[tankname]
                print 'DEAD tank', tankname, 'health is 0'
                proc.stdin.flush()
                proc.stdin.close()
                proc.stdout.close()
                proc.kill()
                continue

            proc.stdin.write(line)
            try:
                result = proc.stdout.readline().strip()
            except IOError:
                print 'ERROR with', tankname

            if result == 'TIMEOUT':
                timeouts[tankname] += 1
                if timeouts[tankname] > 5:
                    del procs[tankname]
                    print 'REMOVED tank', tankname, 'due to excessive timeouts'
                    proc.stdin.flush()
                    proc.stdin.close()
                    proc.stdout.close()
                    proc.kill()

            elif result == 'END':
                del procs[tankname]
                print 'FINISHED: tank', tankname
                proc.stdin.flush()
                proc.stdin.close()
                proc.stdout.close()
                proc.kill()

            elif result == 'ERROR':
                del procs[tankname]
                print 'ERROR: tank', tankname
                proc.stdin.flush()
                proc.stdin.close()
                proc.stdout.close()
                proc.kill()

            else:
                timeouts[tankname] = 0


            
            commands = {}
            try:
                props = result.split('|')
                for prop in props:
                    kind, val = prop.split(':')
                    try:
                        vconv = int(val)
                    except ValueError:
                        pass
                    else:
                        val = vconv
                    commands[kind] = val
            except ValueError:
                continue

            
            
            

            for kind, val in commands.items():
                if kind == 'FORCE':
                    
                    val = min(val, 100)
                    val = max(-100, val)
                    
                    localforce = box2d.b2Vec2(val, 0)
                    worldforce = body.GetWorldVector(localforce)
                    body.ApplyForce(worldforce, pos)
                elif kind == 'TORQUE':
                    
                    val = min(val, 100)
                    val = max(-100, val)
                    torque = conf.maxtorque * val/100.0
                    body.ApplyTorque(torque)
                elif kind == 'FIRE':
                    if val == '_':
                        
                        pass
                    elif val == 'X':
                        
                        w.makebullet(tankname)
                    else:
                        
                        ticks = int(60 * val / conf.bulletspeed)
                        w.makebullet(tankname, ticks)
                elif kind == 'PING':
                    if val:
                        kind, angle, dist = w.makeping(tankname, rnd)
                        if kind is not None:
                            model._pingtype = kind[0]
                            model._pingangle = angle
                            model._pingdist = int(dist)
                elif kind == 'TURRET':
                    model.set_turretangle(val)


        w.step()



        self.rnd += 1


    def finish(self):
        print 'FINISHING'

        models = self.models
        testmode = self.testmode
        ntanks = self.ntanks
        procs = self.procs
        tournament = self.tournament

        alive = [model for model in models.values() if model.alive]
        if not testmode and len(alive)==1:
            model = alive[0]
            print 'WINNER:', model.name
            winner = model
            model._kills = ntanks-1
        elif not testmode:
            winner = None
            if self.rnd >= conf.maxtime*60:
                print 'The battle finished after maximum time:', conf.maxtime, 'seconds.'
            else:
                print 'The battle finished after', int(self.rnd/60), 'seconds.'
            print 'STILL ALIVE:'
            for model in alive:
                print '   ', model.name
        else:
            winner = None

        for tankname, model in models.items():
            print tankname, 'caused', model._damage_caused, 'damage'
            if tankname in procs:
                line = 'FINISH\n'
                proc = procs[tankname]
                proc.stdin.write(line)
                proc.stdin.flush()
                proc.stdin.close()
                proc.stdout.close()
                del procs[tankname]

            if winner is None and model.alive:
                model._kills = ntanks - len(alive)

            if model == winner:
                win = 1
            else:
                win = 0

            if not testmode:
                stats.update(model.kind, win, ntanks-1, model._kills,
                                model._damage_caused)

            if tournament is not None:
                stats.tournament_update(tournament, model.kind, model.name, win,
                                                ntanks-1, model._kills,
                                                model._damage_caused)
