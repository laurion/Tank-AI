import random

import Box2D as box2d
pi = 3.1415927410125732

import conf

import selectvisual
view = selectvisual.get_view_module()


class Tank(object):
    ntanks = 0
    def __init__(self, wld, kind, name, pos, ang):
        w = wld.w

        Tank.ntanks += 1
        self.n = Tank.ntanks

        self.alive = True
        self.health = conf.maxhealth
        self.kind = kind
        self.name = name

        self._pingtype = 'w'
        self._pingangle = 0
        self._pingdist = 0

        self._pinged = -5

        self._cannonheat = 0
        self._cannonreload = 0

        self._kills = 0 
        self._damage_caused = 0

        bodyDef = box2d.b2BodyDef()
        bodyDef.position = pos
        bodyDef.angle = ang

        bodyDef.linearDamping = conf.tank_linearDamping
        bodyDef.angularDamping = conf.tank_angularDamping
        bodyDef.userData = {}

        body = w.CreateBody(bodyDef)

        shapeDef = box2d.b2PolygonDef()
        shapeDef.SetAsBox(1, 1)
        shapeDef.density = conf.tank_density
        shapeDef.friction = conf.tank_friction
        shapeDef.restitution = conf.tank_restitution
        shapeDef.filter.groupIndex = -self.n
        body.CreateShape(shapeDef)
        body.SetMassFromShapes()

        body.userData['actor'] = self
        body.userData['kind'] = 'tank'

        self.body = body

        turretDef = box2d.b2BodyDef()
        turretDef.position = pos
        turretDef.angle = ang

        turretDef.linearDamping = 0
        turretDef.angularDamping = 0
        turret = w.CreateBody(bodyDef)

        shapeDef = box2d.b2PolygonDef()
        shapeDef.SetAsBox(.1, .1)
        shapeDef.density = 1
        shapeDef.friction = 0
        shapeDef.restitution = 0
        shapeDef.filter.groupIndex = -self.n
        turret.CreateShape(shapeDef)
        turret.SetMassFromShapes()
        self.turret = turret

        jointDef = box2d.b2RevoluteJointDef()
        jointDef.Initialize(body, turret, pos)
        jointDef.maxMotorTorque = conf.turret_maxMotorTorque
        jointDef.motorSpeed = 0.0
        jointDef.enableMotor = True
        self.turretjoint = w.CreateJoint(jointDef)
        self._turretangletarget = 0

        v = wld.v.addtank(pos, ang)
        self.v = v

        i = wld.v.addtankinfo(self.n, name)
        self.i = i

    def gyro(self):
        radians = self.body.angle
        degrees = int(round((180 / pi) * radians))
        return degrees

    def set_turretangle(self, angle):
        radians = (pi / 180.) * angle
        self._turretangletarget = radians

    def get_turretangle(self):
        degrees = int(round((180 / pi) * self.turretjoint.GetJointAngle()))
        return degrees

    def turretcontrol(self):
        joint = self.turretjoint
        angleError = joint.GetJointAngle() - self._turretangletarget
        gain = 0.5
        joint.SetMotorSpeed(-gain * angleError)


class Bullet(object):
    def __init__(self, wld, tank):
        self.wld = wld
        w = wld.w
        self.tank = tank

        self._fuse = None
        self._exploding = False

        r = tank.turret
        pos = r.position
        vel = r.linearVelocity
        ang = r.angle

        blocalvel = box2d.b2Vec2(conf.bulletspeed, 0)
        bwvel = r.GetWorldVector(blocalvel)
        bvel = bwvel + vel
        

        bodyDef = box2d.b2BodyDef()
        blocalpos = box2d.b2Vec2(.1, 0)
        bwpos = r.GetWorldVector(blocalpos)
        bpos = bwpos + pos
        bodyDef.position = bpos
        bodyDef.angle = ang
        bodyDef.isBullet = True
        bodyDef.linearDamping = 0
        bodyDef.userData = {}

        body = w.CreateBody(bodyDef)
        
        
        body.linearVelocity = bvel

        shapeDef = box2d.b2PolygonDef()
        shapeDef.SetAsBox(.1, .1)
        shapeDef.density = conf.bullet_density
        shapeDef.restitution = 0
        shapeDef.friction = 0
        shapeDef.filter.groupIndex = -tank.n
        b = body.CreateShape(shapeDef)
        b.userData = {}
        body.SetMassFromShapes()

        body.userData['actor'] = self
        body.userData['kind'] = 'bullet'
        body.userData['shooter'] = tank

        self.body = body

        v = wld.v.addbullet(pos)
        self.v = v

    def explode(self):
        self._exploding = 1

        tank = self.body.userData['shooter'].name
        

        for ring, radius in enumerate(conf.explosion_radii):
            cdef = box2d.b2CircleDef()
            cdef.radius = radius

            s = self.body.CreateShape(cdef)
            s.userData = {}
            s.userData['ring'] = ring
            s.userData['bullet'] = self
            s.userData['hits'] = {0:[], 1:[], 2:[]}

        e = self.wld.v.addexplosion(self.body.position)
        self.e = e

class Wall(object):
    def __init__(self, w, pos, size):
        walldef = box2d.b2BodyDef()
        walldef.position = pos
        walldef.userData = {}
        wallbod = w.CreateBody(walldef)
        wallbod.userData['actor'] = None
        wallbod.userData['kind'] = 'wall'
        wallbod.iswall = True
        wallshp = box2d.b2PolygonDef()
        width, height = size
        wallshp.SetAsBox(width, height)
        wallbod.CreateShape(wallshp)

        v = view.Wall(pos, size)
        self.v = v


class World(object):
    def __init__(self):
        self.count = 1000
        self.force = 10

        self.tanks = {}
        self.bullets = []
        self.sprites = {}
        self.to_destroy = []

        halfx = 30
        self.ahalfx = 20
        halfy = 25
        self.ahalfy = 20

        gravity = (0, 0)
        doSleep = True

        self.timeStep = 1.0 / 60.0
        self.velIterations = 10
        self.posIterations = 8


        aabb = box2d.b2AABB()
        aabb.lowerBound = (-halfx, -halfy)
        aabb.upperBound = (halfx, halfy)

        self.w = box2d.b2World(aabb, gravity, doSleep)
        self.w.GetGroundBody().SetUserData({'actor': None})

        self.makearena()


    def makearena(self):
        self.v = view.Arena()

        ahx = self.ahalfx
        ahy = self.ahalfy

        wl = Wall(self.w, (-ahx, 0), (1, ahy+1))
        wl = Wall(self.w, (ahx, 0), (1, ahy+1))
        wl = Wall(self.w, (0, ahy), (ahx+1, 1))
        wl = Wall(self.w, (0, -ahy), (ahx+1, 1))

        for block in range(5):
            
            pass

    def makeblock(self):
        x = random.randrange(-self.ahalfx, self.ahalfx+1)
        y = random.randrange(-self.ahalfy, self.ahalfy+1)
        w = random.randrange(1, 20)/10.0
        h = random.randrange(1, 20)/10.0
        wl = Wall(self.w, (x, y), (w, h))

    def posoccupied(self, pos):
        px, py = pos.x, pos.y
        for name, tank in self.tanks.items():
            rbpos = tank.body.position
            rx, ry = rbpos.x, rbpos.y
            if (rx-2 < px < rx+2) and (ry-2 < py < ry+2):
                return True

        return False

    def maketank(self, kind, name, pos=None, ang=None):
        rhx = self.ahalfx-2
        rhy = self.ahalfy-2

        while pos is None or self.posoccupied(pos):
            rx = random.randrange(-rhx, rhx)
            ry = random.randrange(-rhy, rhy)
            pos = box2d.b2Vec2(rx, ry)

        if ang is None:
            ang = random.randrange(628) / float(100)

        tank = Tank(self, kind, name, pos, ang)
        self.tanks[name] = tank

        return tank

    def makebullet(self, rname, fuse=None):
        tank = self.tanks[rname]
        if tank._cannonheat > conf.cannon_maxheat:
            
            tank._cannonreload += conf.overheat_fire_reload_penalty
            return None
        elif tank._cannonreload > 0:
            
            tank._cannonreload += conf.unloaded_fire_reload_penalty
            return None

        bullet = Bullet(self, tank)
        bullet._fuse = fuse
        self.bullets.append(bullet)

        tank._cannonheat += conf.cannon_heating_per_shot
        tank._cannonreload = conf.cannon_reload_ticks

        return bullet

    def makeping(self, rname, rnd):
        tank = self.tanks[rname]
        body = tank.turret

        segmentLength = 65.0

        blocalpos = box2d.b2Vec2(1.12, 0)

        segment = box2d.b2Segment()
        laserStart = (1.12, 0)
        laserDir = (segmentLength, 0.0)
        segment.p1 = body.GetWorldPoint(laserStart)
        segment.p2 = body.GetWorldVector(laserDir)
        segment.p2+=segment.p1

        lambda_, normal, shape = self.w.RaycastOne(segment, False, None)
        hitp = (1 - lambda_) * segment.p1 + lambda_ * segment.p2
        angle = tank.get_turretangle()
        dist = box2d.b2Distance(segment.p1, hitp)

        if shape is not None:
            hitbody = shape.GetBody()
            kind = hitbody.userData['kind']
            if kind == 'tank':
                actor = hitbody.userData['actor']
                if actor._pinged != rnd - 1:
                    actor._pinged = rnd
            return kind, angle, dist
        else:
            
            
            return 'w', angle, 0

    def step(self):
        
        
        self.w.Step(self.timeStep, self.velIterations, self.posIterations)
        self.do_destroy()
        self.showit()


    def showit(self):
        for name, tank in self.tanks.items():
            r = tank.body
            tank.turretcontrol()
            
            
            pos2 = r.position
            ang = r.angle

            turret = tank.turretjoint
            tang = turret.GetJointAngle()

            
            

            tank.v.setpos(pos2)
            tank.v.set_rotation(-ang)

            
            tank.v.set_turr_rot(-tang)

            if tank._cannonheat > 0:
                tank._cannonheat -= conf.cannon_cooling_per_tick
            if tank._cannonreload > 0:
                tank._cannonreload -= 1

        for bullet in self.bullets:
            b = bullet.body
            pos2 = b.position
            bullet.v.setpos(pos2)
            

            if bullet._fuse is not None:
                bullet._fuse -= 1
                if bullet._fuse == 0:
                    print 'shell explodes'
                    bullet.explode()

            if bullet._exploding:
                if bullet._exploding > 2:
                    if bullet not in self.to_destroy:
                        self.to_destroy.append(bullet)
                else:
                    bullet._exploding += 1

        
        self.v.step()

    def do_destroy(self):
        while self.to_destroy:
            model = self.to_destroy.pop()
            body = model.body
            if hasattr(body, 'iswall') and body.iswall:
                continue
            
            if model in self.bullets:
                self.bullets.remove(model)
                if model._exploding:
                    model.e.kill()
            
            model.v.kill()

            if model.body.userData['kind'] == 'tank':
                self.w.DestroyBody(model.turret)
                del self.tanks[model.name]
            
            
            self.w.DestroyBody(body)
            




    def make_testtanks(self):
        self.maketank('R1', (4, 0), pi)
        self.maketank('R2', (-4, 0), 0)
        self.maketank('R3', (0, 4), pi)
        self.maketank('R4', (0, -4), 0)

        self.maketank('R5', (4, 4), pi)
        self.maketank('R6', (-4, 4), 0)
        self.maketank('R7', (-4, -4), pi)
        self.maketank('R8', (4, -4), 0)

        self.maketank('R1')
        self.maketank('R2')
        self.maketank('R3')
        self.maketank('R4')

        self.maketank('R5')
        self.maketank('R6')
        self.maketank('R7')
        self.maketank('R8')

    def testmoves(self):
        self.count -= 1
        if self.count < 0:
            self.force = -self.force
            self.count = 1000

        for name, tank in self.tanks.items():
            r = tank.body
            pos = r.position
            vel = r.linearVelocity

            
            

            localforce = box2d.b2Vec2(self.force, 0)
            worldforce = r.GetWorldVector(localforce)

            r.ApplyForce(worldforce, pos)

            
                
            
                

            r.ApplyTorque(4)

            bullet = random.randrange(3)
            if bullet == 2:
                
                self.makebullet(name)


class CL(box2d.b2ContactListener):
    def Result(self, result):
        s1 = result.shape1
        b1 = s1.GetBody()
        actor1 = b1.userData['actor']
        kind1 = b1.userData.get('kind', None)

        s2 = result.shape2
        b2 = s2.GetBody()
        actor2 = b2.userData['actor']
        kind2 = b2.userData.get('kind', None)

        dmg = 0
        hitdmg = conf.direct_hit_damage
        cds = conf.collision_damage_start
        cdf = conf.collision_damage_factor
        nimpulse = result.normalImpulse
        timpulse = result.tangentImpulse
        impulse = box2d.b2Vec2(nimpulse, timpulse).Length()
        coldmg = int((cdf * (impulse - cds))**2) + 1

        if kind2=='tank':
            if kind1=='bullet':
                ring = s1.userData.get('ring', None)
                shooter = b1.userData['shooter']
                if ring is None and shooter == actor2:
                    
                    pass
                elif ring is None:
                    dmg = hitdmg
                    print 'Tank', actor2.name, 'shot for', dmg,
                else:
                    hits = s1.userData['hits']
                    if actor2 not in hits[ring]:
                        dmg = conf.explosion_damage[ring]
                        print 'Tank', actor2.name, 'in blast area for', dmg
                        hits[ring].append(actor2)
                    else:
                        pass
                        
            else:
                shooter = None
                if impulse > cds:
                    dmg = coldmg
                    print 'Tank', actor2.name, 'coll for', dmg,

            if dmg:
                actor2.health -= dmg
                if shooter is not None:
                    shooter._damage_caused += dmg
                actor2.i.health.step(dmg)
                if actor2.health <= 0:
                    actor2.alive = False
                    if conf.remove_dead_tanks:
                        if actor2 not in self.w.to_destroy:
                            self.w.to_destroy.append(actor2)
                    print
                else:
                    print 'down to', actor2.health

        if kind1=='tank':
            if kind2=='bullet':
                ring = s2.userData.get('ring', None)
                shooter = b2.userData['shooter']
                if ring is None and shooter == actor1:
                    
                    pass
                elif ring is None:
                    dmg = hitdmg
                    print 'Tank', actor1.name, 'shot for', dmg,
                else:
                    hits = s2.userData['hits']
                    if actor1 not in hits[ring]:
                        dmg = conf.explosion_damage[ring]
                        print 'Tank', actor1.name, 'in blast area for', dmg
                        hits[ring].append(actor1)
                    else:
                        pass
                        
            else:
                shooter = None
                if impulse > cds:
                    dmg = coldmg
                    print 'Tank', actor1.name, 'coll for', dmg,

            if dmg:
                actor1.health -= dmg
                if shooter is not None:
                    shooter._damage_caused += dmg
                actor1.i.health.step(dmg)
                if actor1.health <= 0:
                    actor1.alive = False
                    if conf.remove_dead_tanks:
                        if actor1 not in self.w.to_destroy:
                            self.w.to_destroy.append(actor1)
                    print
                else:
                    print 'down to', actor1.health

        if actor1 in self.w.bullets and not actor1._exploding:
            if actor1 not in self.w.to_destroy:
                self.w.to_destroy.append(actor1)

        if actor2 in self.w.bullets and not actor2._exploding:
            if actor2 not in self.w.to_destroy:
                self.w.to_destroy.append(actor2)



if __name__ == '__main__':
    w = World()
    cl = CL()
    w.w.SetContactListener(cl)
    cl.w = w
    while not w.v.quit:
        w.step()
