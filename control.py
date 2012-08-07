import os
from threading import Thread
from time import sleep

from utilities import defaultNonedict

import conf


_overtime_count = 0

def loop(r, i):
    data = i.split('|')
    sensors = defaultNonedict()
    for d in data:
        k, v = d.split(':')

        if ';' in v:
            
            v = v.split(';')
            vconv = []
            for vv in v:
                try:
                    vvconv = int(vv)
                except:
                    vvconv = vv

                vconv.append(vvconv)

        else:
            try:
                vconv = int(v)
            except:
                vconv = v

        sensors[k] = vconv

    timeout = conf.tick_timeout

    user_thread = Thread(target=get_response, args=(r, sensors))
    response = None
    user_thread.start()

    user_thread.join(timeout)
    if user_thread.isAlive():
        global _overtime_count
        _overtime_count += 1
        response = 'TIMEOUT'
        if _overtime_count > 10:
            os.kill(os.getpid(), 9)
    else:
        _overtime_count = 0

    return response or r.response

def get_response(r, sensors):
    try:
        r.sensors = sensors
        r.respond()
    except Exception, e:
        r.err()
        import traceback
        tb = traceback.format_exc()
        r.log(tb)

def communicate(r):
    while True:
        line = sys.stdin.readline().strip()
        if line == 'FINISH':
            break

        o = loop(r, line)
        if o is not None:
            oline = '%s\n' % (str(o))
            try:
                sys.stdout.write(oline)
                sys.stdout.flush()
            except IOError:
                break
        else:
            oline = 'END\n'
            try:
                sys.stdout.write(oline)
                sys.stdout.flush()
            except IOError:
                pass
            break


def build_tank(modname, tankname, testmode, rbox):

    if testmode:
        logfilename = '%s.log' % tankname
        logfilepath = os.path.join(conf.logdir, logfilename)
        logfile = open(logfilepath, 'a')
    else:
        logfile = None

    try:
        mod = __import__(modname)
        r = mod.TheTank(tankname)

        r.logfile = logfile

        r.initialize()

    except:
        rbox.append(None)

        import traceback
        tb = traceback.format_exc()
        if logfile is not None:
            logfile.write(tb)
            logfile.write('\n')
            logfile.flush()
        else:
            import sys
            sys.stderr.write(tb)
            sys.stderr.write('\n')
            sys.stderr.flush()

    else:
        rbox.append(r)


if __name__ == '__main__':
    import sys
    for d in conf.tank_dirs:
        sys.path.append(d)

    if len(sys.argv) != 4:
        raise SystemExit
    else:
        modname = sys.argv[1]
        tankname = sys.argv[2]
        testmode = bool(int(sys.argv[3]))

        timeout = conf.init_timeout

        rbox = [] 
        user_thread = Thread(target=build_tank, args=(modname, tankname, testmode, rbox))
        user_thread.start()

        user_thread.join(timeout)
        if user_thread.isAlive():
            rbox = [None]

        tank = rbox[0]

        if tank is None:
            
            oline = 'ERROR\n'
            sys.stdout.write(oline)
            sys.stdout.flush()

        else:
            oline = 'START\n'
            sys.stdout.write(oline)
            sys.stdout.flush()
            try:
                communicate(tank)
            except KeyboardInterrupt:
                pass
