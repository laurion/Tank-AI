#!/usr/bin/env python
import time

try:
    import conf
except ImportError:
    import utilities
    utilities.makeconf()

    import stats
    stats.dbcheck()

    raise SystemExit

import selectvisual

if __name__ == '__main__':
    import sys
    import os

    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-T", "--testmode", dest="testmode",
                    action="store_true", default=False,
                    help="run in test mode")
    parser.add_option("-t", "--tournament", dest="tournament",
                    action="store_true", default=False,
                    help="run a tournament")
    parser.add_option("-n", "--battles", dest="nbattles",
                    action="store", type='int', default=5,
                    help="number of battles in tournament")
    parser.add_option("-g", "--no-graphics", dest="nographics",
                    action="store_true", default=False,
                    help="non graphics mode")
    parser.add_option("-D", "--upgrade-db", dest="upgrade_db",
                    action="store_true", default=False,
                    help="upgrade database (WARNING! Deletes database!)")

    (options, args) = parser.parse_args()

    testmode = options.testmode
    tournament = options.tournament
    nbattles = options.nbattles
    nographics = options.nographics
    upgrade_db = options.upgrade_db


    if nographics:
        selectvisual.select_view_module('none')
    else:
        selectvisual.select_view_module('pygame')

view = selectvisual.get_view_module()

from game import Game

import world
from world import box2d

import stats


def dbcheck():
    if not stats.dbcheck():
        print 'Run TankAI with -D switch.'
        print 'WARNING: This will delete your current database!'
        import sys
        sys.exit(0)



def runmain():
    if upgrade_db:
        stats.dbremove()
        stats.initialize()

    dbcheck()

    global testmode
    if testmode:
        if not os.path.exists(conf.logdir):
            print 'Log directory does not exist:', conf.logdir
            print 'test mode disabled'
            testmode = False

    stats.dbopen()

    if tournament:
        import datetime
        dt = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print 'Beginning tournament with %s battles.' % nbattles
        for battle in range(nbattles):
            print 'Battle', battle+1
            game = Game(testmode, dt)
            game.run()
            world.Tank.ntanks = 0
            view.Tank.ntanks = 0

        results = stats.tournament_results(dt)
        print;print;print;
        print 'Tournament Results'
        print nbattles, 'battles between', len(results), 'tanks'
        print
        for line in results:
            print line[1], ':', line[4], 'wins', ':', line[6], 'defeated', ':', line[7], 'dmg caused'

    else:
        game = Game(testmode)
        game.run()

    stats.dbclose()

    
    if not testmode and os.path.exists(conf.logdir):
        for f in os.listdir(conf.logdir):
            fpath = os.path.join(conf.logdir, f)
            os.remove(fpath)


if __name__ == '__main__':
    try:
        runmain()
    except KeyboardInterrupt:
        pass

