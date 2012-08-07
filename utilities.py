import os

def makeconf():
    conf_file = 'conf.py'

    if os.path.exists(conf_file):
        print 'conf.py already exists'
        raise SystemExit

    contents = '''\
from defaults import *


'''

    f = open(conf_file, 'w')
    f.write(contents)
    f.close()

    print 'conf.py created'


from collections import defaultdict

class defaultNonedict(defaultdict):
    def __missing__(self, key):
        return None
