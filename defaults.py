
subproc_python = '/usr/bin/python'
subproc_main = 'control.py'

init_timeout = 1.0
tick_timeout = 0.015



tank_dirs = ['tanks', 'tanks/examples'] 
t1 = 'tank01'
t2 = 'tank02'
t3 = 'tank03'
t4 = 'tank04'
t5 = 'tank05'
t6 = 'tank06'
t7 = 'tank07'
tanks = [t2, t4, t5, t6, t7]

logdir = 'logs'

template = 'tanks/template.py'

lineups = 'tanks/lineups'



maxtime = 600 

maxhealth = 100
direct_hit_damage = 10

explosion_radii = [1, 2, 3] 
explosion_damage = [3, 4, 5] 

collision_damage_start = 25 
collision_damage_factor = 0.15 

remove_dead_tanks = True

graphical_display = True




maxforce = 5
maxtorque = 15

tank_density = 1

tank_linearDamping = 1.5
tank_angularDamping = 3.0

tank_friction = 0.3
tank_restitution = 0.4


cannon_reload_ticks = 15 
cannon_maxheat = 100
cannon_heating_per_shot = 20
cannon_cooling_per_tick = 0.1
overheat_fire_reload_penalty = 0 
unloaded_fire_reload_penalty = 0 


turret_maxMotorTorque = 10.0



bulletspeed = 40

bullet_density = .3



dbfile = 'stats.db'

