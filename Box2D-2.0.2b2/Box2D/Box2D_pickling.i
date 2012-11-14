/*
* Python SWIG interface file for Box2D (www.box2d.org)
*
* Copyright (c) 2008 kne / sirkne at gmail dot com
* 
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

%pythoncode %{
class b2PickleError (Exception): pass

def _pickle_fix_value_load(lists, value):
    """
    Returns the appropriate object (a b2Body, b2Shape, b2Joint, b2Controller)
    based on the indices in the passed-in dictionary.
    """
    bodyList, jointList, controllerList=lists
    
    if not isinstance(value, dict):
        return value
    elif 'pickle_type' not in value:
        return value

    # Depending on the type, use the right list
    if value['pickle_type']=='b2Body':
        return bodyList[ value['body'] ]

    elif value['pickle_type']=='b2Shape':
        body  = bodyList[ value['body'] ]
        shape = body.shapeList[ value['shape'] ]
        return shape

    elif value['pickle_type']=='b2Joint':
        return jointList[ value['joint'] ]

    elif value['pickle_type']=='b2Controller':
        return controllerList[ value['controller'] ]

    return value

def _pickle_fix_value_save(lists, value):
    """
    Fixes: b2Body, b2Shape, b2Joint, b2Controller

    In place of an unpicklable b2Body outside of a world, use a dictionary with
    an index to the appropriate place in the world.
    """
    bodyList, jointList, controllerList=lists

    if isinstance(value, b2Body):
        value = { 'pickle_type' : 'b2Body', 'body' : bodyList.index(value) }
    elif isinstance(value, b2Shape):
        body = value.GetBody()
        shapeID = body.shapeList.index(value)
        value = { 'pickle_type' : 'b2Shape', 'body': bodyList.index(body), 'shape' : shapeID}
    elif isinstance(value, b2Joint):
        value = { 'pickle_type' : 'b2Joint',  'joint': jointList.index(value) }
    elif isinstance(value, b2Controller):
        value = { 'pickle_type' : 'b2Controller', 'controller' : controllerList.index(value)}

    return value

def pickle_fix(world, var, func='save', lists=None):
    """
    Fix variables so that they may be pickled (or loaded from a pickled state).
    You cannot save a b2Body by itself, but if passed in with the world, it's possible
    to pickle it.

    So, be sure to use this on your box2d-related variables before and after pickling.

    e.g.,
    + Save:
      my_pickled_vars = box2d.pickle_fix(myworld, my_vars, 'save')
      pickle.dump([myworld, my_pickled_vars], open(fn, 'wb'))

    + Load
      world, my_pickled_vars = pickle.load(open(fn, 'rb'))
      myworld = world._pickle_finalize()
      my_vars=box2d.pickle_fix(myworld, my_pickled_vars, 'load')

    For an actual implementation of pickling, see the testbed (main test and test_pickle).
    """
    if func=='save':
        fix_function=_pickle_fix_value_save
    elif func=='load':
        fix_function=_pickle_fix_value_load
    else:
        raise ValueError('Expected func in ("save", "load")')

    if not lists:
        # these lists are all created dynamically, so do this once
        lists=[world.bodyList, world.jointList, world.controllerList]

    if isinstance(var, (list, tuple)):
        # Create a new list/tuple and fix each item
        new_list=[pickle_fix(world, value, func, lists) for value in var]
        if isinstance(var, tuple):
            # If it was originally a tuple, make this new list a tuple
            new_list=tuple(new_list)
        return new_list
    elif isinstance(var, dict):
        if func=='load' and 'pickle_type' in var:
            # Loading a dictionary placeholder for an object
            return fix_function(lists, var)

        # Create a new dictionary and fix each item
        new_dict={}
        for var, value in list(var.items()):
            new_dict[var]=pickle_fix(world, value, func, lists)
        return new_dict
    else:
        # Not a dictionary/list, so it is probably just a normal value. 
        # Fix and return it.
        ret= fix_function(lists, var)
        return ret

# -- unpicklable object --
def no_pickle(self):
    raise b2PickleError('Cannot pickle this object. Pickle the typecasted object: object.getAsType()')

# -- generic get and set state --
def _generic_setstate(self, dict):
    """
    Takes each variable=value pair in the dictionary and
    sets the attributes based on them
    """
    self.__init__()
    for key, value in dict.items():
        setattr(self, key, value)

def _generic_getstate(self, additional_ignore=[]):
    """
    Returns a dictionary representation of self, with 
     dict(var=value [, ...])

    additional_ignore can be specified to ignore certain
    properties. 
    """
    ignore_properties = ['thisown', 'this', 'next', 'prev', 
                         'world', 'coreVertices', 'normals']
    if additional_ignore:
        ignore_properties += additional_ignore

    vars = [v for v in dir(self.__class__) 
        if isinstance(getattr(self.__class__, v), property) 
            and v not in ignore_properties]
    return dict((var, getattr(self, var)) for var in vars)

# -- factory output -- (i.e., b2Body, 
def _pickle_factory_set(self, data):
    """
    The factory output cannot be created just yet,
    so store the necessary information to create it later.
    """
    self.__pickle_data__ = data

# -- factory output finalizing (loading)
def _pickle_finalize(self, world=None, body=None):
    """
    Finalize one of the outputs that we previously set as a 
    dictionary.
    """
    if not hasattr(self, '__pickle_data__'):
        raise b2PickleError("Invalid object passed to _pickle_finalize")

    # At this point, 'self' is invalid on the SWIG-end of things.
    # __init__ has not been called, so it only exists on the Python-end.

    # The previously saved-data
    data = self.__pickle_data__
    
    # These types are what are passed in:
    #                 create_function          output          input
    pairs = [ (lambda w,b,v: w.CreateBody(v) , b2Body        , b2BodyDef),
              (lambda w,b,v: b.CreateShape(v), b2PolygonShape, b2PolygonDef),
              (lambda w,b,v: b.CreateShape(v), b2CircleShape , b2CircleDef),
              (lambda w,b,v: b.CreateShape(v), b2EdgeChainDef, b2EdgeChainDef),
            ]
    
    createfcn = None

    # Create a new instance of the definition so that it may re-create
    # the object.
    for fcn, output, input in pairs:
        if isinstance(self, output):
            try:
                self=input()
            except:
                # Eval worked in some old or new Python, I can't remember which...
                self=eval(input())
            createfcn=fcn
            break

    if not createfcn:
        raise b2PickleError("Invalid object passed to _pickle_finalize")

    # A few things exist that cannot be set in the definition and can only
    # be set after the object is created. Check for these and then set them
    # after if necessary.
    do_after_classes=(b2Body, b2Shape, list)
    do_after_props  =['linearVelocity', 'angularVelocity', 'isSleeping']
    finalize_after = []
    
    if isinstance(self, (b2PolygonDef, b2EdgeChainDef)):
        # Polygon/edge shape. Set their vertices first, as normally they would
        # be put in the 'do after' section
        self.vertices = data['vertices']
        del data['vertices']

    for var in data:
        value = data[var]
        if isinstance(value, do_after_classes) or var in do_after_props:
            # Set these after creation
            finalize_after.append( (var, value) )
        elif hasattr(self, var):
            setattr(self, var, value)

    # Create the actual object (not just the definition)
    self = createfcn(world, body, self)

    # If we just created a body, set that for the upcoming recursion.
    # Finalizing the shape will require that this be set.
    if isinstance(self, b2Body):
        body = self

    for var, value in finalize_after:
        if var == 'shapeList':
            # A shapelist is a special case, do it separately
            _pickle_finalize_shapelist(world, body, value)
        elif var == 'isSleeping':
            # Sleeping is only modifiable by functions, and as such is a special case
            if value:
                self.PutToSleep()
            else:
                self.WakeUp()
        elif hasattr(self, var):
            # The attribute exists, so set it.
            if hasattr(value, '_pickle_finalize'):
                # But first finalize it if necessary.
                value=_pickle_finalize(value,world,body)
            setattr(self, var, value)

    return self

# -- custom handlers --
def _pickle_finalize_controller(data, world):
    """
    Finalize a controller. It's mostly standard, just
    requires a custom bodyList.
    """
    defn = globals()["b2%sControllerDef" % data['_type']] ()

    bodyList  = world.bodyList
    for var in data:
        value = data[var]
        if hasattr(defn, var):
            setattr(defn, var, value)

    # Create the controller
    controller = world.CreateController(defn)

    # And now add the bodies to it
    for body in data['bodyList']:
        try:
            real_body = bodyList[ int(body) ]
        except:
            raise b2PickleError('World not initialized properly; unable to create controller')
        controller.AddBody(real_body)

    return controller

def _pickle_finalize_joint(data, world):
    """
    Finalize a joint.
    The bodies and joints need to be retrieved from the world list
    in order to make the joint.
    """

    defn = globals()["b2%sJointDef" % data['_type']] ()

    body_names  = ['body1' , 'body2' ]
    joint_names = ['joint1', 'joint2']

    bodyList  = world.bodyList
    jointList = world.jointList

    for var in data:
        value = data[var]

        if var=='localXAxis1': 
            var = 'localAxis1' # single rename necessary

        if not hasattr(defn, var):
            continue

        if var in body_names:
            # Set the body based on the global body list
            try:
                value = bodyList[ int(value) ]
            except:
                raise b2PickleError('World not initialized properly; unable to create joint')
        elif var in joint_names:
            # Set the joint based on the global joint list
            try:
                value = jointList[ int(value) ]
            except:
                raise b2PickleError('World not initialized properly; unable to create joint')

        # Set the value
        setattr(defn, var, value)

    return world.CreateJoint(defn)

def _pickle_finalize_shapelist(world, body, shapelist):
    """
    Finalize the shape list for a body.
    Only reason this has to be implemented separately is because of the
    way edge chains are implemented.
    """
    for s in shapelist:
        if isinstance(s, dict):
            # Special case, an edge shape. pickled as a
            # dictionary. Create a fake definition and finalize it.
            temp=b2EdgeChainDef()
            temp.__pickle_data__=s
            _pickle_finalize(temp, world, body)
        else:
            s._pickle_finalize(world, body)

def _pickle_finalize_world(self):
    """
    Finalize a b2World.
    """
    if not hasattr(self, '__pickle_data__'):
        raise b2PickleError('Finalizing a world that was not loaded?')

    data = self.__pickle_data__
    
    # Create the world. Only 3 parameters to deal with.
    world = b2World(data['worldAABB'], data['gravity'], data['doSleep'])

    # First, deal with the ground body. It cannot be taken care of as a
    # normal body since we do not create it; the constructor of the world
    # on the C++-side creates it.
    gb_data = data['groundBody'].__pickle_data__

    # Finalize its shapelist
    _pickle_finalize_shapelist(world, world.groundBody, gb_data['shapeList'])

    # And then go through each variable, setting the properties
    for var in list(gb_data.keys()):
        value = gb_data[var]
        if isinstance(value, (b2Shape)) or var=='shapeList':
            pass
        elif hasattr(world.groundBody, var):
            try:
                setattr(world.groundBody, var, value)
            except AttributeError:
                pass

    # Finalize each body
    for body in data['bodyList']:
        body._pickle_finalize(world)

    # Finalize each joint
    for joint in data['jointList']:
        _pickle_finalize_joint(joint, world)

    # Finalize each controller
    for controller in data['controllerList']:
        _pickle_finalize_controller(controller, world)

    # And that is it. :)
    return world

def _pickle_body_getstate(self):
    """
    Everything is essentially generic_getstate,
     except for edge shape handling.

    TODO: I can see a possible issue in this if joints are used on
    an edge shape or a body with edge shapes and other shapes.
    The shape list order could be improperly recreated. We'll see
    if anything happens...
    """

    def get_edge_vertices_and_shapes(shape):
        """
        Determine whether or not the edge is a loop.
        Also, return each shape that this passes through.
        Then return the vertices.

        Returns is_loop, shapes, vertices
        """

        vertices = []
        shapes   = []
        edge     = shape
        while edge:
            shapes.append(edge)
            vertices.append( edge.vertex1 )
            last=edge.vertex2

            # Go to the next edge
            edge=edge.next
            if edge==shape: # A loop
                return True, shapes, vertices

        # Not a loop
        vertices.append( last )
        return False, shapes, vertices
        
    # Get all the basic attributes except for shapeList
    ret = _generic_getstate(self, ['shapeList'])
    
    # Now check each shape in the list
    ret['shapeList']=[]
    handled_edges = []
    for shape in self.shapeList:
        if isinstance(shape, b2EdgeShape):
            if shape in handled_edges:
                # This edge was already added from another one
                # because they were linked together.
                continue

            is_loop, shapes, vertices=get_edge_vertices_and_shapes(shape)
            handled_edges.extend(shapes)

            # Create a dictionary for this edge shape
            # (to be finalized in _pickle_finalize_shapelist when loaded)
            shape_info = _generic_getstate(shape, ['vertices','length','coreVertex1','coreVertex2'])
            shape_info['isALoop'] =is_loop
            shape_info['vertices']=vertices
            ret['shapeList'].append(shape_info)
        else:
            # Regular shapes need no extra processing
            ret['shapeList'].append(shape)
    return ret

def _pickle_get_b2world(self):
    """
    Get the state of the world.
    """

    # The basic properties first
    vars = ['worldAABB', 'gravity', 'doSleep', 'groundBody']
    data=dict((var, getattr(self, var)) for var in vars)

    # Now the body list (excepting the ground body)
    data['bodyList']=self.bodyList[1:]

    # Add all joints, ensuring to downcast to the appropriate type
    jointList = []
    for joint in self.jointList:
        joint=joint.getAsType()
        jointList.append( joint.__getstate__(self) )
    data['jointList']=jointList

    # Add all controllers, ensuring to downcast to the appropriate type
    controllerList = []
    for controller in self.controllerList:
        controller=controller.getAsType()
        controllerList.append( controller.__getstate__(self) )
    data['controllerList']=controllerList

    return data

def _pickle_get_controller(self, world=None):
    """
    Get the state of a controller
    """
    if not world:
        raise b2PickleError("Controllers can't be saved without the world itself")

    ignore_prop =['thisown', 'this', 'bodyList']
    defn = globals()[ "%sDef" % self.__class__.__name__ ]

    # Take the available variables in the _definition_
    # and then create a dictionary
    vars = [v for v in dir(defn) 
        if isinstance(getattr(defn, v), property) 
            and v not in ignore_prop]

    ret=dict((var, getattr(self, var)) for var in vars)

    # Set the type, so we know how to recreate it
    ret['_type'] = self.typeName()

    # Then use indices into the world body list to store 
    # the bodies controlled by this controller
    main_bodyList = world.bodyList
    ctrl_bodyList = self.bodyList
    ret['bodyList']=[main_bodyList.index(body) for body in ctrl_bodyList]
    return ret

def _pickle_get_joint(self, world=None):
    """
    Get the state of a joint.
    """
    if not world:
        raise b2PickleError("Joints can't be saved without the world itself")

    # Take the available variables in the _definition_
    # and then create a dictionary
    ignore_prop =['thisown', 'this', 'world', 'type']
    defn = globals()[ "%sDef" % self.__class__.__name__ ]
    vars = [v for v in dir(defn) 
        if isinstance(getattr(defn, v), property) 
            and v not in ignore_prop]

    # A single rename is necessary. If localAxis1 (definition) exists,
    # rename it to localXAxis1 (joint)
    if 'localAxis1' in vars: # prismatic, line joints
        vars.remove('localAxis1')
        vars.append('localXAxis1')

    ret=dict((var, getattr(self, var)) for var in vars)
    ret['_type'] = self.typeName()

    # Recreate the body/joint lists.
    bodyList = world.bodyList
    jointList= world.jointList
    for key, value in ret.items():
        if isinstance(value, b2Body):
            ret[key]=bodyList.index(value)
        elif isinstance(value, b2Joint):
            ret[key]=jointList.index(value)

    return ret

%}

# These were originally set programmatically, which is perhaps cleaner,
# but this will set every class up properly without any unnecessary code
# execution

%extend b2World {
 %pythoncode %{
  __getstate__=_pickle_get_b2world
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_world
 %}
}
            

%extend b2PrismaticJoint {
 %pythoncode %{
  __getstate__=_pickle_get_joint
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_joint
 %}
}
            

%extend b2ContactID {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ShapeDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2GravityController {
 %pythoncode %{
  __getstate__=_pickle_get_controller
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_controller
 %}
}
            

%extend b2LineJointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Body {
 %pythoncode %{
  __getstate__=_pickle_body_getstate
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize
 %}
}
            

%extend b2TensorDampingController {
 %pythoncode %{
  __getstate__=_pickle_get_controller
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_controller
 %}
}
            

%extend b2ManifoldPoint {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Color {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2BodyDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Version {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Joint {
 %pythoncode %{
  __getstate__=no_pickle
  __setstate__=_generic_setstate
 %}
}
            

%extend b2JointEdge {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactListener {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2StackEntry {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactManager {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Bound {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Segment {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2DebugDraw {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ConstantForceControllerDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Pair {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2RevoluteJoint {
 %pythoncode %{
  __getstate__=_pickle_get_joint
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_joint
 %}
}
            

%extend b2BuoyancyController {
 %pythoncode %{
  __getstate__=_pickle_get_controller
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_controller
 %}
}
            

%extend b2EdgeShape {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize
 %}
}
            

%extend b2PolygonDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2XForm {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2DistanceJoint {
 %pythoncode %{
  __getstate__=_pickle_get_joint
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_joint
 %}
}
            

%extend b2Contact {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2EdgeChainDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Vec2 {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Vec3 {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2BoundaryListener {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2GravityControllerDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactFilter {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ConstantForceController {
 %pythoncode %{
  __getstate__=_pickle_get_controller
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_controller
 %}
}
            

%extend b2PairCallback {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ConstantAccelControllerDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2MassData {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2BuoyancyControllerDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2AABB {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Mat22 {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactID_features {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactRegister {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Sweep {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Controller {
 %pythoncode %{
  __getstate__=no_pickle
  __setstate__=_generic_setstate
 %}
}
            

%extend b2PulleyJoint {
 %pythoncode %{
  __getstate__=_pickle_get_joint
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_joint
 %}
}
            

%extend b2BufferedPair {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2JointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2DestructionListener {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2CircleDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ControllerEdge {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2PolygonShape {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize
 %}
}
            

%extend b2Manifold {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2GearJoint {
 %pythoncode %{
  __getstate__=_pickle_get_joint
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_joint
 %}
}
            

%extend b2BlockAllocator {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2FilterData {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2RevoluteJointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2MouseJointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2TensorDampingControllerDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Mat33 {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactPoint {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2OBB {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ControllerDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2TimeStep {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2PairManager {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2PulleyJointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactEdge {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2DistanceJointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ContactResult {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2ConstantAccelController {
 %pythoncode %{
  __getstate__=_pickle_get_controller
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_controller
 %}
}
            

%extend b2StackAllocator {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2MouseJoint {
 %pythoncode %{
  __getstate__=_pickle_get_joint
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_joint
 %}
}
            

%extend b2Proxy {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2BroadPhase {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2PrismaticJointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2CircleShape {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize
 %}
}
            

%extend b2LineJoint {
 %pythoncode %{
  __getstate__=_pickle_get_joint
  __setstate__=_pickle_factory_set
  _pickle_finalize=_pickle_finalize_joint
 %}
}
            

%extend b2GearJointDef {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Jacobian {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2NullContact {
 %pythoncode %{
  __getstate__=_generic_getstate
  __setstate__=_generic_setstate
 %}
}
            

%extend b2Shape {
 %pythoncode %{
  __getstate__=no_pickle
  __setstate__=_generic_setstate
 %}
}
            

