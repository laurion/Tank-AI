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

%extend b2AABB {
 %pythoncode %{
 def __repr__(self):
  return """b2AABB(
    lowerBound = %s,
    upperBound = %s,
    IsValid()  = %s)"""% tuple(str(a) for a in\
   (self.lowerBound,self.upperBound,self.IsValid()))
 %}
}

%extend b2BlockAllocator {
 %pythoncode %{
 def __repr__(self):
  return "b2BlockAllocator()"
 %}
}

%extend b2Body {
 %pythoncode %{
 def __repr__(self):
  return """b2Body(
    allowSleep        = %s,
    angle             = %s,
    angularDamping    = %s,
    angularVelocity   = %s,
    fixedRotation     = %s,
    isBullet          = %s,
    isSleeping        = %s,
    linearDamping     = %s,
    linearVelocity    = %s,
    massData          = %s,
    position          = %s,
    userData          = %s,
    GetInertia()      = %s,
    GetLocalCenter()  = %s,
    GetMass()         = %s,
    GetWorldCenter()  = %s,
    GetXForm()        = %s,
    IsBullet()        = %s,
    IsDynamic()       = %s,
    IsFrozen()        = %s,
    IsFixedRotation() = %s,
    IsSleeping()      = %s,
    IsStatic()        = %s)"""% tuple(str(a) for a in\
   (self.allowSleep,self.angle,self.angularDamping,self.angularVelocity,self.fixedRotation,self.isBullet,self.isSleeping,self.linearDamping,self.linearVelocity,self.massData,self.position,self.userData,self.GetInertia(),self.GetLocalCenter(),self.GetMass(),self.GetWorldCenter(),self.GetXForm(),self.IsBullet(),self.IsDynamic(),self.IsFrozen(),self.IsFixedRotation(),self.IsSleeping(),self.IsStatic()))
 %}
}

%extend b2BodyDef {
 %pythoncode %{
 def __repr__(self):
  return """b2BodyDef(
    allowSleep     = %s,
    angle          = %s,
    angularDamping = %s,
    fixedRotation  = %s,
    isBullet       = %s,
    isSleeping     = %s,
    linearDamping  = %s,
    massData       = %s,
    position       = %s,
    userData       = %s)"""% tuple(str(a) for a in\
   (self.allowSleep,self.angle,self.angularDamping,self.fixedRotation,self.isBullet,self.isSleeping,self.linearDamping,self.massData,self.position,self.userData))
 %}
}

%extend b2Bound {
 %pythoncode %{
 def __repr__(self):
  return """b2Bound(
    proxyId       = %s,
    stabbingCount = %s,
    value         = %s,
    IsLower()     = %s,
    IsUpper()     = %s)"""% tuple(str(a) for a in\
   (self.proxyId,self.stabbingCount,self.value,self.IsLower(),self.IsUpper()))
 %}
}

%extend b2BoundaryListener {
 %pythoncode %{
 def __repr__(self):
  return "b2BoundaryListener()"
 %}
}

%extend b2BroadPhase {
 %pythoncode %{
 def __repr__(self):
  return """b2BroadPhase(
    bounds             = %s,
    freeProxy          = %s,
    pairManager        = %s,
    proxyCount         = %s,
    proxyPool          = %s,
    quantizationFactor = %s,
    queryResultCount   = %s,
    queryResults       = %s,
    querySortKeys      = %s,
    s_validate         = %s,
    timeStamp          = %s,
    worldAABB          = %s)"""% tuple(str(a) for a in\
   (self.bounds,self.freeProxy,self.pairManager,self.proxyCount,self.proxyPool,self.quantizationFactor,self.queryResultCount,self.queryResults,self.querySortKeys,self.s_validate,self.timeStamp,self.worldAABB))
 %}
}

%extend b2BufferedPair {
 %pythoncode %{
 def __repr__(self):
  return """b2BufferedPair(
    proxyId1 = %s,
    proxyId2 = %s)"""% tuple(str(a) for a in\
   (self.proxyId1,self.proxyId2))
 %}
}

%extend b2BuoyancyController {
 %pythoncode %{
 def __repr__(self):
  return """b2BuoyancyController(
    angularDrag     = %s,
    density         = %s,
    gravity         = %s,
    linearDrag      = %s,
    normal          = %s,
    offset          = %s,
    useDensity      = %s,
    useWorldGravity = %s,
    velocity        = %s)"""% tuple(str(a) for a in\
   (self.angularDrag,self.density,self.gravity,self.linearDrag,self.normal,self.offset,self.useDensity,self.useWorldGravity,self.velocity))
 %}
}

%extend b2BuoyancyControllerDef {
 %pythoncode %{
 def __repr__(self):
  return """b2BuoyancyControllerDef(
    angularDrag     = %s,
    density         = %s,
    gravity         = %s,
    linearDrag      = %s,
    normal          = %s,
    offset          = %s,
    useDensity      = %s,
    useWorldGravity = %s,
    velocity        = %s)"""% tuple(str(a) for a in\
   (self.angularDrag,self.density,self.gravity,self.linearDrag,self.normal,self.offset,self.useDensity,self.useWorldGravity,self.velocity))
 %}
}

%extend b2CircleDef {
 %pythoncode %{
 def __repr__(self):
  return """b2CircleDef(
    density       = %s,
    filter        = %s,
    friction      = %s,
    isSensor      = %s,
    localPosition = %s,
    radius        = %s,
    restitution   = %s,
    type          = %s,
    userData      = %s)"""% tuple(str(a) for a in\
   (self.density,self.filter,self.friction,self.isSensor,self.localPosition,self.radius,self.restitution,self.type,self.userData))
 %}
}

%extend b2CircleShape {
 %pythoncode %{
 def __repr__(self):
  return """b2CircleShape(
    density          = %s,
    filter           = %s,
    friction         = %s,
    localPosition    = %s,
    radius           = %s,
    restitution      = %s,
    userData         = %s,
    GetBody()        = %s,
    GetFilterData()  = %s,
    GetSweepRadius() = %s,
    GetType()        = %s,
    IsSensor()       = %s)"""% tuple(str(a) for a in\
   (self.density,self.filter,self.friction,self.localPosition,self.radius,self.restitution,self.userData,self.GetBody(),self.GetFilterData(),self.GetSweepRadius(),self.GetType(),self.IsSensor()))
 %}
}

%extend b2Color {
 %pythoncode %{
 def __repr__(self):
  return """b2Color(
    b = %s,
    g = %s,
    r = %s)"""% tuple(str(a) for a in\
   (self.b,self.g,self.r))
 %}
}

%extend b2ConstantAccelController {
 %pythoncode %{
 def __repr__(self):
  return """b2ConstantAccelController(
    A = %s)"""% tuple(str(a) for a in\
   (self.A))
 %}
}

%extend b2ConstantAccelControllerDef {
 %pythoncode %{
 def __repr__(self):
  return """b2ConstantAccelControllerDef(
    A = %s)"""% tuple(str(a) for a in\
   (self.A))
 %}
}

%extend b2ConstantForceController {
 %pythoncode %{
 def __repr__(self):
  return """b2ConstantForceController(
    F = %s)"""% tuple(str(a) for a in\
   (self.F))
 %}
}

%extend b2ConstantForceControllerDef {
 %pythoncode %{
 def __repr__(self):
  return """b2ConstantForceControllerDef(
    F = %s)"""% tuple(str(a) for a in\
   (self.F))
 %}
}

%extend b2Contact {
 %pythoncode %{
 def __repr__(self):
  return """b2Contact(
    flags          = %s,
    manifoldCount  = %s,
    node1          = %s,
    node2          = %s,
    s_initialized  = %s,
    s_registers    = %s,
    shape1         = %s,
    shape2         = %s,
    toi            = %s,
    GetManifolds() = %s,
    IsSolid()      = %s,
    e_islandFlag   = %s,
    e_nonSolidFlag = %s,
    e_slowFlag     = %s,
    e_toiFlag      = %s)"""% tuple(str(a) for a in\
   (self.flags,self.manifoldCount,self.node1,self.node2,self.s_initialized,self.s_registers,self.shape1,self.shape2,self.toi,self.GetManifolds(),self.IsSolid(),self.e_islandFlag,self.e_nonSolidFlag,self.e_slowFlag,self.e_toiFlag))
 %}
}

%extend b2ContactEdge {
 %pythoncode %{
 def __repr__(self):
  return """b2ContactEdge(
    contact = %s,
    other   = %s)"""% tuple(str(a) for a in\
   (self.contact,self.other))
 %}
}

%extend b2ContactFilter {
 %pythoncode %{
 def __repr__(self):
  return "b2ContactFilter()"
 %}
}

%extend b2ContactID {
 %pythoncode %{
 def __repr__(self):
  return """b2ContactID(
    features = %s,
    key      = %s)"""% tuple(str(a) for a in\
   (self.features,self.key))
 %}
}

%extend b2ContactID_features {
 %pythoncode %{
 def __repr__(self):
  return """b2ContactID_features(
    flip           = %s,
    incidentEdge   = %s,
    incidentVertex = %s,
    referenceEdge  = %s)"""% tuple(str(a) for a in\
   (self.flip,self.incidentEdge,self.incidentVertex,self.referenceEdge))
 %}
}

%extend b2ContactListener {
 %pythoncode %{
 def __repr__(self):
  return "b2ContactListener()"
 %}
}

%extend b2ContactManager {
 %pythoncode %{
 def __repr__(self):
  return """b2ContactManager(
    destroyImmediate = %s,
    nullContact      = %s,
    world            = %s)"""% tuple(str(a) for a in\
   (self.destroyImmediate,self.nullContact,self.world))
 %}
}

%extend b2ContactPoint {
 %pythoncode %{
 def __repr__(self):
  return """b2ContactPoint(
    friction    = %s,
    id          = %s,
    normal      = %s,
    position    = %s,
    restitution = %s,
    separation  = %s,
    shape1      = %s,
    shape2      = %s,
    velocity    = %s)"""% tuple(str(a) for a in\
   (self.friction,self.id,self.normal,self.position,self.restitution,self.separation,self.shape1,self.shape2,self.velocity))
 %}
}

%extend b2ContactRegister {
 %pythoncode %{
 def __repr__(self):
  return """b2ContactRegister(
    createFcn  = %s,
    destroyFcn = %s,
    primary    = %s)"""% tuple(str(a) for a in\
   (self.createFcn,self.destroyFcn,self.primary))
 %}
}

%extend b2ContactResult {
 %pythoncode %{
 def __repr__(self):
  return """b2ContactResult(
    id             = %s,
    normal         = %s,
    normalImpulse  = %s,
    position       = %s,
    shape1         = %s,
    shape2         = %s,
    tangentImpulse = %s)"""% tuple(str(a) for a in\
   (self.id,self.normal,self.normalImpulse,self.position,self.shape1,self.shape2,self.tangentImpulse))
 %}
}

%extend b2Controller {
 %pythoncode %{
 def __repr__(self):
  return "b2Controller()"
 %}
}

%extend b2ControllerDef {
 %pythoncode %{
 def __repr__(self):
  return "b2ControllerDef()"
 %}
}

%extend b2ControllerEdge {
 %pythoncode %{
 def __repr__(self):
  return """b2ControllerEdge(
    body       = %s,
    controller = %s)"""% tuple(str(a) for a in\
   (self.body,self.controller))
 %}
}

%extend b2DebugDraw {
 %pythoncode %{
 def __repr__(self):
  return """b2DebugDraw(
    GetFlags()        = %s,
    e_aabbBit         = %s,
    e_centerOfMassBit = %s,
    e_controllerBit   = %s,
    e_coreShapeBit    = %s,
    e_jointBit        = %s,
    e_obbBit          = %s,
    e_pairBit         = %s,
    e_shapeBit        = %s)"""% tuple(str(a) for a in\
   (self.GetFlags(),self.e_aabbBit,self.e_centerOfMassBit,self.e_controllerBit,self.e_coreShapeBit,self.e_jointBit,self.e_obbBit,self.e_pairBit,self.e_shapeBit))
 %}
}

%extend b2DestructionListener {
 %pythoncode %{
 def __repr__(self):
  return "b2DestructionListener()"
 %}
}

%extend b2DistanceJoint {
 %pythoncode %{
 def __repr__(self):
  return """b2DistanceJoint(
    bias             = %s,
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    dampingRatio     = %s,
    frequencyHz      = %s,
    gamma            = %s,
    impulse          = %s,
    length           = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    mass             = %s,
    type             = %s,
    u                = %s,
    userData         = %s,
    GetAnchor1()     = %s,
    GetAnchor2()     = %s)"""% tuple(str(a) for a in\
   (self.bias,self.body1,self.body2,self.collideConnected,self.dampingRatio,self.frequencyHz,self.gamma,self.impulse,self.length,self.localAnchor1,self.localAnchor2,self.mass,self.type,self.u,self.userData,self.GetAnchor1(),self.GetAnchor2()))
 %}
}

%extend b2DistanceJointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2DistanceJointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    dampingRatio     = %s,
    frequencyHz      = %s,
    length           = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    type             = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.dampingRatio,self.frequencyHz,self.length,self.localAnchor1,self.localAnchor2,self.type,self.userData))
 %}
}

%extend b2EdgeChainDef {
 %pythoncode %{
 def __repr__(self):
  return """b2EdgeChainDef(
    density     = %s,
    filter      = %s,
    friction    = %s,
    isALoop     = %s,
    isSensor    = %s,
    restitution = %s,
    type        = %s,
    userData    = %s,
    vertexCount = %s,
    vertices    = %s)"""% tuple(str(a) for a in\
   (self.density,self.filter,self.friction,self.isALoop,self.isSensor,self.restitution,self.type,self.userData,self.vertexCount,self.vertices))
 %}
}

%extend b2EdgeShape {
 %pythoncode %{
 def __repr__(self):
  return """b2EdgeShape(
    density              = %s,
    filter               = %s,
    friction             = %s,
    restitution          = %s,
    userData             = %s,
    GetBody()            = %s,
    GetCoreVertex1()     = %s,
    GetCoreVertex2()     = %s,
    GetCorner1Vector()   = %s,
    GetCorner2Vector()   = %s,
    GetDirectionVector() = %s,
    GetFilterData()      = %s,
    GetLength()          = %s,
    GetNormalVector()    = %s,
    GetSweepRadius()     = %s,
    GetType()            = %s,
    GetVertex1()         = %s,
    GetVertex2()         = %s,
    IsSensor()           = %s)"""% tuple(str(a) for a in\
   (self.density,self.filter,self.friction,self.restitution,self.userData,self.GetBody(),self.GetCoreVertex1(),self.GetCoreVertex2(),self.GetCorner1Vector(),self.GetCorner2Vector(),self.GetDirectionVector(),self.GetFilterData(),self.GetLength(),self.GetNormalVector(),self.GetSweepRadius(),self.GetType(),self.GetVertex1(),self.GetVertex2(),self.IsSensor()))
 %}
}

%extend b2FilterData {
 %pythoncode %{
 def __repr__(self):
  return """b2FilterData(
    categoryBits = %s,
    groupIndex   = %s,
    maskBits     = %s)"""% tuple(str(a) for a in\
   (self.categoryBits,self.groupIndex,self.maskBits))
 %}
}

%extend b2GearJoint {
 %pythoncode %{
 def __repr__(self):
  return """b2GearJoint(
    J                = %s,
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    constant         = %s,
    ground1          = %s,
    ground2          = %s,
    groundAnchor1    = %s,
    groundAnchor2    = %s,
    impulse          = %s,
    joint1           = %s,
    joint2           = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    mass             = %s,
    prismatic1       = %s,
    prismatic2       = %s,
    ratio            = %s,
    revolute1        = %s,
    revolute2        = %s,
    type             = %s,
    userData         = %s,
    GetAnchor1()     = %s,
    GetAnchor2()     = %s)"""% tuple(str(a) for a in\
   (self.J,self.body1,self.body2,self.collideConnected,self.constant,self.ground1,self.ground2,self.groundAnchor1,self.groundAnchor2,self.impulse,self.joint1,self.joint2,self.localAnchor1,self.localAnchor2,self.mass,self.prismatic1,self.prismatic2,self.ratio,self.revolute1,self.revolute2,self.type,self.userData,self.GetAnchor1(),self.GetAnchor2()))
 %}
}

%extend b2GearJointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2GearJointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    joint1           = %s,
    joint2           = %s,
    ratio            = %s,
    type             = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.joint1,self.joint2,self.ratio,self.type,self.userData))
 %}
}

%extend b2GravityController {
 %pythoncode %{
 def __repr__(self):
  return """b2GravityController(
    G      = %s,
    invSqr = %s)"""% tuple(str(a) for a in\
   (self.G,self.invSqr))
 %}
}

%extend b2GravityControllerDef {
 %pythoncode %{
 def __repr__(self):
  return """b2GravityControllerDef(
    G      = %s,
    invSqr = %s)"""% tuple(str(a) for a in\
   (self.G,self.invSqr))
 %}
}

%extend b2Jacobian {
 %pythoncode %{
 def __repr__(self):
  return """b2Jacobian(
    angular1 = %s,
    angular2 = %s,
    linear1  = %s,
    linear2  = %s)"""% tuple(str(a) for a in\
   (self.angular1,self.angular2,self.linear1,self.linear2))
 %}
}

%extend b2Joint {
 %pythoncode %{
 def __repr__(self):
  return """b2Joint(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    type             = %s,
    userData         = %s,
    GetAnchor1()     = %s,
    GetAnchor2()     = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.type,self.userData,self.GetAnchor1(),self.GetAnchor2()))
 %}
}

%extend b2JointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2JointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    type             = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.type,self.userData))
 %}
}

%extend b2JointEdge {
 %pythoncode %{
 def __repr__(self):
  return """b2JointEdge(
    joint = %s,
    other = %s)"""% tuple(str(a) for a in\
   (self.joint,self.other))
 %}
}

%extend b2LineJoint {
 %pythoncode %{
 def __repr__(self):
  return """b2LineJoint(
    K                     = %s,
    a1                    = %s,
    a2                    = %s,
    axis                  = %s,
    body1                 = %s,
    body2                 = %s,
    collideConnected      = %s,
    enableLimit           = %s,
    enableMotor           = %s,
    impulse               = %s,
    limitState            = %s,
    localAnchor1          = %s,
    localAnchor2          = %s,
    localXAxis1           = %s,
    localYAxis1           = %s,
    lowerTranslation      = %s,
    maxMotorForce         = %s,
    motorImpulse          = %s,
    motorMass             = %s,
    motorSpeed            = %s,
    perp                  = %s,
    s1                    = %s,
    s2                    = %s,
    type                  = %s,
    upperTranslation      = %s,
    userData              = %s,
    GetAnchor1()          = %s,
    GetAnchor2()          = %s,
    GetJointSpeed()       = %s,
    GetJointTranslation() = %s,
    GetLowerLimit()       = %s,
    GetMotorForce()       = %s,
    GetUpperLimit()       = %s,
    IsLimitEnabled()      = %s,
    IsMotorEnabled()      = %s)"""% tuple(str(a) for a in\
   (self.K,self.a1,self.a2,self.axis,self.body1,self.body2,self.collideConnected,self.enableLimit,self.enableMotor,self.impulse,self.limitState,self.localAnchor1,self.localAnchor2,self.localXAxis1,self.localYAxis1,self.lowerTranslation,self.maxMotorForce,self.motorImpulse,self.motorMass,self.motorSpeed,self.perp,self.s1,self.s2,self.type,self.upperTranslation,self.userData,self.GetAnchor1(),self.GetAnchor2(),self.GetJointSpeed(),self.GetJointTranslation(),self.GetLowerLimit(),self.GetMotorForce(),self.GetUpperLimit(),self.IsLimitEnabled(),self.IsMotorEnabled()))
 %}
}

%extend b2LineJointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2LineJointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    enableLimit      = %s,
    enableMotor      = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    localAxis1       = %s,
    lowerTranslation = %s,
    maxMotorForce    = %s,
    motorSpeed       = %s,
    type             = %s,
    upperTranslation = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.enableLimit,self.enableMotor,self.localAnchor1,self.localAnchor2,self.localAxis1,self.lowerTranslation,self.maxMotorForce,self.motorSpeed,self.type,self.upperTranslation,self.userData))
 %}
}

%extend b2Manifold {
 %pythoncode %{
 def __repr__(self):
  return """b2Manifold(
    normal     = %s,
    pointCount = %s,
    points     = %s)"""% tuple(str(a) for a in\
   (self.normal,self.pointCount,self.points))
 %}
}

%extend b2ManifoldPoint {
 %pythoncode %{
 def __repr__(self):
  return """b2ManifoldPoint(
    id             = %s,
    localPoint1    = %s,
    localPoint2    = %s,
    normalImpulse  = %s,
    separation     = %s,
    tangentImpulse = %s)"""% tuple(str(a) for a in\
   (self.id,self.localPoint1,self.localPoint2,self.normalImpulse,self.separation,self.tangentImpulse))
 %}
}

%extend b2MassData {
 %pythoncode %{
 def __repr__(self):
  return """b2MassData(
    I      = %s,
    center = %s,
    mass   = %s)"""% tuple(str(a) for a in\
   (self.I,self.center,self.mass))
 %}
}

%extend b2Mat22 {
 %pythoncode %{
 def __repr__(self):
  return """b2Mat22(
    col1       = %s,
    col2       = %s,
    GetAngle() = %s)"""% tuple(str(a) for a in\
   (self.col1,self.col2,self.GetAngle()))
 %}
}

%extend b2Mat33 {
 %pythoncode %{
 def __repr__(self):
  return """b2Mat33(
    col1 = %s,
    col2 = %s,
    col3 = %s)"""% tuple(str(a) for a in\
   (self.col1,self.col2,self.col3))
 %}
}

%extend b2MouseJoint {
 %pythoncode %{
 def __repr__(self):
  return """b2MouseJoint(
    C                = %s,
    beta             = %s,
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    dampingRatio     = %s,
    frequencyHz      = %s,
    gamma            = %s,
    impulse          = %s,
    localAnchor      = %s,
    mass             = %s,
    maxForce         = %s,
    target           = %s,
    type             = %s,
    userData         = %s,
    GetAnchor1()     = %s,
    GetAnchor2()     = %s)"""% tuple(str(a) for a in\
   (self.C,self.beta,self.body1,self.body2,self.collideConnected,self.dampingRatio,self.frequencyHz,self.gamma,self.impulse,self.localAnchor,self.mass,self.maxForce,self.target,self.type,self.userData,self.GetAnchor1(),self.GetAnchor2()))
 %}
}

%extend b2MouseJointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2MouseJointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    dampingRatio     = %s,
    frequencyHz      = %s,
    maxForce         = %s,
    target           = %s,
    type             = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.dampingRatio,self.frequencyHz,self.maxForce,self.target,self.type,self.userData))
 %}
}

%extend b2NullContact {
 %pythoncode %{
 def __repr__(self):
  return """b2NullContact(
    flags          = %s,
    manifoldCount  = %s,
    node1          = %s,
    node2          = %s,
    s_initialized  = %s,
    s_registers    = %s,
    shape1         = %s,
    shape2         = %s,
    toi            = %s,
    GetManifolds() = %s,
    IsSolid()      = %s,
    e_islandFlag   = %s,
    e_nonSolidFlag = %s,
    e_slowFlag     = %s,
    e_toiFlag      = %s)"""% tuple(str(a) for a in\
   (self.flags,self.manifoldCount,self.node1,self.node2,self.s_initialized,self.s_registers,self.shape1,self.shape2,self.toi,self.GetManifolds(),self.IsSolid(),self.e_islandFlag,self.e_nonSolidFlag,self.e_slowFlag,self.e_toiFlag))
 %}
}

%extend b2OBB {
 %pythoncode %{
 def __repr__(self):
  return """b2OBB(
    R       = %s,
    center  = %s,
    extents = %s)"""% tuple(str(a) for a in\
   (self.R,self.center,self.extents))
 %}
}

%extend b2Pair {
 %pythoncode %{
 def __repr__(self):
  return """b2Pair(
    proxyId1       = %s,
    proxyId2       = %s,
    status         = %s,
    userData       = %s,
    IsBuffered()   = %s,
    IsFinal()      = %s,
    IsRemoved()    = %s,
    e_pairBuffered = %s,
    e_pairFinal    = %s,
    e_pairRemoved  = %s)"""% tuple(str(a) for a in\
   (self.proxyId1,self.proxyId2,self.status,self.userData,self.IsBuffered(),self.IsFinal(),self.IsRemoved(),self.e_pairBuffered,self.e_pairFinal,self.e_pairRemoved))
 %}
}

%extend b2PairCallback {
 %pythoncode %{
 def __repr__(self):
  return "b2PairCallback()"
 %}
}

%extend b2PairManager {
 %pythoncode %{
 def __repr__(self):
  return """b2PairManager(
    broadPhase      = %s,
    callback        = %s,
    freePair        = %s,
    hashTable       = %s,
    pairBuffer      = %s,
    pairBufferCount = %s,
    pairCount       = %s,
    pairs           = %s)"""% tuple(str(a) for a in\
   (self.broadPhase,self.callback,self.freePair,self.hashTable,self.pairBuffer,self.pairBufferCount,self.pairCount,self.pairs))
 %}
}

%extend b2PolygonDef {
 %pythoncode %{
 def __repr__(self):
  return """b2PolygonDef(
    density     = %s,
    filter      = %s,
    friction    = %s,
    isSensor    = %s,
    restitution = %s,
    type        = %s,
    userData    = %s,
    vertexCount = %s,
    vertices    = %s)"""% tuple(str(a) for a in\
   (self.density,self.filter,self.friction,self.isSensor,self.restitution,self.type,self.userData,self.vertexCount,self.vertices))
 %}
}

%extend b2PolygonShape {
 %pythoncode %{
 def __repr__(self):
  return """b2PolygonShape(
    coreVertices     = %s,
    density          = %s,
    filter           = %s,
    friction         = %s,
    normals          = %s,
    restitution      = %s,
    userData         = %s,
    vertices         = %s,
    GetBody()        = %s,
    GetCentroid()    = %s,
    GetFilterData()  = %s,
    GetOBB()         = %s,
    GetSweepRadius() = %s,
    GetType()        = %s,
    GetVertexCount() = %s,
    IsSensor()       = %s)"""% tuple(str(a) for a in\
   (self.coreVertices,self.density,self.filter,self.friction,self.normals,self.restitution,self.userData,self.vertices,self.GetBody(),self.GetCentroid(),self.GetFilterData(),self.GetOBB(),self.GetSweepRadius(),self.GetType(),self.GetVertexCount(),self.IsSensor()))
 %}
}

%extend b2PrismaticJoint {
 %pythoncode %{
 def __repr__(self):
  return """b2PrismaticJoint(
    K                     = %s,
    a1                    = %s,
    a2                    = %s,
    axis                  = %s,
    body1                 = %s,
    body2                 = %s,
    collideConnected      = %s,
    enableLimit           = %s,
    enableMotor           = %s,
    impulse               = %s,
    limitState            = %s,
    localAnchor1          = %s,
    localAnchor2          = %s,
    localXAxis1           = %s,
    localYAxis1           = %s,
    lowerTranslation      = %s,
    maxMotorForce         = %s,
    motorImpulse          = %s,
    motorMass             = %s,
    motorSpeed            = %s,
    perp                  = %s,
    referenceAngle        = %s,
    s1                    = %s,
    s2                    = %s,
    type                  = %s,
    upperTranslation      = %s,
    userData              = %s,
    GetAnchor1()          = %s,
    GetAnchor2()          = %s,
    GetJointSpeed()       = %s,
    GetJointTranslation() = %s,
    GetLowerLimit()       = %s,
    GetMotorForce()       = %s,
    GetUpperLimit()       = %s,
    IsLimitEnabled()      = %s,
    IsMotorEnabled()      = %s)"""% tuple(str(a) for a in\
   (self.K,self.a1,self.a2,self.axis,self.body1,self.body2,self.collideConnected,self.enableLimit,self.enableMotor,self.impulse,self.limitState,self.localAnchor1,self.localAnchor2,self.localXAxis1,self.localYAxis1,self.lowerTranslation,self.maxMotorForce,self.motorImpulse,self.motorMass,self.motorSpeed,self.perp,self.referenceAngle,self.s1,self.s2,self.type,self.upperTranslation,self.userData,self.GetAnchor1(),self.GetAnchor2(),self.GetJointSpeed(),self.GetJointTranslation(),self.GetLowerLimit(),self.GetMotorForce(),self.GetUpperLimit(),self.IsLimitEnabled(),self.IsMotorEnabled()))
 %}
}

%extend b2PrismaticJointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2PrismaticJointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    enableLimit      = %s,
    enableMotor      = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    localAxis1       = %s,
    lowerTranslation = %s,
    maxMotorForce    = %s,
    motorSpeed       = %s,
    referenceAngle   = %s,
    type             = %s,
    upperTranslation = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.enableLimit,self.enableMotor,self.localAnchor1,self.localAnchor2,self.localAxis1,self.lowerTranslation,self.maxMotorForce,self.motorSpeed,self.referenceAngle,self.type,self.upperTranslation,self.userData))
 %}
}

%extend b2Proxy {
 %pythoncode %{
 def __repr__(self):
  return """b2Proxy(
    lowerBounds  = %s,
    overlapCount = %s,
    timeStamp    = %s,
    upperBounds  = %s,
    userData     = %s,
    IsValid()    = %s)"""% tuple(str(a) for a in\
   (self.lowerBounds,self.overlapCount,self.timeStamp,self.upperBounds,self.userData,self.IsValid()))
 %}
}

%extend b2PulleyJoint {
 %pythoncode %{
 def __repr__(self):
  return """b2PulleyJoint(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    constant         = %s,
    ground           = %s,
    groundAnchor1    = %s,
    groundAnchor2    = %s,
    impulse          = %s,
    length1          = %s,
    length2          = %s,
    limitImpulse1    = %s,
    limitImpulse2    = %s,
    limitMass1       = %s,
    limitMass2       = %s,
    limitState1      = %s,
    limitState2      = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    maxLength1       = %s,
    maxLength2       = %s,
    pulleyMass       = %s,
    ratio            = %s,
    state            = %s,
    type             = %s,
    u1               = %s,
    u2               = %s,
    userData         = %s,
    GetAnchor1()     = %s,
    GetAnchor2()     = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.constant,self.ground,self.groundAnchor1,self.groundAnchor2,self.impulse,self.length1,self.length2,self.limitImpulse1,self.limitImpulse2,self.limitMass1,self.limitMass2,self.limitState1,self.limitState2,self.localAnchor1,self.localAnchor2,self.maxLength1,self.maxLength2,self.pulleyMass,self.ratio,self.state,self.type,self.u1,self.u2,self.userData,self.GetAnchor1(),self.GetAnchor2()))
 %}
}

%extend b2PulleyJointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2PulleyJointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    groundAnchor1    = %s,
    groundAnchor2    = %s,
    length1          = %s,
    length2          = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    maxLength1       = %s,
    maxLength2       = %s,
    ratio            = %s,
    type             = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.groundAnchor1,self.groundAnchor2,self.length1,self.length2,self.localAnchor1,self.localAnchor2,self.maxLength1,self.maxLength2,self.ratio,self.type,self.userData))
 %}
}

%extend b2RevoluteJoint {
 %pythoncode %{
 def __repr__(self):
  return """b2RevoluteJoint(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    enableLimit      = %s,
    enableMotor      = %s,
    impulse          = %s,
    limitState       = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    lowerAngle       = %s,
    mass             = %s,
    maxMotorTorque   = %s,
    motorImpulse     = %s,
    motorMass        = %s,
    motorSpeed       = %s,
    referenceAngle   = %s,
    type             = %s,
    upperAngle       = %s,
    userData         = %s,
    GetAnchor1()     = %s,
    GetAnchor2()     = %s,
    GetJointAngle()  = %s,
    GetJointSpeed()  = %s,
    GetLowerLimit()  = %s,
    GetMotorTorque() = %s,
    GetUpperLimit()  = %s,
    IsLimitEnabled() = %s,
    IsMotorEnabled() = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.enableLimit,self.enableMotor,self.impulse,self.limitState,self.localAnchor1,self.localAnchor2,self.lowerAngle,self.mass,self.maxMotorTorque,self.motorImpulse,self.motorMass,self.motorSpeed,self.referenceAngle,self.type,self.upperAngle,self.userData,self.GetAnchor1(),self.GetAnchor2(),self.GetJointAngle(),self.GetJointSpeed(),self.GetLowerLimit(),self.GetMotorTorque(),self.GetUpperLimit(),self.IsLimitEnabled(),self.IsMotorEnabled()))
 %}
}

%extend b2RevoluteJointDef {
 %pythoncode %{
 def __repr__(self):
  return """b2RevoluteJointDef(
    body1            = %s,
    body2            = %s,
    collideConnected = %s,
    enableLimit      = %s,
    enableMotor      = %s,
    localAnchor1     = %s,
    localAnchor2     = %s,
    lowerAngle       = %s,
    maxMotorTorque   = %s,
    motorSpeed       = %s,
    referenceAngle   = %s,
    type             = %s,
    upperAngle       = %s,
    userData         = %s)"""% tuple(str(a) for a in\
   (self.body1,self.body2,self.collideConnected,self.enableLimit,self.enableMotor,self.localAnchor1,self.localAnchor2,self.lowerAngle,self.maxMotorTorque,self.motorSpeed,self.referenceAngle,self.type,self.upperAngle,self.userData))
 %}
}

%extend b2Segment {
 %pythoncode %{
 def __repr__(self):
  return """b2Segment(
    p1 = %s,
    p2 = %s)"""% tuple(str(a) for a in\
   (self.p1,self.p2))
 %}
}

%extend b2Shape {
 %pythoncode %{
 def __repr__(self):
  return """b2Shape(
    density          = %s,
    filter           = %s,
    friction         = %s,
    restitution      = %s,
    userData         = %s,
    GetBody()        = %s,
    GetFilterData()  = %s,
    GetSweepRadius() = %s,
    GetType()        = %s,
    IsSensor()       = %s)"""% tuple(str(a) for a in\
   (self.density,self.filter,self.friction,self.restitution,self.userData,self.GetBody(),self.GetFilterData(),self.GetSweepRadius(),self.GetType(),self.IsSensor()))
 %}
}

%extend b2ShapeDef {
 %pythoncode %{
 def __repr__(self):
  return """b2ShapeDef(
    density     = %s,
    filter      = %s,
    friction    = %s,
    isSensor    = %s,
    restitution = %s,
    type        = %s,
    userData    = %s)"""% tuple(str(a) for a in\
   (self.density,self.filter,self.friction,self.isSensor,self.restitution,self.type,self.userData))
 %}
}

%extend b2StackAllocator {
 %pythoncode %{
 def __repr__(self):
  return """b2StackAllocator(
    GetMaxAllocation() = %s)"""% tuple(str(a) for a in\
   (self.GetMaxAllocation()))
 %}
}

%extend b2StackEntry {
 %pythoncode %{
 def __repr__(self):
  return """b2StackEntry(
    data       = %s,
    size       = %s,
    usedMalloc = %s)"""% tuple(str(a) for a in\
   (self.data,self.size,self.usedMalloc))
 %}
}

%extend b2Sweep {
 %pythoncode %{
 def __repr__(self):
  return """b2Sweep(
    a           = %s,
    a0          = %s,
    c           = %s,
    c0          = %s,
    localCenter = %s,
    t0          = %s)"""% tuple(str(a) for a in\
   (self.a,self.a0,self.c,self.c0,self.localCenter,self.t0))
 %}
}

%extend b2TensorDampingController {
 %pythoncode %{
 def __repr__(self):
  return """b2TensorDampingController(
    T           = %s,
    maxTimestep = %s)"""% tuple(str(a) for a in\
   (self.T,self.maxTimestep))
 %}
}

%extend b2TensorDampingControllerDef {
 %pythoncode %{
 def __repr__(self):
  return """b2TensorDampingControllerDef(
    T           = %s,
    maxTimestep = %s)"""% tuple(str(a) for a in\
   (self.T,self.maxTimestep))
 %}
}

%extend b2TimeStep {
 %pythoncode %{
 def __repr__(self):
  return """b2TimeStep(
    dt                 = %s,
    dtRatio            = %s,
    inv_dt             = %s,
    positionIterations = %s,
    velocityIterations = %s,
    warmStarting       = %s)"""% tuple(str(a) for a in\
   (self.dt,self.dtRatio,self.inv_dt,self.positionIterations,self.velocityIterations,self.warmStarting))
 %}
}

%extend b2Vec2 {
 %pythoncode %{
 def __repr__(self):
  return """b2Vec2(
    x         = %s,
    y         = %s,
    IsValid() = %s)"""% tuple(str(a) for a in\
   (self.x,self.y,self.IsValid()))
 %}
}

%extend b2Vec3 {
 %pythoncode %{
 def __repr__(self):
  return """b2Vec3(
    x = %s,
    y = %s,
    z = %s)"""% tuple(str(a) for a in\
   (self.x,self.y,self.z))
 %}
}

%extend b2Version {
 %pythoncode %{
 def __repr__(self):
  return """b2Version(
    major    = %s,
    minor    = %s,
    revision = %s)"""% tuple(str(a) for a in\
   (self.major,self.minor,self.revision))
 %}
}

%extend b2World {
 %pythoncode %{
 def __repr__(self):
  return """b2World(
    doSleep              = %s,
    gravity              = %s,
    groundBody           = %s,
    worldAABB            = %s,
    GetBodyCount()       = %s,
    GetContactCount()    = %s,
    GetControllerCount() = %s,
    GetJointCount()      = %s,
    GetPairCount()       = %s,
    GetProxyCount()      = %s)"""% tuple(str(a) for a in\
   (self.doSleep,self.gravity,self.groundBody,self.worldAABB,self.GetBodyCount(),self.GetContactCount(),self.GetControllerCount(),self.GetJointCount(),self.GetPairCount(),self.GetProxyCount()))
 %}
}

%extend b2XForm {
 %pythoncode %{
 def __repr__(self):
  return """b2XForm(
    R        = %s,
    position = %s)"""% tuple(str(a) for a in\
   (self.R,self.position))
 %}
}

