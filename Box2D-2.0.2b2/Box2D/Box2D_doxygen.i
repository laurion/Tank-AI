%feature("docstring") b2AABB "An axis aligned bounding box.";

%feature("docstring") b2AABB::IsValid "Verify that the bounds are sorted.";

%feature("docstring") b2Body "A rigid body.";

%feature("docstring") b2Body::CreateShape "Creates a shape and attach it to this body.

Parameters:
-----------

shapeDef: the shape definition.

WARNING: 
This function is locked during callbacks.";

%feature("docstring") b2Body::DestroyShape "Destroy a shape. This removes the shape from the broad-phase and therefore destroys any contacts associated with this shape. All shapes attached to a body are implicitly destroyed when the body is destroyed.

Parameters:
-----------

shape: the shape to be removed.

WARNING: 
This function is locked during callbacks.";

%feature("docstring") b2Body::SetMass "Set the mass properties. Note that this changes the center of mass position. If you are not sure how to compute mass properties, use SetMassFromShapes. The inertia tensor is assumed to be relative to the center of mass.

Parameters:
-----------

massData: the mass properties.";

%feature("docstring") b2Body::SetMassFromShapes "Compute the mass properties from the attached shapes. You typically call this after adding all the shapes. If you add or remove shapes later, you may want to call this again. Note that this changes the center of mass position.";

%feature("docstring") b2Body::SetXForm "Set the position of the body's origin and rotation (radians). This breaks any contacts and wakes the other bodies.

Parameters:
-----------

position: the new world position of the body's origin (not necessarily the center of mass).

angle: the new world rotation angle of the body in radians.

false if the movement put a shape outside the world. In this case the body is automatically frozen.";

%feature("docstring") b2Body::GetXForm "Get the body transform for the body's origin. 
the world transform of the body's origin.";

%feature("docstring") b2Body::GetPosition "Get the world body origin position. 
the world position of the body's origin.";

%feature("docstring") b2Body::GetAngle "Get the angle in radians. 
the current world rotation angle in radians.";

%feature("docstring") b2Body::GetLinearDamping "Get the linear damping.";

%feature("docstring") b2Body::GetAngularDamping "Get the angular damping.";

%feature("docstring") b2Body::GetWorldCenter "Get the world position of the center of mass.";

%feature("docstring") b2Body::GetLocalCenter "Get the local position of the center of mass.";

%feature("docstring") b2Body::SetLinearVelocity "Set the linear velocity of the center of mass.

Parameters:
-----------

v: the new linear velocity of the center of mass.";

%feature("docstring") b2Body::GetLinearVelocity "Get the linear velocity of the center of mass. 
the linear velocity of the center of mass.";

%feature("docstring") b2Body::SetAngularVelocity "Set the angular velocity.

Parameters:
-----------

omega: the new angular velocity in radians/second.";

%feature("docstring") b2Body::GetAngularVelocity "Get the angular velocity. 
the angular velocity in radians/second.";

%feature("docstring") b2Body::ApplyForce "Apply a force at a world point. If the force is not applied at the center of mass, it will generate a torque and affect the angular velocity. This wakes up the body.

Parameters:
-----------

force: the world force vector, usually in Newtons (N).

point: the world position of the point of application.";

%feature("docstring") b2Body::ApplyTorque "Apply a torque. This affects the angular velocity without affecting the linear velocity of the center of mass. This wakes up the body.

Parameters:
-----------

torque: about the z-axis (out of the screen), usually in N-m.";

%feature("docstring") b2Body::ApplyImpulse "Apply an impulse at a point. This immediately modifies the velocity. It also modifies the angular velocity if the point of application is not at the center of mass. This wakes up the body.

Parameters:
-----------

impulse: the world impulse vector, usually in N-seconds or kg-m/s.

point: the world position of the point of application.";

%feature("docstring") b2Body::GetMass "Get the total mass of the body. 
the mass, usually in kilograms (kg).";

%feature("docstring") b2Body::GetInertia "Get the central rotational inertia of the body. 
the rotational inertia, usually in kg-m^2.";

%feature("docstring") b2Body::GetWorldPoint "Get the world coordinates of a point given the local coordinates.

Parameters:
-----------

localPoint: a point on the body measured relative the the body's origin.

the same point expressed in world coordinates.";

%feature("docstring") b2Body::GetWorldVector "Get the world coordinates of a vector given the local coordinates.

Parameters:
-----------

localVector: a vector fixed in the body.

the same vector expressed in world coordinates.";

%feature("docstring") b2Body::GetLocalPoint "Gets a local point relative to the body's origin given a world point.

Parameters:
-----------

a: point in world coordinates.

the corresponding local point relative to the body's origin.";

%feature("docstring") b2Body::GetLocalVector "Gets a local vector given a world vector.

Parameters:
-----------

a: vector in world coordinates.

the corresponding local vector.";

%feature("docstring") b2Body::GetLinearVelocityFromWorldPoint "Get the world linear velocity of a world point attached to this body.

Parameters:
-----------

a: point in world coordinates.

the world velocity of a point.";

%feature("docstring") b2Body::GetLinearVelocityFromLocalPoint "Get the world velocity of a local point.

Parameters:
-----------

a: point in local coordinates.

the world velocity of a point.";

%feature("docstring") b2Body::IsBullet "Is this body treated like a bullet for continuous collision detection?";

%feature("docstring") b2Body::SetBullet "Should this body be treated like a bullet for continuous collision detection?";

%feature("docstring") b2Body::IsStatic "Is this body static (immovable)?";

%feature("docstring") b2Body::IsDynamic "Is this body dynamic (movable)?";

%feature("docstring") b2Body::IsFrozen "Is this body frozen?";

%feature("docstring") b2Body::IsSleeping "Is this body sleeping (not simulating).";

%feature("docstring") b2Body::AllowSleeping "You can disable sleeping on this body.";

%feature("docstring") b2Body::CanSleep "Get whether or not this body is allowed to sleep.";

%feature("docstring") b2Body::IsRotationFixed "Get whether or not this body is allowed to rotate.";

%feature("docstring") b2Body::WakeUp "Wake up this body so it will begin simulating.";

%feature("docstring") b2Body::PutToSleep "Put this body to sleep so it will stop simulating. This also sets the velocity to zero.";

%feature("docstring") b2Body::GetShapeList "Get the list of all shapes attached to this body.";

%feature("docstring") b2Body::GetJointList "Get the list of all joints attached to this body.";

%feature("docstring") b2Body::GetControllerList "Get the list of all controllers attached to this body.";

%feature("docstring") b2Body::GetNext "Get the next body in the world's body list.";

%feature("docstring") b2Body::GetUserData "Get the user data pointer that was provided in the body definition.";

%feature("docstring") b2Body::SetUserData "Set the user data. Use this to store your application specific data.";

%feature("docstring") b2Body::GetWorld "Get the parent world of this body.";

%feature("docstring") b2BodyDef "A body definition holds all the data needed to construct a rigid body. You can safely re-use body definitions.";

%feature("docstring") b2BodyDef::b2BodyDef "This constructor sets the body definition default values.";


%feature("docstring") b2BoundaryListener "This is called when a body's shape passes outside of the world boundary.";

%feature("docstring") b2BoundaryListener::Violation "This is called for each body that leaves the world boundary. 
WARNING: 
you can't modify the world inside this callback.";




%feature("docstring") b2BuoyancyController "Calculates buoyancy forces for fluids in the form of a half plane.";

%feature("docstring") b2BuoyancyController::Step "

See: b2Controller.Step";

%feature("docstring") b2BuoyancyController::Draw "

See: b2Controller.Draw";

%feature("docstring") b2BuoyancyControllerDef "This class is used to build buoyancy controllers.";



%feature("docstring") b2CircleDef "This structure is used to build circle shapes.";

%feature("docstring") b2CircleShape "A circle shape.";

%feature("docstring") b2CircleShape::TestPoint "

See: b2Shape.TestPoint";

%feature("docstring") b2CircleShape::TestSegment "

See: b2Shape.TestSegment";

%feature("docstring") b2CircleShape::ComputeAABB "

See: b2Shape.ComputeAABB";

%feature("docstring") b2CircleShape::ComputeSweptAABB "

See: b2Shape.ComputeSweptAABB";

%feature("docstring") b2CircleShape::ComputeMass "

See: b2Shape.ComputeMass";

%feature("docstring") b2CircleShape::ComputeSubmergedArea "

See: b2Shape.ComputeSubmergedArea";

%feature("docstring") b2CircleShape::GetLocalPosition "Get the local position of this circle in its parent body.";

%feature("docstring") b2CircleShape::GetRadius "Get the radius of this circle.";

%feature("docstring") b2Color "Color for debug drawing. Each value has the range [0,1].";

%feature("docstring") b2ConstantAccelController "Applies a force every frame.";

%feature("docstring") b2ConstantAccelController::Step "

See: b2Controller.Step";

%feature("docstring") b2ConstantAccelControllerDef "This class is used to build constant acceleration controllers.";

%feature("docstring") b2ConstantForceController "Applies a force every frame.";

%feature("docstring") b2ConstantForceController::Step "

See: b2Controller.Step";

%feature("docstring") b2ConstantForceControllerDef "This class is used to build constant force controllers.";

%feature("docstring") b2Contact "The class manages contact between two shapes. A contact exists for each overlapping AABB in the broad-phase (except if filtered). Therefore a contact object may exist that has no contact points.";

%feature("docstring") b2Contact::GetManifolds "Get the manifold array.";

%feature("docstring") b2Contact::GetManifoldCount "Get the number of manifolds. This is 0 or 1 between convex shapes. This may be greater than 1 for convex-vs-concave shapes. Each manifold holds up to two contact points with a shared contact normal.";

%feature("docstring") b2Contact::IsSolid "Is this contact solid? 
true if this contact should generate a response.";

%feature("docstring") b2Contact::GetNext "Get the next contact in the world's contact list.";

%feature("docstring") b2Contact::GetShape1 "Get the first shape in this contact.";

%feature("docstring") b2Contact::GetShape2 "Get the second shape in this contact.";



%feature("docstring") b2ContactEdge "A contact edge is used to connect bodies and contacts together in a contact graph where each body is a node and each contact is an edge. A contact edge belongs to a doubly linked list maintained in each attached body. Each contact has two contact nodes, one for each attached body.";

%feature("docstring") b2ContactFilter "Implement this class to provide collision filtering. In other words, you can implement this class if you want finer control over contact creation.";

%feature("docstring") b2ContactFilter::ShouldCollide "Return true if contact calculations should be performed between these two shapes. 
WARNING: 
for performance reasons this is only called when the AABBs begin to overlap.";

%feature("docstring") b2ContactFilter::RayCollide "Return true if the given shape should be considered for ray intersection.";

%feature("docstring") b2ContactID::Features "The features that intersect to form the contact point.";

%feature("docstring") b2ContactListener "Implement this class to get collision results. You can use these results for things like sounds and game logic. You can also get contact results by traversing the contact lists after the time step. However, you might miss some contacts because continuous physics leads to sub-stepping. Additionally you may receive multiple callbacks for the same contact in a single time step. You should strive to make your callbacks efficient because there may be many callbacks per time step. 
WARNING: 
The contact separation is the last computed value.

You cannot create/destroy Box2D entities inside these callbacks.";

%feature("docstring") b2ContactListener::Add "Called when a contact point is added. This includes the geometry and the forces.";

%feature("docstring") b2ContactListener::Persist "Called when a contact point persists. This includes the geometry and the forces.";

%feature("docstring") b2ContactListener::Remove "Called when a contact point is removed. This includes the last computed geometry and forces.";

%feature("docstring") b2ContactListener::Result "Called after a contact point is solved.";


%feature("docstring") b2ContactPoint "This structure is used to report contact points.";


%feature("docstring") b2ContactResult "This structure is used to report contact point results.";


%feature("docstring") b2Controller "Base class for controllers. Controllers are a convience for encapsulating common per-step functionality.";

%feature("docstring") b2Controller::Step "Controllers override this to implement per-step functionality.";

%feature("docstring") b2Controller::Draw "Controllers override this to provide debug drawing.";

%feature("docstring") b2Controller::AddBody "Adds a body to the controller list.";

%feature("docstring") b2Controller::RemoveBody "Removes a body from the controller list.";

%feature("docstring") b2Controller::Clear "Removes all bodies from the controller list.";

%feature("docstring") b2Controller::GetType "Get the type of the controller.";

%feature("docstring") b2Controller::GetNext "Get the next controller in the world's body list.";

%feature("docstring") b2Controller::GetWorld "Get the parent world of this body.";

%feature("docstring") b2Controller::GetBodyList "Get the attached body list.";


%feature("docstring") b2ControllerEdge "A controller edge is used to connect bodies and controllers together in a bipartite graph.";

%feature("docstring") b2DebugDraw "Implement and register this class with a  b2Worldto provide debug drawing of physics entities in your game.";

%feature("docstring") b2DebugDraw::SetFlags "Set the drawing flags.";

%feature("docstring") b2DebugDraw::GetFlags "Get the drawing flags.";

%feature("docstring") b2DebugDraw::AppendFlags "Append flags to the current flags.";

%feature("docstring") b2DebugDraw::ClearFlags "Clear flags from the current flags.";

%feature("docstring") b2DebugDraw::DrawPolygon "Draw a closed polygon provided in CCW order.";

%feature("docstring") b2DebugDraw::DrawSolidPolygon "Draw a solid closed polygon provided in CCW order.";

%feature("docstring") b2DebugDraw::DrawCircle "Draw a circle.";

%feature("docstring") b2DebugDraw::DrawSolidCircle "Draw a solid circle.";

%feature("docstring") b2DebugDraw::DrawSegment "Draw a line segment.";

%feature("docstring") b2DebugDraw::DrawXForm "Draw a transform. Choose your own length scale.

Parameters:
-----------

xf: a transform.";

%feature("docstring") b2DestructionListener "Joints and shapes are destroyed when their associated body is destroyed. Implement this listener so that you may nullify references to these joints and shapes.";

%feature("docstring") b2DestructionListener::SayGoodbye "Called when any joint is about to be destroyed due to the destruction of one of its attached bodies.";

%feature("docstring") b2DestructionListener::SayGoodbye "Called when any shape is about to be destroyed due to the destruction of its parent body.";

%feature("docstring") b2DistanceJoint "A distance joint constrains two points on two bodies to remain at a fixed distance from each other. You can view this as a massless, rigid rod.";

%feature("docstring") b2DistanceJoint::GetAnchor1 "Get the anchor point on body1 in world coordinates.";

%feature("docstring") b2DistanceJoint::GetAnchor2 "Get the anchor point on body2 in world coordinates.";

%feature("docstring") b2DistanceJoint::GetReactionForce "Get the reaction force on body2 at the joint anchor.";

%feature("docstring") b2DistanceJoint::GetReactionTorque "Get the reaction torque on body2.";

%feature("docstring") b2DistanceJointDef "Distance joint definition. This requires defining an anchor point on both bodies and the non-zero length of the distance joint. The definition uses local anchor points so that the initial configuration can violate the constraint slightly. This helps when saving and loading a game. 
WARNING: 
Do not use a zero or short length.";

%feature("docstring") b2DistanceJointDef::Initialize "Initialize the bodies, anchors, and length using the world anchors.";


%feature("docstring") b2EdgeChainDef "This structure is used to build edge shapes.";

%feature("docstring") b2EdgeShape "The edge shape.";

%feature("docstring") b2EdgeShape::TestPoint "

See: b2Shape.TestPoint";

%feature("docstring") b2EdgeShape::TestSegment "

See: b2Shape.TestSegment";

%feature("docstring") b2EdgeShape::ComputeAABB "

See: b2Shape.ComputeAABB";

%feature("docstring") b2EdgeShape::ComputeSweptAABB "

See: b2Shape.ComputeSweptAABB";

%feature("docstring") b2EdgeShape::ComputeMass "

See: b2Shape.ComputeMass";

%feature("docstring") b2EdgeShape::ComputeSubmergedArea "

WARNING: 
This only gives a consistent and sensible answer when when summed over a body only contains loops of edges

See: b2Shape.ComputeSubmergedArea";

%feature("docstring") b2EdgeShape::GetLength "Linear distance from vertex1 to vertex2:.";

%feature("docstring") b2EdgeShape::GetVertex1 "Local position of vertex in parent body.";

%feature("docstring") b2EdgeShape::GetVertex2 "Local position of vertex in parent body.";

%feature("docstring") b2EdgeShape::GetCoreVertex1 "\"Core\" vertex with TOI slop for b2Distance functions:";

%feature("docstring") b2EdgeShape::GetCoreVertex2 "\"Core\" vertex with TOI slop for b2Distance functions:";

%feature("docstring") b2EdgeShape::GetNormalVector "Perpendicular unit vector point, pointing from the solid side to the empty side:.";

%feature("docstring") b2EdgeShape::GetDirectionVector "Parallel unit vector, pointing from vertex1 to vertex2:.";

%feature("docstring") b2EdgeShape::GetNextEdge "Get the next edge in the chain.";

%feature("docstring") b2EdgeShape::GetPrevEdge "Get the previous edge in the chain.";

%feature("docstring") b2FilterData "This holds contact filtering data.";

%feature("docstring") b2GearJoint "A gear joint is used to connect two joints together. Either joint can be a revolute or prismatic joint. You specify a gear ratio to bind the motions together: coordinate1 + ratio * coordinate2 = constant The ratio can be negative or positive. If one joint is a revolute joint and the other joint is a prismatic joint, then the ratio will have units of length or units of 1/length. 
WARNING: 
The revolute and prismatic joints must be attached to fixed bodies (which must be body1 on those joints).";

%feature("docstring") b2GearJoint::GetAnchor1 "Get the anchor point on body1 in world coordinates.";

%feature("docstring") b2GearJoint::GetAnchor2 "Get the anchor point on body2 in world coordinates.";

%feature("docstring") b2GearJoint::GetReactionForce "Get the reaction force on body2 at the joint anchor.";

%feature("docstring") b2GearJoint::GetReactionTorque "Get the reaction torque on body2.";

%feature("docstring") b2GearJoint::GetRatio "Get the gear ratio.";

%feature("docstring") b2GearJointDef "Gear joint definition. This definition requires two existing revolute or prismatic joints (any combination will work). The provided joints must attach a dynamic body to a static body.";

%feature("docstring") b2GravityController "Applies simplified gravity between every pair of bodies.";

%feature("docstring") b2GravityController::Step "

See: b2Controller.Step";

%feature("docstring") b2GravityControllerDef "This class is used to build gravity controllers.";



%feature("docstring") b2Joint "The base joint class. Joints are used to constraint two bodies together in various fashions. Some joints also feature limits and motors.";

%feature("docstring") b2Joint::GetType "Get the type of the concrete joint.";

%feature("docstring") b2Joint::GetBody1 "Get the first body attached to this joint.";

%feature("docstring") b2Joint::GetBody2 "Get the second body attached to this joint.";

%feature("docstring") b2Joint::GetAnchor1 "Get the anchor point on body1 in world coordinates.";

%feature("docstring") b2Joint::GetAnchor2 "Get the anchor point on body2 in world coordinates.";

%feature("docstring") b2Joint::GetReactionForce "Get the reaction force on body2 at the joint anchor.";

%feature("docstring") b2Joint::GetReactionTorque "Get the reaction torque on body2.";

%feature("docstring") b2Joint::GetNext "Get the next joint the world joint list.";

%feature("docstring") b2Joint::GetUserData "Get the user data pointer.";

%feature("docstring") b2Joint::SetUserData "Set the user data pointer.";

%feature("docstring") b2Joint::GetCollideConnected "Get whether or not joint bodies can collide.";

%feature("docstring") b2JointDef "Joint definitions are used to construct joints.";

%feature("docstring") b2JointEdge "A joint edge is used to connect bodies and joints together in a joint graph where each body is a node and each joint is an edge. A joint edge belongs to a doubly linked list maintained in each attached body. Each joint has two joint nodes, one for each attached body.";

%feature("docstring") b2LineJoint "A line joint. This joint provides one degree of freedom: translation along an axis fixed in body1. You can use a joint limit to restrict the range of motion and a joint motor to drive the motion or to model joint friction.";

%feature("docstring") b2LineJoint::GetAnchor1 "Get the anchor point on body1 in world coordinates.";

%feature("docstring") b2LineJoint::GetAnchor2 "Get the anchor point on body2 in world coordinates.";

%feature("docstring") b2LineJoint::GetReactionForce "Get the reaction force on body2 at the joint anchor.";

%feature("docstring") b2LineJoint::GetReactionTorque "Get the reaction torque on body2.";

%feature("docstring") b2LineJoint::GetJointTranslation "Get the current joint translation, usually in meters.";

%feature("docstring") b2LineJoint::GetJointSpeed "Get the current joint translation speed, usually in meters per second.";

%feature("docstring") b2LineJoint::IsLimitEnabled "Is the joint limit enabled?";

%feature("docstring") b2LineJoint::EnableLimit "Enable/disable the joint limit.";

%feature("docstring") b2LineJoint::GetLowerLimit "Get the lower joint limit, usually in meters.";

%feature("docstring") b2LineJoint::GetUpperLimit "Get the upper joint limit, usually in meters.";

%feature("docstring") b2LineJoint::SetLimits "Set the joint limits, usually in meters.";

%feature("docstring") b2LineJoint::IsMotorEnabled "Is the joint motor enabled?";

%feature("docstring") b2LineJoint::EnableMotor "Enable/disable the joint motor.";

%feature("docstring") b2LineJoint::SetMotorSpeed "Set the motor speed, usually in meters per second.";

%feature("docstring") b2LineJoint::GetMotorSpeed "Get the motor speed, usually in meters per second.";

%feature("docstring") b2LineJoint::SetMaxMotorForce "Set the maximum motor force, usually in N.";

%feature("docstring") b2LineJoint::GetMotorForce "Get the current motor force, usually in N.";

%feature("docstring") b2LineJointDef "Line joint definition. This requires defining a line of motion using an axis and an anchor point. The definition uses local anchor points and a local axis so that the initial configuration can violate the constraint slightly. The joint translation is zero when the local anchor points coincide in world space. Using local anchors and a local axis helps when saving and loading a game.";

%feature("docstring") b2LineJointDef::Initialize "Initialize the bodies, anchors, axis, and reference angle using the world anchor and world axis.";

%feature("docstring") b2Manifold "A manifold for two touching convex shapes.";

%feature("docstring") b2ManifoldPoint "A manifold point is a contact point belonging to a contact manifold. It holds details related to the geometry and dynamics of the contact points. The point is stored in local coordinates because CCD requires sub-stepping in which the separation is stale.";

%feature("docstring") b2MassData "This holds the mass data computed for a shape.";

%feature("docstring") b2Mat22 "A 2-by-2 matrix. Stored in column-major order.";

%feature("docstring") b2Mat22::b2Mat22 "The default constructor does nothing (for performance).";

%feature("docstring") b2Mat22::b2Mat22 "Construct this matrix using columns.";

%feature("docstring") b2Mat22::b2Mat22 "Construct this matrix using scalars.";

%feature("docstring") b2Mat22::b2Mat22 "Construct this matrix using an angle. This matrix becomes an orthonormal rotation matrix.";

%feature("docstring") b2Mat22::Set "Initialize this matrix using columns.";

%feature("docstring") b2Mat22::Set "Initialize this matrix using an angle. This matrix becomes an orthonormal rotation matrix.";

%feature("docstring") b2Mat22::SetIdentity "Set this to the identity matrix.";

%feature("docstring") b2Mat22::SetZero "Set this matrix to all zeros.";

%feature("docstring") b2Mat22::GetAngle "Extract the angle from this matrix (assumed to be a rotation matrix).";

%feature("docstring") b2Mat22::Solve "Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases.";

%feature("docstring") b2Mat33 "A 3-by-3 matrix. Stored in column-major order.";

%feature("docstring") b2Mat33::b2Mat33 "The default constructor does nothing (for performance).";

%feature("docstring") b2Mat33::b2Mat33 "Construct this matrix using columns.";

%feature("docstring") b2Mat33::SetZero "Set this matrix to all zeros.";

%feature("docstring") b2Mat33::Solve33 "Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases.

Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases.";

%feature("docstring") b2Mat33::Solve22 "Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases. Solve only the upper 2-by-2 matrix equation.

Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse in one-shot cases.";

%feature("docstring") b2MouseJoint "A mouse joint is used to make a point on a body track a specified world point. This a soft constraint with a maximum force. This allows the constraint to stretch and without applying huge forces.";

%feature("docstring") b2MouseJoint::GetAnchor1 "Implements  b2Joint.";

%feature("docstring") b2MouseJoint::GetAnchor2 "Implements  b2Joint.";

%feature("docstring") b2MouseJoint::GetReactionForce "Implements  b2Joint.";

%feature("docstring") b2MouseJoint::GetReactionTorque "Implements  b2Joint.";

%feature("docstring") b2MouseJoint::SetTarget "Use this to update the target point.";

%feature("docstring") b2MouseJointDef "Mouse joint definition. This requires a world target point, tuning parameters, and the time step.";

%feature("docstring") b2OBB "An oriented bounding box.";

%feature("docstring") b2PolygonDef "Convex polygon. The vertices must be in CCW order for a right-handed coordinate system with the z-axis coming out of the screen.";

%feature("docstring") b2PolygonDef::SetAsBox "Build vertices to represent an axis-aligned box.

Parameters:
-----------

hx: the half-width.

hy: the half-height.";

%feature("docstring") b2PolygonDef::SetAsBox "Build vertices to represent an oriented box.

Parameters:
-----------

hx: the half-width.

hy: the half-height.

center: 
the center of the box in local coordinates.

angle: 
the rotation of the box in local coordinates.";

%feature("docstring") b2PolygonShape "A convex polygon.";

%feature("docstring") b2PolygonShape::TestPoint "

See: b2Shape.TestPoint";

%feature("docstring") b2PolygonShape::TestSegment "

See: b2Shape.TestSegment";

%feature("docstring") b2PolygonShape::ComputeAABB "

See: b2Shape.ComputeAABB";

%feature("docstring") b2PolygonShape::ComputeSweptAABB "

See: b2Shape.ComputeSweptAABB";

%feature("docstring") b2PolygonShape::ComputeMass "

See: b2Shape.ComputeMass";

%feature("docstring") b2PolygonShape::ComputeSubmergedArea "

See: b2Shape.ComputeSubmergedArea";

%feature("docstring") b2PolygonShape::GetOBB "Get the oriented bounding box relative to the parent body.";

%feature("docstring") b2PolygonShape::GetCentroid "Get local centroid relative to the parent body.";

%feature("docstring") b2PolygonShape::GetVertexCount "Get the vertex count.";

%feature("docstring") b2PolygonShape::GetVertices "Get the vertices in local coordinates.";

%feature("docstring") b2PolygonShape::GetCoreVertices "Get the core vertices in local coordinates. These vertices represent a smaller polygon that is used for time of impact computations.";

%feature("docstring") b2PolygonShape::GetNormals "Get the edge normal vectors. There is one for each vertex.";

%feature("docstring") b2PolygonShape::GetFirstVertex "Get the first vertex and apply the supplied transform.";

%feature("docstring") b2PolygonShape::Centroid "Get the centroid and apply the supplied transform.";

%feature("docstring") b2PolygonShape::Support "Get the support point in the given world direction. Use the supplied transform.";


%feature("docstring") b2PrismaticJoint "A prismatic joint. This joint provides one degree of freedom: translation along an axis fixed in body1. Relative rotation is prevented. You can use a joint limit to restrict the range of motion and a joint motor to drive the motion or to model joint friction.";

%feature("docstring") b2PrismaticJoint::GetAnchor1 "Get the anchor point on body1 in world coordinates.";

%feature("docstring") b2PrismaticJoint::GetAnchor2 "Get the anchor point on body2 in world coordinates.";

%feature("docstring") b2PrismaticJoint::GetReactionForce "Get the reaction force on body2 at the joint anchor.";

%feature("docstring") b2PrismaticJoint::GetReactionTorque "Get the reaction torque on body2.";

%feature("docstring") b2PrismaticJoint::GetJointTranslation "Get the current joint translation, usually in meters.";

%feature("docstring") b2PrismaticJoint::GetJointSpeed "Get the current joint translation speed, usually in meters per second.";

%feature("docstring") b2PrismaticJoint::IsLimitEnabled "Is the joint limit enabled?";

%feature("docstring") b2PrismaticJoint::EnableLimit "Enable/disable the joint limit.";

%feature("docstring") b2PrismaticJoint::GetLowerLimit "Get the lower joint limit, usually in meters.";

%feature("docstring") b2PrismaticJoint::GetUpperLimit "Get the upper joint limit, usually in meters.";

%feature("docstring") b2PrismaticJoint::SetLimits "Set the joint limits, usually in meters.";

%feature("docstring") b2PrismaticJoint::IsMotorEnabled "Is the joint motor enabled?";

%feature("docstring") b2PrismaticJoint::EnableMotor "Enable/disable the joint motor.";

%feature("docstring") b2PrismaticJoint::SetMotorSpeed "Set the motor speed, usually in meters per second.";

%feature("docstring") b2PrismaticJoint::GetMotorSpeed "Get the motor speed, usually in meters per second.";

%feature("docstring") b2PrismaticJoint::SetMaxMotorForce "Set the maximum motor force, usually in N.";

%feature("docstring") b2PrismaticJoint::GetMotorForce "Get the current motor force, usually in N.";

%feature("docstring") b2PrismaticJointDef "Prismatic joint definition. This requires defining a line of motion using an axis and an anchor point. The definition uses local anchor points and a local axis so that the initial configuration can violate the constraint slightly. The joint translation is zero when the local anchor points coincide in world space. Using local anchors and a local axis helps when saving and loading a game.";

%feature("docstring") b2PrismaticJointDef::Initialize "Initialize the bodies, anchors, axis, and reference angle using the world anchor and world axis.";


%feature("docstring") b2PulleyJoint "The pulley joint is connected to two bodies and two fixed ground points. The pulley supports a ratio such that: length1 + ratio * length2 <= constant Yes, the force transmitted is scaled by the ratio. The pulley also enforces a maximum length limit on both sides. This is useful to prevent one side of the pulley hitting the top.";

%feature("docstring") b2PulleyJoint::GetAnchor1 "Get the anchor point on body1 in world coordinates.";

%feature("docstring") b2PulleyJoint::GetAnchor2 "Get the anchor point on body2 in world coordinates.";

%feature("docstring") b2PulleyJoint::GetReactionForce "Get the reaction force on body2 at the joint anchor.";

%feature("docstring") b2PulleyJoint::GetReactionTorque "Get the reaction torque on body2.";

%feature("docstring") b2PulleyJoint::GetGroundAnchor1 "Get the first ground anchor.";

%feature("docstring") b2PulleyJoint::GetGroundAnchor2 "Get the second ground anchor.";

%feature("docstring") b2PulleyJoint::GetLength1 "Get the current length of the segment attached to body1.";

%feature("docstring") b2PulleyJoint::GetLength2 "Get the current length of the segment attached to body2.";

%feature("docstring") b2PulleyJoint::GetRatio "Get the pulley ratio.";

%feature("docstring") b2PulleyJointDef "Pulley joint definition. This requires two ground anchors, two dynamic body anchor points, max lengths for each side, and a pulley ratio.";

%feature("docstring") b2PulleyJointDef::Initialize "Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.";

%feature("docstring") b2RevoluteJoint "A revolute joint constrains to bodies to share a common point while they are free to rotate about the point. The relative rotation about the shared point is the joint angle. You can limit the relative rotation with a joint limit that specifies a lower and upper angle. You can use a motor to drive the relative rotation about the shared point. A maximum motor torque is provided so that infinite forces are not generated.";

%feature("docstring") b2RevoluteJoint::GetAnchor1 "Get the anchor point on body1 in world coordinates.";

%feature("docstring") b2RevoluteJoint::GetAnchor2 "Get the anchor point on body2 in world coordinates.";

%feature("docstring") b2RevoluteJoint::GetReactionForce "Get the reaction force on body2 at the joint anchor.";

%feature("docstring") b2RevoluteJoint::GetReactionTorque "Get the reaction torque on body2.";

%feature("docstring") b2RevoluteJoint::GetJointAngle "Get the current joint angle in radians.";

%feature("docstring") b2RevoluteJoint::GetJointSpeed "Get the current joint angle speed in radians per second.";

%feature("docstring") b2RevoluteJoint::IsLimitEnabled "Is the joint limit enabled?";

%feature("docstring") b2RevoluteJoint::EnableLimit "Enable/disable the joint limit.";

%feature("docstring") b2RevoluteJoint::GetLowerLimit "Get the lower joint limit in radians.";

%feature("docstring") b2RevoluteJoint::GetUpperLimit "Get the upper joint limit in radians.";

%feature("docstring") b2RevoluteJoint::SetLimits "Set the joint limits in radians.";

%feature("docstring") b2RevoluteJoint::IsMotorEnabled "Is the joint motor enabled?";

%feature("docstring") b2RevoluteJoint::EnableMotor "Enable/disable the joint motor.";

%feature("docstring") b2RevoluteJoint::SetMotorSpeed "Set the motor speed in radians per second.";

%feature("docstring") b2RevoluteJoint::GetMotorSpeed "Get the motor speed in radians per second.";

%feature("docstring") b2RevoluteJoint::SetMaxMotorTorque "Set the maximum motor torque, usually in N-m.";

%feature("docstring") b2RevoluteJoint::GetMotorTorque "Get the current motor torque, usually in N-m.";

%feature("docstring") b2RevoluteJointDef "Revolute joint definition. This requires defining an anchor point where the bodies are joined. The definition uses local anchor points so that the initial configuration can violate the constraint slightly. You also need to specify the initial relative angle for joint limits. This helps when saving and loading a game. The local anchor points are measured from the body's origin rather than the center of mass because: 1. you might not know where the center of mass will be. 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken.";

%feature("docstring") b2RevoluteJointDef::Initialize "Initialize the bodies, anchors, and reference angle using the world anchor.";

%feature("docstring") b2Segment "A line segment.";

%feature("docstring") b2Segment::TestSegment "Ray cast against this segment with another segment.";

%feature("docstring") b2Shape "A shape is used for collision detection. Shapes are created in  b2World. You can use shape for collision detection before they are attached to the world. 
WARNING: 
you cannot reuse shapes.";

%feature("docstring") b2Shape::GetType "Get the type of this shape. You can use this to down cast to the concrete shape. 
the shape type.";

%feature("docstring") b2Shape::IsSensor "Is this shape a sensor (non-solid)? 
the true if the shape is a sensor.";

%feature("docstring") b2Shape::SetFilterData "Set the contact filtering data. You must call  b2World::Refilterto correct existing contacts/non-contacts.";

%feature("docstring") b2Shape::GetFilterData "Get the contact filtering data.";

%feature("docstring") b2Shape::GetBody "Get the parent body of this shape. This is NULL if the shape is not attached. 
the parent body.";

%feature("docstring") b2Shape::GetNext "Get the next shape in the parent body's shape list. 
the next shape.";

%feature("docstring") b2Shape::GetUserData "Get the user data that was assigned in the shape definition. Use this to store your application specific data.";

%feature("docstring") b2Shape::SetUserData "Set the user data. Use this to store your application specific data.";

%feature("docstring") b2Shape::TestPoint "Test a point for containment in this shape. This only works for convex shapes.

Parameters:
-----------

xf: the shape world transform.

p: a point in world coordinates.";

%feature("docstring") b2Shape::TestSegment "Perform a ray cast against this shape.

Parameters:
-----------

xf: 
the shape world transform.

lambda: 
returns the hit fraction. You can use this to compute the contact point p = (1 - lambda) * segment.p1 + lambda * segment.p2.

normal: 
returns the normal at the contact point. If there is no intersection, the normal is not set.

segment: 
defines the begin and end point of the ray cast.

maxLambda: 
a number typically in the range [0,1].";

%feature("docstring") b2Shape::ComputeAABB "Given a transform, compute the associated axis aligned bounding box for this shape.

Parameters:
-----------

aabb: returns the axis aligned box.

xf: the world transform of the shape.";

%feature("docstring") b2Shape::ComputeSweptAABB "Given two transforms, compute the associated swept axis aligned bounding box for this shape.

Parameters:
-----------

aabb: returns the axis aligned box.

xf1: the starting shape world transform.

xf2: the ending shape world transform.";

%feature("docstring") b2Shape::ComputeMass "Compute the mass properties of this shape using its dimensions and density. The inertia tensor is computed about the local origin, not the centroid.

Parameters:
-----------

massData: returns the mass data for this shape.";

%feature("docstring") b2Shape::ComputeSubmergedArea "Compute the volume and centroid of this shape intersected with a half plane

Parameters:
-----------

normal: the surface normal

offset: the surface offset along normal

xf: the shape transform

c: returns the centroid

the total volume less than offset along normal";

%feature("docstring") b2Shape::GetSweepRadius "Get the maximum radius about the parent body's center of mass.";

%feature("docstring") b2Shape::GetFriction "Get the coefficient of friction.";

%feature("docstring") b2Shape::SetFriction "Set the coefficient of friction.";

%feature("docstring") b2Shape::GetRestitution "Get the coefficient of restitution.";

%feature("docstring") b2Shape::SetRestitution "Set the coefficient of restitution.";

%feature("docstring") b2Shape::GetDensity "Get the density of the shape.";

%feature("docstring") b2Shape::SetDensity "Set the density of the shape.";

%feature("docstring") b2ShapeDef "A shape definition is used to construct a shape. This class defines an abstract shape definition. You can reuse shape definitions safely.";

%feature("docstring") b2ShapeDef::b2ShapeDef "The constructor sets the default shape definition values.";



%feature("docstring") b2Sweep "This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin, which may no coincide with the center of mass. However, to support dynamics we must interpolate the center of mass position.";

%feature("docstring") b2Sweep::GetXForm "Get the interpolated transform at a specific time.

Parameters:
-----------

t: the normalized time in [0,1].";

%feature("docstring") b2Sweep::Advance "Advance the sweep forward, yielding a new initial state.

Parameters:
-----------

t: the new initial time.";

%feature("docstring") b2TensorDampingController "Applies top down linear damping to the controlled bodies The damping is calculated by multiplying velocity by a matrix in local co-ordinates.";

%feature("docstring") b2TensorDampingController::Step "

See: b2Controller.Step";

%feature("docstring") b2TensorDampingControllerDef "This class is used to build tensor damping controllers.";

%feature("docstring") b2TensorDampingControllerDef::SetAxisAligned "Sets damping independantly along the x and y axes.";


%feature("docstring") b2Vec2 "A 2D column vector.";

%feature("docstring") b2Vec2::b2Vec2 "Default constructor does nothing (for performance).";

%feature("docstring") b2Vec2::b2Vec2 "Construct using coordinates.";

%feature("docstring") b2Vec2::SetZero "Set this vector to all zeros.";

%feature("docstring") b2Vec2::Set "Set this vector to some specified coordinates.";

%feature("docstring") b2Vec2::Length "Get the length of this vector (the norm).";

%feature("docstring") b2Vec2::LengthSquared "Get the length squared. For performance, use this instead of  b2Vec2::Length(if possible).";

%feature("docstring") b2Vec2::Normalize "Convert this vector into a unit vector. Returns the length.";

%feature("docstring") b2Vec2::IsValid "Does this vector contain finite coordinates?";

%feature("docstring") b2Vec3 "A 2D column vector with 3 elements.";

%feature("docstring") b2Vec3::b2Vec3 "Default constructor does nothing (for performance).";

%feature("docstring") b2Vec3::b2Vec3 "Construct using coordinates.";

%feature("docstring") b2Vec3::SetZero "Set this vector to all zeros.";

%feature("docstring") b2Vec3::Set "Set this vector to some specified coordinates.";


%feature("docstring") b2Version "Version numbering scheme. See http://en.wikipedia.org/wiki/Software_versioning";

%feature("docstring") b2World "The world class manages all physics entities, dynamic simulation, and asynchronous queries. The world also contains efficient memory management facilities.";

%feature("docstring") b2World::b2World "Construct a world object.

Parameters:
-----------

worldAABB: a bounding box that completely encompasses all your shapes.

gravity: the world gravity vector.

doSleep: improve performance by not simulating inactive bodies.";

%feature("docstring") b2World::~b2World "Destruct the world. All physics entities are destroyed and all heap memory is released.";

%feature("docstring") b2World::SetDestructionListener "Register a destruction listener.";

%feature("docstring") b2World::SetBoundaryListener "Register a broad-phase boundary listener.";

%feature("docstring") b2World::SetContactFilter "Register a contact filter to provide specific control over collision. Otherwise the default filter is used (b2_defaultFilter).";

%feature("docstring") b2World::SetContactListener "Register a contact event listener.";

%feature("docstring") b2World::SetDebugDraw "Register a routine for debug drawing. The debug draw functions are called inside the  b2World::Stepmethod, so make sure your renderer is ready to consume draw commands when you call  Step().";

%feature("docstring") b2World::CreateBody "Create a rigid body given a definition. No reference to the definition is retained. 
WARNING: 
This function is locked during callbacks.";

%feature("docstring") b2World::DestroyBody "Destroy a rigid body given a definition. No reference to the definition is retained. This function is locked during callbacks. 
WARNING: 
This automatically deletes all associated shapes and joints.

This function is locked during callbacks.";

%feature("docstring") b2World::CreateJoint "Create a joint to constrain bodies together. No reference to the definition is retained. This may cause the connected bodies to cease colliding. 
WARNING: 
This function is locked during callbacks.";

%feature("docstring") b2World::DestroyJoint "Destroy a joint. This may cause the connected bodies to begin colliding. 
WARNING: 
This function is locked during callbacks.";

%feature("docstring") b2World::CreateController "Add a controller to the world.";

%feature("docstring") b2World::DestroyController "Removes a controller from the world.";

%feature("docstring") b2World::GetGroundBody "The world provides a single static ground body with no collision shapes. You can use this to simplify the creation of joints and static shapes.";

%feature("docstring") b2World::Step "Take a time step. This performs collision detection, integration, and constraint solution.

Parameters:
-----------

timeStep: the amount of time to simulate, this should not vary.

velocityIterations: for the velocity constraint solver.

positionIterations: for the position constraint solver.";

%feature("docstring") b2World::Query "Query the world for all shapes that potentially overlap the provided AABB. You provide a shape pointer buffer of specified size. The number of shapes found is returned.

Parameters:
-----------

aabb: the query box.

shapes: a user allocated shape pointer array of size maxCount (or greater).

maxCount: the capacity of the shapes array.

the number of shapes found in aabb.";

%feature("docstring") b2World::Raycast "Query the world for all shapes that intersect a given segment. You provide a shape pointer buffer of specified size. The number of shapes found is returned, and the buffer is filled in order of intersection

Parameters:
-----------

segment: 
defines the begin and end point of the ray cast, from p1 to p2. Use b2Segment.Extend to create (semi-)infinite rays

shapes: 
a user allocated shape pointer array of size maxCount (or greater).

maxCount: 
the capacity of the shapes array

solidShapes: 
determines if shapes that the ray starts in are counted as hits.

userData: 
passed through the worlds contact filter, with method RayCollide. This can be used to filter valid shapes

the number of shapes found";

%feature("docstring") b2World::RaycastOne "Performs a raycast as with Raycast, finding the first intersecting shape.

Parameters:
-----------

segment: 
defines the begin and end point of the ray cast, from p1 to p2. Use b2Segment.Extend to create (semi-)infinite rays

lambda: 
returns the hit fraction. You can use this to compute the contact point p = (1 - lambda) * segment.p1 + lambda * segment.p2.

normal: 
returns the normal at the contact point. If there is no intersection, the normal is not set.

solidShapes: 
determines if shapes that the ray starts in are counted as hits.

the colliding shape shape, or null if not found";

%feature("docstring") b2World::InRange "Check if the AABB is within the broadphase limits.";

%feature("docstring") b2World::GetBodyList "Get the world body list. With the returned body, use  b2Body::GetNextto get the next body in the world list. A NULL body indicates the end of the list. 
the head of the world body list.";

%feature("docstring") b2World::GetJointList "Get the world joint list. With the returned joint, use  b2Joint::GetNextto get the next joint in the world list. A NULL joint indicates the end of the list. 
the head of the world joint list.";

%feature("docstring") b2World::GetControllerList "Get the world controller list. With the returned controller, use  b2Controller::GetNextto get the next controller in the world list. A NULL controller indicates the end of the list. 
the head of the world controller list.";

%feature("docstring") b2World::Refilter "Re-filter a shape. This re-runs contact filtering on a shape.";

%feature("docstring") b2World::SetWarmStarting "Enable/disable warm starting. For testing.";

%feature("docstring") b2World::SetContinuousPhysics "Enable/disable continuous physics. For testing.";

%feature("docstring") b2World::Validate "Perform validation of internal data structures.";

%feature("docstring") b2World::GetProxyCount "Get the number of broad-phase proxies.";

%feature("docstring") b2World::GetPairCount "Get the number of broad-phase pairs.";

%feature("docstring") b2World::GetBodyCount "Get the number of bodies.";

%feature("docstring") b2World::GetJointCount "Get the number of joints.";

%feature("docstring") b2World::GetContactCount "Get the number of contacts (each may have 0 or more contact points).";

%feature("docstring") b2World::GetControllerCount "Get the number of controllers.";

%feature("docstring") b2World::SetGravity "Change the global gravity vector.";

%feature("docstring") b2World::GetGravity "Get the global gravity vector.";

%feature("docstring") b2World::GetWorldAABB "Get the world's AABB.";

%feature("docstring") b2World::CanSleep "Whether or not bodies can sleep.";

%feature("docstring") b2XForm "A transform contains translation and rotation. It is used to represent the position and orientation of rigid frames.";

%feature("docstring") b2XForm::b2XForm "The default constructor does nothing (for performance).";

%feature("docstring") b2XForm::b2XForm "Initialize using a position vector and a rotation matrix.";

%feature("docstring") b2XForm::SetIdentity "Set this to the identity transform.";

%feature("docstring") b2CollideCircles "Compute the collision manifold between two circles.";

%feature("docstring") b2CollidePolygonAndCircle "Compute the collision manifold between a polygon and a circle.";

%feature("docstring") b2CollidePolygons "Compute the collision manifold between two circles.";

%feature("docstring") b2CollideCircles "Compute the collision manifold between two circles.";

%feature("docstring") b2CollidePolygonAndCircle "Compute the collision manifold between a polygon and a circle.";

%feature("docstring") b2CollidePolygons "Compute the collision manifold between two circles.";

%feature("docstring") b2Distance "Compute the distance between two shapes and the closest points. 
the distance between the shapes or zero if they are overlapped/touching.";

%feature("docstring") b2TimeOfImpact "Compute the time when two shapes begin to touch or touch at a closer distance. 
WARNING: 
the sweeps must have the same time interval.

the fraction between [0,1] in which the shapes first touch. fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.";

%feature("docstring") b2Distance "Compute the distance between two shapes and the closest points. 
the distance between the shapes or zero if they are overlapped/touching.";


%feature("docstring") b2TimeOfImpact "Compute the time when two shapes begin to touch or touch at a closer distance. 
WARNING: 
the sweeps must have the same time interval.

the fraction between [0,1] in which the shapes first touch. fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.";


%feature("docstring") b2IsValid "This function is used to ensure that a floating point number is not a NaN or infinity.";

%feature("docstring") b2InvSqrt "This is a approximate yet fast inverse square-root.";

%feature("docstring") b2Dot "Peform the dot product on two vectors.";

%feature("docstring") b2Cross "Perform the cross product on two vectors. In 2D this produces a scalar.";

%feature("docstring") b2Cross "Perform the cross product on a vector and a scalar. In 2D this produces a vector.";

%feature("docstring") b2Cross "Perform the cross product on a scalar and a vector. In 2D this produces a vector.";

%feature("docstring") b2Mul "Multiply a matrix times a vector. If a rotation matrix is provided, then this transforms the vector from one frame to another.";

%feature("docstring") b2MulT "Multiply a matrix transpose times a vector. If a rotation matrix is provided, then this transforms the vector from one frame to another (inverse transform).";

%feature("docstring") b2Dot "Perform the dot product on two vectors.";

%feature("docstring") b2Cross "Perform the cross product on two vectors.";

%feature("docstring") b2Mul "Multiply a matrix times a vector.";

%feature("docstring") b2Random "Random floating point number in range [lo, hi]. With no arguments, returns one in -1,1.";

%feature("docstring") b2NextPowerOfTwo "\"Next Largest Power of 2 Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm that recursively \"folds\" the upper bits into the lower bits. This process yields a bit vector with the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next largest power of 2. For a 32-bit value:\"";

%feature("docstring") b2Alloc "Implement this function to use your own memory allocator.";

%feature("docstring") b2Free "If you implement b2Alloc, you should also implement this function.";

%feature("docstring") b2Alloc "Implement this function to use your own memory allocator.";

%feature("docstring") b2Free "If you implement b2Alloc, you should also implement this function.";

%feature("docstring") b2MixFriction "Friction mixing law. Feel free to customize this.";

%feature("docstring") b2MixRestitution "Restitution mixing law. Feel free to customize this.";




