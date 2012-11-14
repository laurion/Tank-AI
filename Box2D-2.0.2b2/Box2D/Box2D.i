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

%module(directors="1") Box2D
%{
    #include "Box2D/Box2D.h"
    
    //Define these functions so that SWIG does not fail
    void b2BroadPhase::ValidatePairs() { }
%}

#ifdef SWIGPYTHON
    #ifdef USE_EXCEPTIONS
        // See Common/b2Settings.h also
        %include "exception.i"

        %exception {
            try {
                $action
            } catch(b2AssertException) {
                // error already set, pass it on to python
            }

            if (PyErr_Occurred()) {
                // This is if we set the error inside a function; report it to swig
                SWIG_fail;
            }
        }
    #endif

    #pragma SWIG nowarn=314
    // Add support for == and != in Python for shapes, joints, and bodies.
    %inline %{
        bool __b2PythonJointPointerEquals__(b2Joint* a, b2Joint* b) {
            return a==b;
        }
        bool __b2PythonBodyPointerEquals__(b2Body* a, b2Body* b) {
            return a==b;
        }
        bool __b2PythonShapePointerEquals__(b2Shape* a, b2Shape* b) {
            return a==b;
        }
        bool __b2PythonControllerPointerEquals__(b2Controller* a, b2Controller* b) {
            return a==b;
        }
    %}

    %include "Box2D/Box2D_doxygen.i"
    %include "Box2D/Box2D_printing.i"
    %include "Box2D/Box2D_pickling.i"

    /* ---- features ---- */

    //Autodoc puts the basic docstrings for each function
    %feature("autodoc", "1");

    //Add callback support for the following classes:
    %feature("director") b2ContactListener;
    %feature("director") b2ContactFilter;
    %feature("director") b2BoundaryListener;
    %feature("director") b2DestructionListener;
    %feature("director") b2DebugDraw;

    // Director-exceptions are a result of callbacks that happen as a result to
    // the physics step, usually. So, catch those errors and report them back to Python.
    %exception b2World::Step {
        try { $action }
        catch (Swig::DirectorException) { SWIG_fail; }
    }
    /* ---- renames ---- */

    //These operators do not work unless explicitly defined like this 
    %rename(b2add) operator  + (const b2Vec2& a, const b2Vec2& b);
    %rename(b2add) operator  + (const b2Mat22& A, const b2Mat22& B);
    %rename(b2sub) operator  - (const b2Vec2& a, const b2Vec2& b);
    %rename(b2mul) operator  * (float32 s, const b2Vec2& a);
    %rename(b2equ) operator == (const b2Vec2& a, const b2Vec2& b);

    %rename(b2mul) operator * (float32 s, const b2Vec3& a);
    %rename(b2add) operator + (const b2Vec3& a, const b2Vec3& b);
    %rename(b2sub) operator - (const b2Vec3& a, const b2Vec3& b);

    //Since Python (apparently) requires __imul__ to return self,
    //these void operators will not do. So, rename them, then call them
    //with Python code, and return self. (see further down in b2Vec2)
    %rename(add_vector) b2Vec2::operator += (const b2Vec2& v);
    %rename(sub_vector) b2Vec2::operator -= (const b2Vec2& v);
    %rename(mul_float ) b2Vec2::operator *= (float32 a);

    %rename(_GetShapeList) b2Body::GetShapeList; //Modify these to return actual lists, not linked lists
    %rename(_GetBodyList)  b2World::GetBodyList;
    %rename(_GetBodyList)  b2Controller::GetBodyList;
    %rename(_GetJointList) b2World::GetJointList;
    %rename(_GetControllerList) b2World::GetControllerList;

    /* ---- handle userData ---- */
    %include "Box2D/Box2D_userdata.i"

    /* ---- typemaps ---- */
    %typemap(in) b2Vec2* self {
        int res1 = SWIG_ConvertPtr($input, (void**)&$1, SWIGTYPE_p_b2Vec2, 0);
        if (!SWIG_IsOK(res1)) {
            SWIG_exception_fail(SWIG_ArgError(res1), "in method '" "$symname" "', argument " "$1_name"" of type '" "$1_type""'"); 
        }
    }

    //Resolve ambiguities in overloaded functions when you pass a tuple or list when 
    //SWIG expects a b2Vec2
    %typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) b2Vec2*,b2Vec2& {
       $1 = (PyList_Check($input)  || 
             PyTuple_Check($input) || 
             SWIG_CheckState(SWIG_ConvertPtr($input, 0, SWIGTYPE_p_b2Vec2, 0))
            ) ? 1 : 0;
    }

    // Allow b2Vec2* arguments be passed in as tuples or lists
    %typemap(in) b2Vec2* (b2Vec2 temp) {
        //input - $input -> ($1_type) $1 $1_descriptor
        if (PyTuple_Check($input) || PyList_Check($input)) {
            int sz = (PyList_Check($input) ? PyList_Size($input) : PyTuple_Size($input));
            if (sz != 2) {
                PyErr_Format(PyExc_TypeError, "Expected tuple or list of length 2, got length %d", PyTuple_Size($input));
                SWIG_fail;
            }
            int res1 = SWIG_AsVal_float(PySequence_GetItem($input, 0), &temp.x);
            if (!SWIG_IsOK(res1)) {
                PyErr_SetString(PyExc_TypeError,"Converting from sequence to b2Vec2, expected int/float arguments");
                SWIG_fail;
            } 
            res1 = SWIG_AsVal_float(PySequence_GetItem($input, 1), &temp.y);
            if (!SWIG_IsOK(res1)) {
                PyErr_SetString(PyExc_TypeError,"Converting from sequence to b2Vec2, expected int/float arguments");
                SWIG_fail;
            } 
        } else if ($input==Py_None) {
            temp.Set(0.0f,0.0f);
        } else {
            int res1 = SWIG_ConvertPtr($input, (void**)&$1, $1_descriptor, 0);
            if (!SWIG_IsOK(res1)) {
                SWIG_exception_fail(SWIG_ArgError(res1), "in method '" "$symname" "', argument " "$1_name"" of type '" "$1_type""'"); 
                SWIG_fail;
            }
            temp =(b2Vec2&) *$1;
        }
        $1 = &temp;
    }

    // Allow b2Vec2& arguments be passed in as tuples or lists
    %typemap(in) b2Vec2& (b2Vec2 temp) {
        //input - $input -> ($1_type) $1 $1_descriptor
        if (PyTuple_Check($input) || PyList_Check($input)) {
            int sz = (PyList_Check($input) ? PyList_Size($input) : PyTuple_Size($input));
            if (sz != 2) {
                PyErr_Format(PyExc_TypeError, "Expected tuple or list of length 2, got length %d", PyTuple_Size($input));
                SWIG_fail;
            }
            int res1 = SWIG_AsVal_float(PySequence_GetItem($input, 0), &temp.x);
            if (!SWIG_IsOK(res1)) {
                PyErr_SetString(PyExc_TypeError,"Converting from sequence to b2Vec2, expected int/float arguments");
                SWIG_fail;
            } 
            res1 = SWIG_AsVal_float(PySequence_GetItem($input, 1), &temp.y);
            if (!SWIG_IsOK(res1)) {
                PyErr_SetString(PyExc_TypeError,"Converting from sequence to b2Vec2, expected int/float arguments");
                SWIG_fail;
            } 
        } else if ($input == Py_None) {
            temp.Set(0.0f,0.0f);
        } else {
            int res1 = SWIG_ConvertPtr($input, (void**)&$1, $1_descriptor, 0);
            if (!SWIG_IsOK(res1)) {
                SWIG_exception_fail(SWIG_ArgError(res1), "in method '" "$symname" "', argument " "$1_name"" of type '" "$1_type""'"); 
            }
            temp =(b2Vec2&) *$1;
        }
        $1 = &temp;
    }


    //Allow access to void* types
    %typemap(in) void* {
        $1 = $input;
        Py_INCREF((PyObject*)$1);
    }
    %typemap(out) void* {
        if (!$1)
            $result=Py_None;
        else
            $result=(PyObject*)$1;

        Py_INCREF($result);
    }

    %typemap(directorin) (const b2Vec2* vertices, int32 vertexCount) {
        $input = PyTuple_New(vertexCount);
        PyObject* vertex;
        for (int i=0; i < vertexCount; i++) {
            vertex = PyTuple_New(2);
            PyTuple_SetItem(vertex, 0, SWIG_From_double((float32)vertices[i].x));
            PyTuple_SetItem(vertex, 1, SWIG_From_double((float32)vertices[i].y));

            PyTuple_SetItem($input, i, vertex);
        }
    }

    %typemap(directorin) b2Vec2& {
        $input = PyTuple_New(2);
        PyTuple_SetItem( $input, 0, SWIG_From_double((float32)$1_name.x));
        PyTuple_SetItem( $input, 1, SWIG_From_double((float32)$1_name.y));
    }

    /* ---- ignores ---- */
    /*Re-implement these inaccessible members on the Python side:*/
    %ignore b2PolygonDef::vertices;
    %ignore b2EdgeChainDef::vertices;
    %ignore b2PolygonShape::vertices;
    %ignore b2PolygonShape::GetVertices; //Inaccessible 
    %ignore b2PolygonShape::GetNormals;

    /* ---- extending classes ---- */
    %extend b2World {
    public:        
        PyObject* Raycast(const b2Segment& segment, int32 maxCount, bool solidShapes, PyObject* userData) {
            //returns tuple (shapecount, shapes)
            PyObject* ret=Py_None;
            b2Shape** shapes=new b2Shape* [maxCount];

            if (!shapes) {
                PyErr_SetString(PyExc_MemoryError, "Insufficient memory");
                return NULL;
            }
            
            if (userData==Py_None) {
                userData=NULL;
            } else {
                Py_INCREF(userData);
            }

            int32 num = $self->Raycast(segment, shapes, maxCount, solidShapes, (void*)userData);

            ret = PyTuple_New(2);
            
            PyObject* shapeList=PyTuple_New(num);

            for (int i=0; i < num; i++) {
                PyTuple_SetItem(shapeList, i, _downcasted_shape(shapes[i]));
            }

            PyTuple_SetItem(ret, 0, SWIG_From_int(num));
            PyTuple_SetItem(ret, 1, shapeList);

            delete [] shapes;
            return ret;
        }

        PyObject* RaycastOne(const b2Segment& segment, bool solidShapes, PyObject* userData) {
            //returns tuple (float32 lambda, b2Vec2 normal, shape)
            PyObject* ret=Py_None;
            float32 lambda=1.0;
            b2Vec2* normal=new b2Vec2(0.0, 0.0);
            
            if (userData==Py_None) {
                userData=NULL;
            } else {
                Py_INCREF(userData);
            }

            b2Shape* shape = $self->RaycastOne(segment, &lambda, normal, solidShapes, (void*)userData);
            
            ret = PyTuple_New(3);

            PyTuple_SetItem(ret, 0, SWIG_From_float(lambda));
            PyTuple_SetItem(ret, 1, SWIG_NewPointerObj(SWIG_as_voidptr(normal), SWIGTYPE_p_b2Vec2, 0) );
            PyTuple_SetItem(ret, 2, _downcasted_shape(shape));

            return ret;
        }

        PyObject* Query(const b2AABB& aabb, uint32 maxCount) {
            // Returns tuple: (number of shapes, shapelist)
            PyObject* ret=Py_None;
            b2Shape** shapes=new b2Shape* [maxCount];

            if (!shapes) {
                PyErr_SetString(PyExc_MemoryError, "Insufficient memory");
                return NULL;
            }

            int32 num=$self->Query(aabb, shapes, maxCount);
            if (num < 0)
                num = 0;

            ret = PyTuple_New(2);
            
            PyObject* shapeList=PyTuple_New(num);

            for (int i=0; i < num; i++) {
                PyTuple_SetItem(shapeList, i, _downcasted_shape(shapes[i]));
            }

            PyTuple_SetItem(ret, 0, SWIG_From_int(num));
            PyTuple_SetItem(ret, 1, shapeList);

            delete [] shapes;
            return ret;
        }

        %pythoncode %{
            def GetJointList(self):
                """
                Get a list of the joints in this world
                """
                jointList = []
                joint = self._GetJointList()
                while joint:
                    jointList.append(joint)
                    joint = joint.GetNext()
                jointList.reverse() # jointlist is in reverse order
                return jointList
            def GetBodyList(self):
                """
                Get a list of the bodies in this world
                """
                bodyList = []
                body = self._GetBodyList()
                while body:
                    bodyList.append(body)
                    body = body.GetNext()
                bodyList.reverse() # bodylist is in reverse order
                return bodyList
            def GetControllerList(self):
                """
                Get a list of the controllers in this world
                """
                controllerList = []
                controller = self._GetControllerList()
                while controller:
                    controllerList.append(controller)
                    controller = controller.GetNext()
                controllerList.reverse() # controllerlist is in reverse order
                return controllerList

            def __iter__(self):
                """
                Iterates over the bodies in the world
                """
                for body in self.bodyList:
                    yield body

            gravity   = property(GetGravity   , SetGravity)
            jointList = property(GetJointList , None)
            bodyList  = property(GetBodyList  , None)
            groundBody= property(GetGroundBody, None)
            worldAABB = property(GetWorldAABB , None)
            doSleep   = property(CanSleep     , None)
            controllerList = property(GetControllerList, None)
        %}
    }

    %extend b2Shape {
    public:
        long __hash__() { return (long)self; }
        PyObject* TestSegment(const b2XForm& xf, const b2Segment& segment, float32 maxLambda) {
            int hit;
            float32 lambda=0.0f;
            b2Vec2 normal(0.0f ,0.0f);

            hit=(int)$self->TestSegment(xf, &lambda, &normal, segment, maxLambda);

            PyObject* normal_tuple=PyTuple_New(2);
            PyTuple_SetItem(normal_tuple, 0, SWIG_From_float(normal.x));
            PyTuple_SetItem(normal_tuple, 1, SWIG_From_float(normal.y));

            PyObject* ret=PyTuple_New(3);
            PyTuple_SetItem(ret, 0, SWIG_From_int(hit));
            PyTuple_SetItem(ret, 1, SWIG_From_float(lambda));
            PyTuple_SetItem(ret, 2, normal_tuple);
            return ret;
        }
        
        %pythoncode %{
        filter     = property(GetFilterData, SetFilterData)
        friction   = property(GetFriction, SetFriction)
        restitution= property(GetRestitution, SetRestitution)
        density    = property(GetDensity, SetDensity)
        isSensor   = property(IsSensor, None) # for symmetry with defn + pickling
        __eq__ = b2ShapeCompare
        __ne__ = lambda self,other: not b2ShapeCompare(self,other)
        %}
        b2CircleShape* asCircle() {
            if ($self->GetType()==e_circleShape)
                return (b2CircleShape*)$self;
            return NULL;
        }
        b2PolygonShape* asPolygon() {
            if ($self->GetType()==e_polygonShape)
                return (b2PolygonShape*)$self;
            return NULL;
        }
        b2EdgeShape* asEdge() {
            if ($self->GetType()==e_edgeShape)
                return (b2EdgeShape*)$self;
            return NULL;
        }
    }

    //Support using == on bodies, joints, and shapes
    %pythoncode %{
        def b2ShapeCompare(a, b):
            if not isinstance(a, b2Shape) or not isinstance(b, b2Shape):
                return False
            return __b2PythonShapePointerEquals__(a, b)
        def b2BodyCompare(a, b):
            if not isinstance(a, b2Body) or not isinstance(b, b2Body):
                return False
            return __b2PythonBodyPointerEquals__(a, b)
        def b2JointCompare(a, b):
            if not isinstance(a, b2Joint) or not isinstance(b, b2Joint):
                return False
            return __b2PythonJointPointerEquals__(a, b)
        def b2ControllerCompare(a, b):
            if not isinstance(a, b2Controller) or not isinstance(b, b2Controller):
                return False
            return __b2PythonControllerPointerEquals__(a, b)
    %}
    /* Properly downcast joints for all return values using b2Joint */
    %typemap(out) b2Controller* {
        if ($1) {
            switch (($1)->GetType())
            {
            case e_buoyancyController:
                $result=SWIG_NewPointerObj($1, $descriptor(b2BuoyancyController*), 0); break;
            case e_constantAccelController:
                $result=SWIG_NewPointerObj($1, $descriptor(b2ConstantAccelController*), 0); break;
            case e_constantForceController:
                $result=SWIG_NewPointerObj($1, $descriptor(b2ConstantForceController*), 0); break;
            case e_gravityController:
                $result=SWIG_NewPointerObj($1, $descriptor(b2GravityController*), 0); break;
            case e_tensorDampingController:
                $result=SWIG_NewPointerObj($1, $descriptor(b2TensorDampingController*), 0); break;
            case e_unknownController:
            default:
                $result=SWIG_NewPointerObj($1, $descriptor(b2Controller*), 0); break;
                break;
            }
        } else {
            $result=Py_None; 
            Py_INCREF($result);
        }
    }

    %typemap(out) b2Joint* {
        if ($1) {
            switch (($1)->GetType())
            {
            case e_revoluteJoint:
                $result=SWIG_NewPointerObj($1, $descriptor(b2RevoluteJoint*), 0); break;
            case e_prismaticJoint:
                $result=SWIG_NewPointerObj($1, $descriptor(b2PrismaticJoint*), 0); break;
            case e_distanceJoint:
                $result=SWIG_NewPointerObj($1, $descriptor(b2DistanceJoint*), 0); break;
            case e_pulleyJoint:
                $result=SWIG_NewPointerObj($1, $descriptor(b2PulleyJoint*), 0); break;
            case e_mouseJoint:
                $result=SWIG_NewPointerObj($1, $descriptor(b2MouseJoint*), 0); break;
            case e_gearJoint:
                $result=SWIG_NewPointerObj($1, $descriptor(b2GearJoint*), 0); break;
            case e_lineJoint:
                $result=SWIG_NewPointerObj($1, $descriptor(b2LineJoint*), 0); break;
            case e_unknownJoint:
            default:
                $result=SWIG_NewPointerObj($1, $descriptor(b2Joint*), 0); break;
                break;
            }
        } else {
            $result=Py_None; 
            Py_INCREF($result);
        }
    }

    /* Properly downcast shapes for all return values using b2Shape */
    %typemap(out) b2Shape* {
        if ($1) {
            switch (($1)->GetType())
            {
            case e_circleShape:
                $result=SWIG_NewPointerObj($1, $descriptor(b2CircleShape*), 0); break;
            case e_polygonShape:
                $result=SWIG_NewPointerObj($1, $descriptor(b2PolygonShape*), 0); break;
            case e_edgeShape:
                $result=SWIG_NewPointerObj($1, $descriptor(b2EdgeShape*), 0); break;
            case e_unknownShape:
            default:
                $result=SWIG_NewPointerObj($1, $descriptor(b2Shape*), 0); break;
            }
        } else {
            $result=Py_None; 
            Py_INCREF($result);
        }
    }


    // Clean up naming. We do not need m_* on the Python end.
    %rename(localAnchor) b2MouseJoint::m_localAnchor;
    %rename(target)      b2MouseJoint::m_target;
    %rename(impulse)     b2MouseJoint::m_impulse;
    %rename(mass)        b2MouseJoint::m_mass;
    %rename(C)           b2MouseJoint::m_C;
    %rename(maxForce)    b2MouseJoint::m_maxForce;
    %rename(frequencyHz) b2MouseJoint::m_frequencyHz;
    %rename(dampingRatio)b2MouseJoint::m_dampingRatio;
    %rename(beta)        b2MouseJoint::m_beta;
    %rename(gamma)       b2MouseJoint::m_gamma;
    %extend b2MouseJoint {
    public:
        %pythoncode %{
        %}
    }

    %rename(ground1)       b2GearJoint::m_ground1;
    %rename(ground2)       b2GearJoint::m_ground2;
    %rename(revolute1)     b2GearJoint::m_revolute1;
    %rename(prismatic1)    b2GearJoint::m_prismatic1;
    %rename(revolute2)     b2GearJoint::m_revolute2;
    %rename(prismatic2)    b2GearJoint::m_prismatic2;
    %rename(groundAnchor1) b2GearJoint::m_groundAnchor1;
    %rename(groundAnchor2) b2GearJoint::m_groundAnchor2;
    %rename(localAnchor1)  b2GearJoint::m_localAnchor1;
    %rename(localAnchor2)  b2GearJoint::m_localAnchor2;
    %rename(J)             b2GearJoint::m_J;
    %rename(constant)      b2GearJoint::m_constant;
    %rename(ratio)         b2GearJoint::m_ratio;
    %rename(mass)          b2GearJoint::m_mass;
    %rename(impulse)       b2GearJoint::m_impulse;
    %extend b2GearJoint {
    public:
        %pythoncode %{
            joint1 = property(lambda self: (self.revolute1 and self.revolute1) or self.prismatic1, None)
            joint2 = property(lambda self: (self.revolute2 and self.revolute2) or self.prismatic2, None)
        %}
    }

    %rename(localAnchor1) b2DistanceJoint::m_localAnchor1;
    %rename(localAnchor2) b2DistanceJoint::m_localAnchor2;
    %rename(u)            b2DistanceJoint::m_u;
    %rename(frequencyHz)  b2DistanceJoint::m_frequencyHz;
    %rename(dampingRatio) b2DistanceJoint::m_dampingRatio;
    %rename(gamma)        b2DistanceJoint::m_gamma;
    %rename(bias)         b2DistanceJoint::m_bias;
    %rename(impulse)      b2DistanceJoint::m_impulse;
    %rename(mass)         b2DistanceJoint::m_mass;
    %rename(length)       b2DistanceJoint::m_length;

    %extend b2DistanceJoint {
    public:
        %pythoncode %{
        %}
    }

    %rename(localAnchor1)    b2PrismaticJoint::m_localAnchor1;
    %rename(localAnchor2)    b2PrismaticJoint::m_localAnchor2;
    %rename(localXAxis1)     b2PrismaticJoint::m_localXAxis1;
    %rename(localYAxis1)     b2PrismaticJoint::m_localYAxis1;
    %rename(referenceAngle)  b2PrismaticJoint::m_refAngle; // symmetry with defn
    %rename(axis)            b2PrismaticJoint::m_axis;
    %rename(perp)            b2PrismaticJoint::m_perp;
    %rename(s1)              b2PrismaticJoint::m_s1;
    %rename(s2)              b2PrismaticJoint::m_s2;
    %rename(a1)              b2PrismaticJoint::m_a1;
    %rename(a2)              b2PrismaticJoint::m_a2;
    %rename(K)               b2PrismaticJoint::m_K;
    %rename(impulse)         b2PrismaticJoint::m_impulse;
    %rename(motorMass)       b2PrismaticJoint::m_motorMass;
    %rename(motorImpulse)    b2PrismaticJoint::m_motorImpulse;
    %rename(lowerTranslation)b2PrismaticJoint::m_lowerTranslation;
    %rename(upperTranslation)b2PrismaticJoint::m_upperTranslation;
    %rename(maxMotorForce)   b2PrismaticJoint::m_maxMotorForce;
    %rename(motorSpeed)      b2PrismaticJoint::m_motorSpeed;
    %rename(enableLimit)     b2PrismaticJoint::m_enableLimit;
    %rename(enableMotor)     b2PrismaticJoint::m_enableMotor;
    %rename(limitState)      b2PrismaticJoint::m_limitState;
    %extend b2PrismaticJoint {
    public:
        %pythoncode %{
        %}
    }

    %rename(ground)          b2PulleyJoint::m_ground;
    %rename(groundAnchor1)   b2PulleyJoint::m_groundAnchor1;
    %rename(groundAnchor2)   b2PulleyJoint::m_groundAnchor2;
    %rename(localAnchor1)    b2PulleyJoint::m_localAnchor1;
    %rename(localAnchor2)    b2PulleyJoint::m_localAnchor2;
    %rename(u1)              b2PulleyJoint::m_u1;
    %rename(u2)              b2PulleyJoint::m_u2;
    %rename(constant)        b2PulleyJoint::m_constant;
    %rename(ratio)           b2PulleyJoint::m_ratio;
    %rename(maxLength1)      b2PulleyJoint::m_maxLength1;
    %rename(maxLength2)      b2PulleyJoint::m_maxLength2;
    %rename(pulleyMass)      b2PulleyJoint::m_pulleyMass;
    %rename(limitMass1)      b2PulleyJoint::m_limitMass1;
    %rename(limitMass2)      b2PulleyJoint::m_limitMass2;
    %rename(impulse)         b2PulleyJoint::m_impulse;
    %rename(limitImpulse1)   b2PulleyJoint::m_limitImpulse1;
    %rename(limitImpulse2)   b2PulleyJoint::m_limitImpulse2;
    %rename(state)           b2PulleyJoint::m_state;
    %rename(limitState1)     b2PulleyJoint::m_limitState1;
    %rename(limitState2)     b2PulleyJoint::m_limitState2;
    %extend b2PulleyJoint {
    public:
        %pythoncode %{
        length1 = property(GetLength1, None)
        length2 = property(GetLength2, None)
        %}
    }

    %rename(localAnchor1)     b2RevoluteJoint::m_localAnchor1;
    %rename(localAnchor2)     b2RevoluteJoint::m_localAnchor2;
    %rename(impulse)          b2RevoluteJoint::m_impulse;
    %rename(motorImpulse)     b2RevoluteJoint::m_motorImpulse;
    %rename(mass)             b2RevoluteJoint::m_mass;
    %rename(motorMass)        b2RevoluteJoint::m_motorMass;
    %rename(enableMotor)      b2RevoluteJoint::m_enableMotor;
    %rename(maxMotorTorque)   b2RevoluteJoint::m_maxMotorTorque;
    %rename(motorSpeed)       b2RevoluteJoint::m_motorSpeed;
    %rename(enableLimit)      b2RevoluteJoint::m_enableLimit;
    %rename(referenceAngle)   b2RevoluteJoint::m_referenceAngle;
    %rename(lowerAngle)       b2RevoluteJoint::m_lowerAngle;
    %rename(upperAngle)       b2RevoluteJoint::m_upperAngle;
    %rename(limitState)       b2RevoluteJoint::m_limitState;
    %extend b2RevoluteJoint {
    public:
        %pythoncode %{
        %}
    }

    %rename(localAnchor1)    b2LineJoint::m_localAnchor1;
    %rename(localAnchor2)    b2LineJoint::m_localAnchor2;
    %rename(localXAxis1)     b2LineJoint::m_localXAxis1;
    %rename(localYAxis1)     b2LineJoint::m_localYAxis1;
    %rename(axis)            b2LineJoint::m_axis;
    %rename(perp)            b2LineJoint::m_perp;
    %rename(s1)              b2LineJoint::m_s1;
    %rename(s2)              b2LineJoint::m_s2;
    %rename(a1)              b2LineJoint::m_a1;
    %rename(a2)              b2LineJoint::m_a2;
    %rename(K)               b2LineJoint::m_K;
    %rename(impulse)         b2LineJoint::m_impulse;
    %rename(motorMass)       b2LineJoint::m_motorMass;
    %rename(motorImpulse)    b2LineJoint::m_motorImpulse;
    %rename(lowerTranslation)b2LineJoint::m_lowerTranslation;
    %rename(upperTranslation)b2LineJoint::m_upperTranslation;
    %rename(maxMotorForce)   b2LineJoint::m_maxMotorForce;
    %rename(motorSpeed)      b2LineJoint::m_motorSpeed;
    %rename(enableLimit)     b2LineJoint::m_enableLimit;
    %rename(enableMotor)     b2LineJoint::m_enableMotor;
    %rename(limitState)      b2LineJoint::m_limitState;
    %extend b2LineJoint {
    public:
        %pythoncode %{
        %}
    }
    %include "Dynamics/Joints/b2Joint.h"

    %extend b2JointDef {
    public:
        %pythoncode %{
        def typeName(self):
            """
            Return the name of the joint from:
             Unknown, Mouse, Gear, Distance, Prismatic, Pulley, Revolute
            """
            types = { e_unknownJoint  : "Unknown",
                      e_mouseJoint    : "Mouse", 
                      e_gearJoint     : "Gear",
                      e_distanceJoint : "Distance",
                      e_prismaticJoint: "Prismatic",
                      e_pulleyJoint   : "Pulley",
                      e_revoluteJoint : "Revolute",
                      e_lineJoint     : "Line" }
            return types[self.type]
        %}
    }

    %extend b2Controller {
        long __hash__() { return (long)self; }
        %pythoncode %{
        def GetBodyList(self):
            bodyList = []
            c_edge = self._GetBodyList()
            while c_edge:
                bodyList.append(c_edge.body)
                c_edge = c_edge.nextBody
            bodyList.reverse() # bodylist is in reverse order
            return bodyList

        def __iter__(self):
            """
            Iterates over the bodies in the controller
            """
            for body in self.bodyList:
                yield body

        __eq__ = b2ControllerCompare
        __ne__ = lambda self,other: not b2ControllerCompare(self,other)
        type = property(GetType, None)
        bodyList = property(GetBodyList, None)
        %}
        
        b2BuoyancyController* _asBuoyancyController() {
            if ($self->GetType()==e_buoyancyController)
                return (b2BuoyancyController*)$self;
            return NULL;
        }
        b2ConstantAccelController* _asConstantAccelController() {
            if ($self->GetType()==e_constantAccelController)
                return (b2ConstantAccelController*)$self;
            return NULL;
        }
        b2ConstantForceController* _asConstantForceController() {
            if ($self->GetType()==e_constantForceController)
                return (b2ConstantForceController*)$self;
            return NULL;
        }
        b2GravityController* _asGravityController() {
            if ($self->GetType()==e_gravityController)
                return (b2GravityController*)$self;
            return NULL;
        }
        b2TensorDampingController* _asTensorDampingController() {
            if ($self->GetType()==e_tensorDampingController)
                return (b2TensorDampingController*)$self;
            return NULL;
        }
    }

    %extend b2Joint {
    public:
        long __hash__() { return (long)self; }
        %pythoncode %{
        __eq__ = b2JointCompare
        __ne__ = lambda self,other: not b2JointCompare(self,other)
        type    =property(GetType    , None)
        body1   =property(GetBody1   , None)
        body2   =property(GetBody2   , None)
        collideConnected=property(GetCollideConnected, None)
        def typeName(self):
            """
            Return the name of the joint from:
             Unknown, Mouse, Gear, Distance, Prismatic, Pulley, Revolute
            """
            types = { e_unknownJoint  : "Unknown",
                      e_mouseJoint    : "Mouse", 
                      e_gearJoint     : "Gear",
                      e_distanceJoint : "Distance",
                      e_prismaticJoint: "Prismatic",
                      e_pulleyJoint   : "Pulley",
                      e_revoluteJoint : "Revolute",
                      e_lineJoint     : "Line" }
            return types[self.GetType()]
        %}

        b2MouseJoint* asMouseJoint() {
            if ($self->GetType()==e_mouseJoint)
                return (b2MouseJoint*)$self;
            return NULL;
        }

        b2GearJoint* asGearJoint() {
            if ($self->GetType()==e_gearJoint)
                return (b2GearJoint*)$self;
            return NULL;
        }

        b2DistanceJoint* asDistanceJoint() {
            if ($self->GetType()==e_distanceJoint)
                return (b2DistanceJoint*)$self;
            return NULL;
        }

        b2PrismaticJoint* asPrismaticJoint() {
            if ($self->GetType()==e_prismaticJoint)
                return (b2PrismaticJoint*)$self;
            return NULL;
        }

        b2PulleyJoint* asPulleyJoint() {
            if ($self->GetType()==e_pulleyJoint)
                return (b2PulleyJoint*)$self;
            return NULL;
        }

        b2RevoluteJoint* asRevoluteJoint() {
            if ($self->GetType()==e_revoluteJoint)
                return (b2RevoluteJoint*)$self;
            return NULL;
        }

        b2LineJoint* asLineJoint() {
            if ($self->GetType()==e_lineJoint)
                return (b2LineJoint*)$self;
            return NULL;
        }
    }

    %extend b2CircleShape {
    public:
        %pythoncode %{
        __eq__ = b2ShapeCompare
        __ne__ = lambda self,other: not b2ShapeCompare(self,other)
        radius = property(GetRadius, None)
        localPosition = property(GetLocalPosition, None)
        %}
    }

    //Let python access all the vertices in the b2PolygonDef/Shape
    %extend b2PolygonShape {
    public:
        %pythoncode %{
        __eq__ = b2ShapeCompare
        __ne__ = lambda self,other: not b2ShapeCompare(self,other)
        def __repr__(self):
            return "b2PolygonShape(vertices: %s count: %d)" % (self.getVertices_tuple(), self.GetVertexCount())
        def getCoreVertices_tuple(self):
            """Returns all of the core vertices as a list of tuples [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.GetVertexCount()):
                vertices.append( (self.getCoreVertex(i).x, self.getCoreVertex(i).y ) )
            return vertices
        def getCoreVertices_b2Vec2(self):
            """Returns all of the core vertices as a list of b2Vec2's [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.GetVertexCount()):
                vertices.append(self.getCoreVertex(i))
            return vertices
        def getVertices_tuple(self):
            """Returns all of the vertices as a list of tuples [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.GetVertexCount()):
                vertices.append( (self.getVertex(i).x, self.getVertex(i).y ) )
            return vertices
        def getVertices_b2Vec2(self):
            """Returns all of the vertices as a list of b2Vec2's [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.GetVertexCount()):
                vertices.append(self.getVertex(i))
            return vertices
        def getNormals_tuple(self):
            """Returns all of the normals as a list of tuples [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.GetVertexCount()):
                vertices.append( (self.getNormal(i).x, self.getNormal(i).y ) )
            return vertices
        def getNormals_b2Vec2(self):
            """Returns all of the normals as a list of b2Vec2's [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.GetVertexCount()):
                vertices.append(self.getNormal(i))
            return vertices
        def __iter__(self):
            """
            Iterates over the vertices in the polygon
            """
            for v in self.vertices:
                yield v

        vertices = property(getVertices_tuple, None)
        coreVertices = property(getCoreVertices_tuple, None)
        normals = property(getNormals_tuple, None)
        %}
        const b2Vec2* getVertex(uint16 vnum) {
            if (vnum >= b2_maxPolygonVertices || vnum >= self->GetVertexCount()) return NULL;
            return &( $self->GetVertices() [vnum] );
        }
        const b2Vec2* getCoreVertex(uint16 vnum) {
            if (vnum >= b2_maxPolygonVertices || vnum >= self->GetVertexCount()) return NULL;
            return &( $self->GetCoreVertices() [vnum] );
        }
        const b2Vec2* getNormal(uint16 vnum) {
            if (vnum >= b2_maxPolygonVertices || vnum >= self->GetVertexCount()) return NULL;
            return &( $self->GetNormals() [vnum] );
        }
    }


    %extend b2EdgeChainDef {
    public:
        %pythoncode %{
        def __repr__(self):
            return "b2EdgeDef(vertices: %s count: %d)" % (self.getVertices_tuple(), self.vertexCount)
        def __del__(self):
            """Cleans up by freeing the allocated vertex array"""
            super(b2EdgeChainDef, self).__del__()
            self._cleanUp()
        def getVertices_tuple(self):
            """Returns all of the vertices as a list of tuples [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.vertexCount):
                vertices.append( (self.getVertex(i).x, self.getVertex(i).y ) )
            return vertices
        def getVertices_b2Vec2(self):
            """Returns all of the vertices as a list of b2Vec2's [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.vertexCount):
                vertices.append(self.getVertex(i))
            return vertices
        def setVertices(self, vertices):
            """Sets all of the vertices given a tuple 
                in the format ( (x1,y1), (x2,y2) ... (xN,yN) )
                where each vertex is either a list/tuple/b2Vec2"""
            self._allocateVertices(len(vertices))
            for i in range(0, self.vertexCount):
                self.setVertex(i, vertices[i])
        setVertices_tuple = setVertices  # pre 202b1 compatibility
        setVertices_b2Vec2 = setVertices # pre 202b1 compatibility
        vertices = property(getVertices_tuple, setVertices)

        %}
        void _cleanUp() {
            if ($self->vertexCount > 0 && $self->vertices)
                delete [] $self->vertices;
            $self->vertices = NULL;
            $self->vertexCount = 0;
        }
        void _allocateVertices(uint16 _count) {
            if ($self->vertexCount > 0 && $self->vertices)
                delete [] $self->vertices;
            $self->vertices = new b2Vec2 [_count];
            if (!$self->vertices) {
                $self->vertexCount = 0;
                PyErr_SetString(PyExc_MemoryError, "Insufficient memory");
                return;
            }
            $self->vertexCount = _count;
        }
        b2Vec2* getVertex(uint16 vnum) {
            if (vnum >= $self->vertexCount) return NULL;
            return &( $self->vertices[vnum] );
        }
        void setVertex(uint16 vnum, b2Vec2& value) {
            if (vnum < $self->vertexCount)
                $self->vertices[vnum].Set(value.x, value.y);
        }
        void setVertex(uint16 vnum, float32 x, float32 y) {
            if (vnum < $self->vertexCount)
                $self->vertices[vnum].Set(x, y);
        }
    }

    %extend b2EdgeShape {
        %pythoncode %{
            def GetVertices(self):
                vertices = []
                edge = self
                while edge:
                    vertices.append( edge.vertex1 )
                    last = edge.vertex2
                    edge=edge.next
                    if edge==self: # a loop
                        vertices.extend( [edge.vertex1, edge.vertex2] )
                        return vertices
                vertices.append( last )
                return vertices

            length      = property(GetLength,      None)
            vertex1     = property(GetVertex1,     None)
            vertex2     = property(GetVertex2,     None)
            coreVertex1 = property(GetCoreVertex1, None)
            coreVertex2 = property(GetCoreVertex2, None)
            next        = property(GetNextEdge,    None)
            prev        = property(GetPrevEdge,    None)
        %}
    }

    %extend b2PolygonDef{
    public:
        %pythoncode %{
        def __repr__(self):
            return "b2PolygonDef(vertices: %s count: %d)" % (self.vertices, self.vertexCount)
        def checkValues(self):
            return b2CheckPolygonDef(self)
        def getVertices_tuple(self):
            """Returns all of the vertices as a list of tuples [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.vertexCount):
                vertices.append( (self.getVertex(i).x, self.getVertex(i).y ) )
            return vertices
        def getVertices_b2Vec2(self):
            """Returns all of the vertices as a list of b2Vec2's [ (x1,y1), (x2,y2) ... (xN,yN) ]"""
            vertices = []
            for i in range(0, self.vertexCount):
                vertices.append(self.getVertex(i))
            return vertices
        def setVertices(self, vertices):
            """Sets all of the vertices given a tuple 
                in the format ( (x1,y1), (x2,y2) ... (xN,yN) )
                where each vertex is a list/tuple/b2Vec2"""
            if len(vertices) > b2_maxPolygonVertices:
                raise ValueError()
            self.vertexCount = len(vertices)
            for i in range(0, self.vertexCount):
                self.setVertex(i, vertices[i]) # possible on pyBox2D >= r2.0.2b1
        setVertices_tuple = setVertices  # pre 202b1 compatibility
        setVertices_b2Vec2 = setVertices # pre 202b1 compatibility
        vertices = property(getVertices_tuple, setVertices)

        %}
        b2Vec2* getVertex(uint16 vnum) {
            if (vnum >= b2_maxPolygonVertices || vnum >= self->vertexCount) return NULL;
            return &( $self->vertices[vnum] );
        }
        void setVertex(uint16 vnum, b2Vec2& value) {
            if (vnum >= b2_maxPolygonVertices) return;
            $self->vertices[vnum].Set(value.x, value.y);
        }
        void setVertex(uint16 vnum, float32 x, float32 y) {
            if (vnum >= b2_maxPolygonVertices) return;
            $self->vertices[vnum].Set(x, y);
        }
    }

    %extend b2Color {
        %pythoncode %{
        __iter__ = lambda self: iter((self.r, self.g, self.b)) 
         %}
    }

    // Vector class
    %extend b2Vec2 {
        b2Vec2(b2Vec2& other) {
            return new b2Vec2(other.x, other.y);
        }

        %pythoncode %{
        __iter__ = lambda self: iter( (self.x, self.y) )
        __eq__ = lambda self, other: self.__equ(other)
        __ne__ = lambda self,other: not self.__equ(other)
        def __len__(self):
            return 2
        def __neg__(self):
            return b2Vec2(-self.x, -self.y)
        def __copy__(self):
            return b2Vec2(self.x, self.y)
        def __repr__(self):
            return "b2Vec2(%g,%g)" % (self.x, self.y)
        def tuple(self):
            """
            Return the vector as a tuple (x,y)
            """
            return tuple(self)
        def fromTuple(self, tuple):
            """
            *DEPRECATED*
            Set the vector to the values found in the tuple (x,y)
            You should use:
                value = b2Vec2(*tuple)
            """
            self.x, self.y = tuple
            return self
        def copy(self):
            """
            Return a copy of the vector.
            Remember that the following:
                a = b2Vec2()
                b = a
            Does not copy the vector itself, but b now refers to a.
            """
            return b2Vec2(self.x, self.y)
        def __iadd__(self, other):
            self.add_vector(other)
            return self
        def __isub__(self, other):
            self.sub_vector(other)
            return self
        def __imul__(self, a):
            self.mul_float(a)
            return self
        def __idiv__(self, a):
            self.div_float(a)
            return self
        %}
        float32 cross(b2Vec2& other) {
            return $self->x * other.y - $self->y * other.x;
        }
        b2Vec2 cross(float32 s) {
            return b2Vec2(s * $self->y, -s * $self->x);
        }

        float32 __getitem__(int i) {
            if (i==0) 
                return $self->x;
            else if (i==1) 
                return $self->y;
            PyErr_SetString(PyExc_IndexError, "Index must be in (0,1)");
            return 0.0f;
        }
        void __setitem__(int i, float32 value) {
            if (i==0) 
                $self->x=value;
            else if (i==1) 
                $self->y=value;
            else
                PyErr_SetString(PyExc_IndexError, "Index must be in (0,1)");
        }
        bool __equ(b2Vec2& other) {
            return ($self->x == other.x && $self->y == other.y);
        }
        float32 dot(b2Vec2& other) {
            return $self->x * other.x + $self->y * other.y;
        }
        b2Vec2 __truediv__(float32 a) { //python 3k
            return b2Vec2($self->x / a, $self->y / a);
        }
        b2Vec2 __div__(float32 a) {
            return b2Vec2($self->x / a, $self->y / a);
        }
        b2Vec2 __mul__(float32 a) {
            return b2Vec2($self->x * a, $self->y * a);
        }
        b2Vec2 __add__(b2Vec2* other) {
            return b2Vec2($self->x + other->x, $self->y + other->y);
        }
        b2Vec2 __sub__(b2Vec2* other) {
            return b2Vec2($self->x - other->x, $self->y - other->y);
        }

        b2Vec2 __rmul__(float32 a) {
            return b2Vec2($self->x * a, $self->y * a);
        }
        b2Vec2 __rdiv__(float32 a) {
            return b2Vec2($self->x / a, $self->y / a);
        }
        void __div_float(float32 a) {
            $self->x /= a;
            $self->y /= a;
        }
    }

    %extend b2Body {
        long __hash__() { return (long)self; }
        %pythoncode %{
            __eq__ = b2BodyCompare
            __ne__ = lambda self,other: not b2BodyCompare(self,other)

            def setAngle(self, angle):
                """
                Set the angle without altering the position

                angle in radians.
                """
                self.SetXForm(self.position, angle)
            def setPosition(self, position):
                """
                Set the position without altering the angle
                """
                self.SetXForm(position, self.GetAngle())

            def getMassData(self):
                """
                Get a b2MassData object that represents this b2Body

                NOTE: To just get the mass, use body.mass (body.GetMass())
                """

                ret = b2MassData()
                ret.mass = self.GetMass()
                ret.I    = self.GetInertia()
                ret.center=self.GetLocalCenter()
                return ret
            
            def GetShapeList(self):
                """
                Get a list of the shapes in this body

                Defaults to returning the typecasted objects.

                e.g., if there is a b2CircleShape and a b2PolygonShape:
                GetShapeList(True) = [b2CircleShape, b2PolygonShape]
                GetShapeList(False)= [b2Shape, b2Shape]
                """
                shapeList = []
                shape = self._GetShapeList()
                while shape:
                    shapeList.append(shape)
                    shape = shape.GetNext()
                shapeList.reverse() # shapelist is in reverse order
                return shapeList

            def __iter__(self):
                """
                Iterates over the shapes in the body
                """
                for shape in self.shapeList:
                    yield shape

            massData      = property(getMassData        , SetMass)
            position      = property(GetPosition        , setPosition)
            angle         = property(GetAngle           , setAngle)
            linearDamping = property(GetLinearDamping   , SetLinearDamping)
            angularDamping= property(GetAngularDamping  , SetAngularDamping)
            allowSleep    = property(IsAllowSleeping    , AllowSleeping)
            isSleeping    = property(IsSleeping         , None)
            fixedRotation = property(IsFixedRotation    , SetFixedRotation)
            isBullet      = property(IsBullet           , SetBullet)
            angularVelocity=property(GetAngularVelocity , SetAngularVelocity)
            linearVelocity =property(GetLinearVelocity  , SetLinearVelocity)
            shapeList      =property(GetShapeList       , None)
        %}
    }

    %rename (__b2Distance__) b2Distance(b2Vec2* x1, b2Vec2* x2, const b2Shape* shape1, const b2XForm& xf1, const b2Shape* shape2, const b2XForm& xf2);
    %inline %{
        //Add a b2Distance:
        // dist, x1, x2 = b2Distance(shape1, xf1, shape2, xf2)
        PyObject* b2Distance(const b2Shape* shape1, const b2XForm& xf1, const b2Shape* shape2, const b2XForm& xf2) {
            PyObject* ret=PyTuple_New(3);
            b2Vec2* x1=new b2Vec2;
            b2Vec2* x2=new b2Vec2;
            float dist=b2Distance(x1,x2,shape1,xf1,shape2,xf2);
            PyTuple_SetItem(ret, 0, SWIG_From_float(dist)); 
            PyTuple_SetItem(ret, 1, SWIG_NewPointerObj(SWIG_as_voidptr(x1), SWIGTYPE_p_b2Vec2, 0 ));
            PyTuple_SetItem(ret, 2, SWIG_NewPointerObj(SWIG_as_voidptr(x2), SWIGTYPE_p_b2Vec2, 0 ));
            return ret;
        }


        PyObject* _downcasted_shape(b2Shape* shape) {
            if (!shape) {
                Py_INCREF(Py_None);
                return Py_None; 
            }
            switch (shape->GetType())
            {
            case e_circleShape:
                return SWIG_NewPointerObj(shape, SWIGTYPE_p_b2CircleShape, 0);
            case e_polygonShape:
                return SWIG_NewPointerObj(shape, SWIGTYPE_p_b2PolygonShape, 0);
            case e_edgeShape:
                return SWIG_NewPointerObj(shape, SWIGTYPE_p_b2EdgeShape, 0); 
            case e_unknownShape:
            default:
                return SWIG_NewPointerObj(shape, SWIGTYPE_p_b2Shape, 0);
            }
        }
    %}

    %ignore _downcasted_shape;

    /* Additional supporting C++ code */
    %typemap(out) bool b2CheckPolygonDef(b2PolygonDef*) {
        if (!$1) 
            SWIG_fail;
        else
            $result = SWIG_From_bool(static_cast< bool >($1));
    }

    %feature("docstring") b2CheckPolygonDef "
        Checks the Polygon definition to see if upon creation it will cause an assertion.
        Raises ValueError if an assertion would be raised.

        b2PolygonDef* poly     - the polygon definition
        bool additional_checks - whether or not to run additional checks

        Additional checking: usually only in DEBUG mode on the C++ code.

        While shapes that pass this test can be created without assertions,
        they will ultimately create unexpected behavior. It's recommended
        to _not_ use any polygon that fails this test.
    ";

    %feature("docstring") b2AABBOverlaps "Checks if two AABBs overlap, or if a point
    lies in an AABB
    
    b2AABBOverlaps(AABB1, [AABB2/point])
    ";

    %inline %{
        // Add some functions that might be commonly used
        bool b2AABBOverlaps(const b2AABB& aabb, const b2Vec2& point) {
            //If point is in aabb (including a small buffer around it), return true.
            if (point.x < (aabb.upperBound.x + B2_FLT_EPSILON) &&
                point.x > (aabb.lowerBound.x - B2_FLT_EPSILON) &&
                point.y < (aabb.upperBound.y + B2_FLT_EPSILON) &&
                point.y > (aabb.lowerBound.y - B2_FLT_EPSILON))
                    return true;
            return false;
        }
        
        bool b2AABBOverlaps(const b2AABB& aabb, const b2AABB& aabb2) {
            //If aabb and aabb2 overlap, return true. (modified from b2BroadPhase::InRange)
            b2Vec2 d = b2Max(aabb.lowerBound - aabb2.upperBound, aabb2.lowerBound - aabb.upperBound);
            return b2Max(d.x, d.y) < 0.0f;
        }

        // Modified from the b2PolygonShape constructor
        // Should be as accurate as the original version
        b2Vec2 __b2ComputeCentroid(const b2Vec2* vs, int32 count) {
            b2Vec2 c; c.Set(0.0f, 0.0f);
            if (count < 3 || count >= b2_maxPolygonVertices) {
                PyErr_SetString(PyExc_ValueError, "Vertex count must be >= 3 and < b2_maxPolygonVertices");
                return c;
            }

            float32 area = 0.0f;

            // pRef is the reference point for forming triangles.
            // It's location doesn't change the result (except for rounding error).
            b2Vec2 pRef(0.0f, 0.0f);

            const float32 inv3 = 1.0f / 3.0f;

            for (int32 i = 0; i < count; ++i)
            {
                // Triangle vertices.
                b2Vec2 p1 = pRef;
                b2Vec2 p2 = vs[i];
                b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

                b2Vec2 e1 = p2 - p1;
                b2Vec2 e2 = p3 - p1;

                float32 D = b2Cross(e1, e2);

                float32 triangleArea = 0.5f * D;
                area += triangleArea;

                // Area weighted centroid
                c += triangleArea * inv3 * (p1 + p2 + p3);
            }

            // Centroid
            if (area <= B2_FLT_EPSILON) {
                PyErr_SetString(PyExc_ValueError, "ComputeCentroid: area <= FLT_EPSILON");
                return c;
            }

            c *= 1.0f / area;
            return c;
        }

        bool __b2ComputeOBB(b2OBB* obb, const b2Vec2* vs, int32 count)
        {
            if (count < 3 || count >= b2_maxPolygonVertices) {
                PyErr_SetString(PyExc_ValueError, "Vertex count must be >= 3 and < b2_maxPolygonVertices");
                return false;
            }

            b2Vec2 p[b2_maxPolygonVertices + 1];
            for (int32 i = 0; i < count; ++i)
            {
                p[i] = vs[i];
            }
            p[count] = p[0];

            float32 minArea = B2_FLT_MAX;
            
            for (int32 i = 1; i <= count; ++i)
            {
                b2Vec2 root = p[i-1];
                b2Vec2 ux = p[i] - root;
                float32 length = ux.Normalize();
                if (length <= B2_FLT_EPSILON) {
                    PyErr_SetString(PyExc_ValueError, "ComputeOBB: length <= B2_FLT_EPSILON");
                    return false;
                }
                b2Vec2 uy(-ux.y, ux.x);
                b2Vec2 lower(B2_FLT_MAX, B2_FLT_MAX);
                b2Vec2 upper(-B2_FLT_MAX, -B2_FLT_MAX);

                for (int32 j = 0; j < count; ++j)
                {
                    b2Vec2 d = p[j] - root;
                    b2Vec2 r;
                    r.x = b2Dot(ux, d);
                    r.y = b2Dot(uy, d);
                    lower = b2Min(lower, r);
                    upper = b2Max(upper, r);
                }

                float32 area = (upper.x - lower.x) * (upper.y - lower.y);
                if (area < 0.95f * minArea)
                {
                    minArea = area;
                    obb->R.col1 = ux;
                    obb->R.col2 = uy;
                    b2Vec2 center = 0.5f * (lower + upper);
                    obb->center = root + b2Mul(obb->R, center);
                    obb->extents = 0.5f * (upper - lower);
                }
            }

            if (minArea >= B2_FLT_MAX) {
                PyErr_SetString(PyExc_ValueError, "ComputeOBB: minArea >= B2_FLT_MAX");
                return false;
            }
            return true;
        }

        bool b2CheckPolygonDef(b2PolygonDef* poly, bool additional_checks=true) {
            // Get the vertices transformed into the body frame.
            if (poly->vertexCount < 3 || poly->vertexCount >= b2_maxPolygonVertices) {
                PyErr_SetString(PyExc_ValueError, "Vertex count must be >= 3 and < b2_maxPolygonVertices");
                return false;
            }

            // Compute normals. Ensure the edges have non-zero length.
            b2Vec2 m_normals[b2_maxPolygonVertices];
            for (int32 i = 0; i < poly->vertexCount; ++i)
            {
                int32 i1 = i;
                int32 i2 = i + 1 < poly->vertexCount ? i + 1 : 0;
                b2Vec2 edge = poly->vertices[i2] - poly->vertices[i1];
                if (edge.LengthSquared() <= B2_FLT_EPSILON * B2_FLT_EPSILON) {
                    PyErr_SetString(PyExc_ValueError, "edge.LengthSquared < FLT_EPSILON**2");
                    return false;
                }
                
                m_normals[i] = b2Cross(edge, 1.0f);
                m_normals[i].Normalize();
            }

            // Compute the polygon centroid.
            b2Vec2 m_centroid = __b2ComputeCentroid(poly->vertices, poly->vertexCount);
            
            // Compute the oriented bounding box.
            b2OBB m_obb;
            __b2ComputeOBB(&m_obb, poly->vertices, poly->vertexCount);

            if (PyErr_Occurred()) 
                return false;

            // Create core polygon shape by shifting edges inward.
            // Also compute the min/max radius for CCD.
            for (int32 i = 0; i < poly->vertexCount; ++i)
            {
                int32 i1 = i - 1 >= 0 ? i - 1 : poly->vertexCount - 1;
                int32 i2 = i;

                b2Vec2 n1 = m_normals[i1];
                b2Vec2 n2 = m_normals[i2];
                b2Vec2 v = poly->vertices[i] - m_centroid;

                b2Vec2 d;
                d.x = b2Dot(n1, v) - b2_toiSlop;
                d.y = b2Dot(n2, v) - b2_toiSlop;

                // Shifting the edge inward by b2_toiSlop should
                // not cause the plane to pass the centroid.

                // Your shape has a radius/extent less than b2_toiSlop.
                if (d.x < 0.0f) {
                    PyErr_SetString(PyExc_ValueError, "Your shape has a radius/extent less than b2_toiSlop. (d.x < 0.0)");
                    return false;
                } else if (d.y < 0.0f) {
                    PyErr_SetString(PyExc_ValueError, "Your shape has a radius/extent less than b2_toiSlop. (d.y < 0.0)");
                    return false;
                }
                b2Mat22 A;
                A.col1.x = n1.x; A.col2.x = n1.y;
                A.col1.y = n2.x; A.col2.y = n2.y;
                //m_coreVertices[i] = A.Solve(d) + m_centroid;
            }

            if (!additional_checks)
                return true;

            // Ensure the polygon is convex.
            for (int32 i = 0; i < poly->vertexCount; ++i)
            {
                for (int32 j = 0; j < poly->vertexCount; ++j)
                {
                    // Do not check vertices on the current edge.
                    if (j == i || j == (i + 1) % poly->vertexCount)
                        continue;
                    
                    float32 s = b2Dot(m_normals[i], poly->vertices[j] - poly->vertices[i]);
                    if (s >= -b2_linearSlop) {
                        PyErr_SetString(PyExc_ValueError, "Your polygon is non-convex (it has an indentation), or it's too skinny");
                        return false;
                    }
                }
            }

            // Ensure the polygon is counter-clockwise.
            for (int32 i = 1; i < poly->vertexCount; ++i)
            {
                float32 cross = b2Cross(m_normals[i-1], m_normals[i]);

                // Keep asinf happy.
                cross = b2Clamp(cross, -1.0f, 1.0f);

                float32 angle = asinf(cross);
                if (angle <= b2_angularSlop) {
                    PyErr_SetString(PyExc_ValueError, "You have consecutive edges that are almost parallel on your polygon.");
                    return false;
                }
            }
            return true;
        }

        /* As of Box2D SVN r191, these functions are no longer in b2Math.h,
           so re-add them here for backwards compatibility */
        #define RAND_LIMIT      32767      
        // Random number in range [-1,1]
        float32 b2Random()      
        {      
                float32 r = (float32)(rand() & (RAND_LIMIT));      
                r /= RAND_LIMIT;      
                r = 2.0f * r - 1.0f;      
                return r;      
        }      
              
        /// Random floating point number in range [lo, hi]      
        float32 b2Random(float32 lo, float32 hi)      
        {      
                float32 r = (float32)(rand() & (RAND_LIMIT));      
                r /= RAND_LIMIT;      
                r = (hi - lo) * r + lo;      
                return r;      
        }

    %}
    
    /* Additional supporting Python code */
    %pythoncode %{
    B2_FLT_EPSILON = 1.192092896e-07
    FLT_EPSILON = B2_FLT_EPSILON
    B2_FLT_MAX     = 3.402823466e+38
    cvars = ('b2_minPulleyLength','b2Contact_s_initialized','b2Contact_s_registers','b2_maxStackEntries','b2_stackSize',
             'b2_chunkArrayIncrement','b2_blockSizes','b2_maxBlockSize','b2_chunkSize','b2_defaultFilter','b2BroadPhase_s_validate',
             'b2_nullEdge','b2_invalid','b2_tableMask','b2_tableCapacity','b2_nullProxy','b2_nullPair','b2_nullFeature','b2XForm_identity',
             'b2Mat22_identity','b2Vec2_zero','b2_version','b2_byteCount','b2_angularSleepTolerance','b2_linearSleepTolerance',
             'b2_timeToSleep','b2_contactBaumgarte','b2_maxAngularVelocitySquared','b2_maxAngularVelocity','b2_maxLinearVelocitySquared',
             'b2_maxLinearVelocity','b2_maxAngularCorrection','b2_maxLinearCorrection','b2_velocityThreshold','b2_maxTOIJointsPerIsland',
             'b2_maxTOIContactsPerIsland','b2_toiSlop','b2_angularSlop','b2_linearSlop','b2_maxPairs','b2_maxProxies','b2_maxPolygonVertices',
             'b2_maxManifoldPoints','b2_pi')

    def b2PythonComputeCentroid(pd):
        """
            Computes the centroid of the polygon shape definition, pd.
            Raises ValueError on an invalid vertex count or a small area.

            Ported from the Box2D C++ code.
        """
        count = pd.vertexCount

        if count < 3:
            raise ValueError("ComputeCentroid: vertex count < 3")

        c = b2Vec2(0.0, 0.0)
        area = 0.0

        # pRef is the reference point for forming triangles.
        # It's location doesn't change the result (except for rounding error).
        pRef = b2Vec2(0.0, 0.0)

        inv3 = 1.0 / 3.0

        for i in range(count):
            # Triangle vertices.
            p1 = pRef
            p2 = pd.getVertex(i)
            if i + 1 < count: 
                p3 = pd.getVertex(i+1)
            else: p3 = pd.getVertex(0)

            e1 = p2 - p1
            e2 = p3 - p1

            D = b2Cross(e1, e2)

            triangleArea = 0.5 * D
            area += triangleArea

            # Area weighted centroid
            c += triangleArea * inv3 * (p1 + p2 + p3)

        # Centroid
        if area <= FLT_EPSILON:
            raise ValueError("ComputeCentroid: area <= FLT_EPSILON")

        return c / area
    %}

    /* Some final naming cleanups, for as of yet unused/unsupported classes */
    //b2PairManager
    %rename(broadPhase)      b2PairManager::m_broadPhase;
    %rename(callback)        b2PairManager::m_callback;
    %rename(pairs)           b2PairManager::m_pairs;
    %rename(freePair)        b2PairManager::m_freePair;
    %rename(pairCount)       b2PairManager::m_pairCount;
    %rename(pairBuffer)      b2PairManager::m_pairBuffer;
    %rename(pairBufferCount) b2PairManager::m_pairBufferCount;
    %rename(hashTable)       b2PairManager::m_hashTable;

    //b2BroadPhase
    %rename(pairManager)        b2BroadPhase::m_pairManager;
    %rename(proxyPool)          b2BroadPhase::m_proxyPool;
    %rename(freeProxy)          b2BroadPhase::m_freeProxy;
    %rename(bounds)             b2BroadPhase::m_bounds;
    %rename(queryResults)       b2BroadPhase::m_queryResults;
    %rename(querySortKeys)      b2BroadPhase::m_querySortKeys;
    %rename(queryResultCount)   b2BroadPhase::m_queryResultCount;
    %rename(worldAABB)          b2BroadPhase::m_worldAABB;
    %rename(quantizationFactor) b2BroadPhase::m_quantizationFactor;
    %rename(proxyCount)         b2BroadPhase::m_proxyCount;
    %rename(timeStamp)          b2BroadPhase::m_timeStamp;

    //b2Contact
    %rename(flags)             b2Contact::m_flags;
    %rename(manifoldCount)     b2Contact::m_manifoldCount;
    %rename(prev)              b2Contact::m_prev;
    %rename(next)              b2Contact::m_next;
    %rename(node1)             b2Contact::m_node1;
    %rename(node2)             b2Contact::m_node2;
    %rename(shape1)            b2Contact::m_shape1;
    %rename(shape2)            b2Contact::m_shape2;
    %rename(toi)               b2Contact::m_toi;

    //b2ContactManager
    %rename(world)             b2ContactManager::m_world;
    %rename(nullContact)       b2ContactManager::m_nullContact;
    %rename(destroyImmediate)  b2ContactManager::m_destroyImmediate;

    %include "Box2D/Box2D_deprecated.i"
#endif

%include "Box2D/Box2D.h"

