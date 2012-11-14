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

/* Note: Given that we have a definition (b2BodyDef) that takes the userData, then
   passes it onto the factory output (b2Body) upon creation, it's necessary 
   to intercept the CreateBody/Joint/Shape functions to increase the refcount
   for each of those functions.

   And upon being destroyed, the userData refcount must be decreased.
 */

%extend b2World {
public:        
    b2Body* CreateBody(b2BodyDef* defn) {
        b2Body* ret;
        if (defn)
            Py_XINCREF((PyObject*)defn->userData);
        ret=self->CreateBody(defn);
        return ret;
    }
    b2Joint* CreateJoint(b2JointDef* defn) {
        b2Joint* ret;
        if (defn)
            Py_XINCREF((PyObject*)defn->userData);
        ret=self->CreateJoint(defn);
        return ret;
    }
    void DestroyBody(b2Body* body) {
        Py_XDECREF((PyObject*)body->GetUserData());
        self->DestroyBody(body);
    }
    void DestroyJoint(b2Joint* joint) {
        Py_XDECREF((PyObject*)joint->GetUserData());
        self->DestroyJoint(joint);
    }

}

%extend b2Body {
public:        
    void DestroyShape(b2Shape* shape) {
        Py_XDECREF((PyObject*)shape->GetUserData());
        self->DestroyShape(shape);
    }
    b2Shape* CreateShape(b2ShapeDef* defn) {
        b2Shape* ret;
        if (defn)
            Py_XINCREF((PyObject*)defn->userData);
        ret=self->CreateShape(defn);
        return ret;
    }
    PyObject* GetUserData() {
        PyObject* ret=(PyObject*)self->GetUserData();
        if (!ret) ret=Py_None;
        Py_XINCREF(ret);
        return ret;
    }
    void SetUserData(PyObject* data) {
        Py_XDECREF((PyObject*)self->GetUserData());
        Py_INCREF(data);
        self->SetUserData(data);
    }
    void ClearUserData() {
        Py_XDECREF((PyObject*)self->GetUserData());
        self->SetUserData(NULL);
    }
    %pythoncode %{
        userData = property(GetUserData, SetUserData)
    %}
}

%extend b2Joint {
public:        
    PyObject* GetUserData() {
        PyObject* ret=(PyObject*)self->GetUserData();
        if (!ret) ret=Py_None;
        Py_XINCREF(ret);
        return ret;
    }
    void SetUserData(PyObject* data) {
        Py_XDECREF((PyObject*)self->GetUserData());
        Py_INCREF(data);
        self->SetUserData(data);
    }
    void ClearUserData() {
        Py_XDECREF((PyObject*)self->GetUserData());
        self->SetUserData(NULL);
    }
    %pythoncode %{
        userData = property(GetUserData, SetUserData)
    %}
}

%extend b2Shape {
public:        
    PyObject* GetUserData() {
        PyObject* ret=(PyObject*)self->GetUserData();
        if (!ret) ret=Py_None;
        Py_XINCREF(ret);
        return ret;
    }
    void SetUserData(PyObject* data) {
        Py_XDECREF((PyObject*)self->GetUserData());
        Py_INCREF(data);
        self->SetUserData(data);
    }
    void ClearUserData() {
        Py_XDECREF((PyObject*)self->GetUserData());
        self->SetUserData(NULL);
    }
    %pythoncode %{
        userData = property(GetUserData, SetUserData)
    %}
}

//Allow access to userData in definitions, with proper destruction
%extend b2JointDef {
public:
    PyObject* GetUserData() {
        PyObject* ret;
        if (!self->userData)
            ret=Py_None;
        else
            ret=(PyObject*)self->userData;
        Py_INCREF((PyObject*)ret);
        return ret;
    }
    void SetUserData(PyObject* data) {
        Py_XDECREF((PyObject*)self->userData);
        Py_INCREF(data);
        self->userData=(void*)data;
    }
    void ClearUserData() {
        Py_XDECREF((PyObject*)self->userData);
        self->userData=NULL;
    }
    %pythoncode %{
        userData = property(GetUserData, SetUserData)
        def __del__(self):
            self.ClearUserData()
    %}
}

%extend b2BodyDef {
public:
    PyObject* GetUserData() {
        PyObject* ret;
        if (!self->userData)
            ret=Py_None;
        else
            ret=(PyObject*)self->userData;
        Py_INCREF((PyObject*)ret);
        return ret;
    }
    void SetUserData(PyObject* data) {
        Py_XDECREF((PyObject*)self->userData);
        Py_INCREF(data);
        self->userData=(void*)data;
    }
    void ClearUserData() {
        Py_XDECREF((PyObject*)self->userData);
        self->userData=NULL;
    }
    %pythoncode %{
        userData = property(GetUserData, SetUserData)
        def __del__(self):
            self.ClearUserData()
    %}
}

%extend b2ShapeDef {
public:
    PyObject* GetUserData() {
        PyObject* ret;
        if (!self->userData)
            ret=Py_None;
        else
            ret=(PyObject*)self->userData;
        Py_INCREF((PyObject*)ret);
        return ret;
    }
    void SetUserData(PyObject* data) {
        Py_XDECREF((PyObject*)self->userData);
        Py_INCREF(data);
        self->userData=(void*)data;
    }
    void ClearUserData() {
        Py_XDECREF((PyObject*)self->userData);
        self->userData=NULL;
    }
    %pythoncode %{
        userData = property(GetUserData, SetUserData)
        def __del__(self):
            self.ClearUserData()
    %}
}

// These renames are intentionally below the above CreateBody, as they will rename the 
// original C++ versions and not the ones I have written.
%ignore SetUserData;
%ignore GetUserData;
%ignore userData;

%newobject b2World::CreateBody;
%newobject b2World::CreateJoint;
%newobject b2Body::CreateShape;

%ignore b2World::CreateBody;
%ignore b2World::CreateJoint;
%ignore b2Body::CreateShape;

%ignore b2World::DestroyBody;
%ignore b2World::DestroyJoint;
%ignore b2Body::DestroyShape;
