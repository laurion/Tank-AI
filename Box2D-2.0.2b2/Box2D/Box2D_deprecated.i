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

%feature("docstring") collideCircleParticle "For liquid simulation. Checks if a particle
would collide with the specified circle.
";

%feature("docstring") b2CollidePolyParticle "For liquid simulation. Checks if a particle
would collide with the specified polygon.
";

%inline %{
    PyObject* collideCircleParticle(b2CircleShape* circle, const b2Vec2& ppos) {
        //out bCollides, b2Vec2 penetration, b2Vec2 penetrationNormal
        //Ported to C from Blaze (D)
        PyObject* ret=PyTuple_New(3);
        PyTuple_SetItem(ret, 0, SWIG_From_bool(false));
        PyTuple_SetItem(ret, 1, SWIG_From_bool(false));
        PyTuple_SetItem(ret, 2, SWIG_From_bool(false));

        b2XForm xf1 = circle->GetBody()->GetXForm();

        b2Vec2 p1 = b2Mul(xf1, circle->GetLocalPosition());
        b2Vec2 p2 = ppos;

        b2Vec2 d = p2 - p1;
        float32 distSqr = b2Dot(d, d);
        float32 r1 = circle->GetRadius();
        float32 r2 = 0.0f;
        float32 radiusSum = r1 + r2;
        if (distSqr > radiusSum * radiusSum) {
            return ret; // false
        }

        b2Vec2* normal=new b2Vec2();
        float32 separation;
        if (distSqr < B2_FLT_EPSILON) {
            separation = -radiusSum;
            normal->Set(0.0f, 1.0f);
        } else {
            float32 dist = sqrt(distSqr);
            separation = dist - radiusSum;
            float32 a = 1.0f / dist;
            normal->x = a * d.x;
            normal->y = a * d.y;
        }

        b2Vec2* penetration=new b2Vec2();
        penetration->x = normal->x * separation;
        penetration->y = normal->y * separation;
        PyTuple_SetItem(ret, 0, SWIG_From_bool(true));
        PyTuple_SetItem(ret, 1, SWIG_NewPointerObj(SWIG_as_voidptr(penetration), SWIGTYPE_p_b2Vec2, 0) );
        PyTuple_SetItem(ret, 2, SWIG_NewPointerObj(SWIG_as_voidptr(normal), SWIGTYPE_p_b2Vec2, 0) );
        return ret;
    }

    PyObject* b2CollidePolyParticle(b2PolygonShape* polygon, const b2Vec2& ppos, float32 pradius) {
        //out bCollides, b2Vec2 penetration, b2Vec2 penetrationNormal
        //Ported to C from Blaze (D)
        PyObject* ret=PyTuple_New(3);
        PyTuple_SetItem(ret, 0, SWIG_From_bool(false));
        PyTuple_SetItem(ret, 1, SWIG_From_bool(false));
        PyTuple_SetItem(ret, 2, SWIG_From_bool(false));

        const b2XForm xf1 = polygon->GetBody()->GetXForm();
        b2XForm xf2;
        xf2.position = ppos;

        // Compute circle position in the frame of the polygon.
        b2Vec2 c = b2Mul(xf2, b2Vec2_zero);
        b2Vec2 cLocal = b2MulT(xf1, c);

        // Find the min separating edge.
        int normalIndex = 0;
        float32 separation = -B2_FLT_MAX;
        float32 radius = pradius;
        b2Vec2* penetration=new b2Vec2();

        int vertexCount = polygon->GetVertexCount();
        const b2Vec2* vertices = polygon->GetVertices();
        const b2Vec2* normals = polygon->GetNormals();

        for (int i = 0; i < vertexCount; ++i) {
            float32 s = b2Dot(normals[i], cLocal - vertices[i]);

            if (s > radius) {
                // Early out.
                return ret; // false
            }

            if (s > separation) {
                separation = s;
                normalIndex = i;
            }
        }

        // If the center is inside the polygon ...
        if (separation < B2_FLT_MAX) {
            b2Vec2 temp = b2Mul(xf1.R, normals[normalIndex]);
            b2Vec2* penetrationNormal=new b2Vec2(temp);
            separation = separation - radius;
            penetration->x = separation * penetrationNormal->x;
            penetration->y = separation * penetrationNormal->y;
            PyTuple_SetItem(ret, 0, SWIG_From_bool(true));
            PyTuple_SetItem(ret, 1, SWIG_NewPointerObj(SWIG_as_voidptr(penetration), SWIGTYPE_p_b2Vec2, 0) );
            PyTuple_SetItem(ret, 2, SWIG_NewPointerObj(SWIG_as_voidptr(penetrationNormal), SWIGTYPE_p_b2Vec2, 0) );
            return ret;
        }

        // Project the circle center onto the edge segment.
        int vertIndex1 = normalIndex;
        int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
        b2Vec2 e = vertices[vertIndex2] - vertices[vertIndex1];

        float32 length = e.Normalize();
        //assert(length > float.epsilon);

        // Project the center onto the edge.
        float32 u = b2Dot(cLocal - vertices[vertIndex1], e);
        b2Vec2 p;
        if (u <= 0.0f) {
            p = vertices[vertIndex1];
        } else if (u >= length) {
            p = vertices[vertIndex2];
        } else {
            p = vertices[vertIndex1] + u * e;
        }

        b2Vec2 d = cLocal - p;
        float32 dist = d.Normalize();
        if (dist > radius) {
            return ret; //false
        }

        b2Vec2 temp = b2Mul(xf1.R, d);
        b2Vec2* penetrationNormal=new b2Vec2(temp);
        separation = dist - radius;
        penetration->x = separation * penetrationNormal->x;
        penetration->y = separation * penetrationNormal->y;
        PyTuple_SetItem(ret, 0, SWIG_From_bool(true));
        PyTuple_SetItem(ret, 1, SWIG_NewPointerObj(SWIG_as_voidptr(penetration), SWIGTYPE_p_b2Vec2, 0) );
        PyTuple_SetItem(ret, 2, SWIG_NewPointerObj(SWIG_as_voidptr(penetrationNormal), SWIGTYPE_p_b2Vec2, 0) );
        return ret;
    }

%}

%pythoncode %{

    def b2PythonCheckPolygonDef(pd):
        """
            Checks the Polygon definition to see if upon creation it will cause an assertion.
            Raises ValueError if an assertion would be raised.

            Ported from the Box2D C++ code for CreateShape(). The C++ version is now
            included as it's more accurate, please use b2CheckPolygonDef instead.
        """

        if pd.vertexCount < 3 or pd.vertexCount >= b2_maxPolygonVertices:
            raise ValueError("Invalid vertexCount")

        threshold = FLT_EPSILON * FLT_EPSILON
        verts = pd.getVertices_b2Vec2()
        normals = []
        v0 = verts[0]
        for i in range(pd.vertexCount):
            if i == pd.vertexCount-1:
                v1 = verts[0]
            else: v1 = verts[i+1]
            edge=v1 - v0
            if edge.LengthSquared() < threshold:
                raise ValueError("edge.LengthSquared < FLT_EPSILON**2" )
            normals.append( b2Cross(edge, 1.0) )
            normals[-1].Normalize()
            v0=v1

        centroid = b2PythonComputeCentroid(pd)

        d=b2Vec2()
        for i in range(pd.vertexCount):
            i1 = i - 1 
            if i1 < 0: i1 = pd.vertexCount - 1
            i2 = i
            n1 = normals[i1]
            n2 = normals[i2]
            v = verts[i] - centroid

            d.x = b2Dot(n1, v) - b2_toiSlop
            d.y = b2Dot(n2, v) - b2_toiSlop

            # Shifting the edge inward by b2_toiSlop should
            # not cause the plane to pass the centroid.

            # Your shape has a radius/extent less than b2_toiSlop.
            if d.x < 0.0 or d.y <= 0.0: 
                raise ValueError("Your shape has a radius/extent less than b2_toiSlop.")

            A = b2Mat22()
            A.col1.x = n1.x; A.col2.x = n1.y
            A.col1.y = n2.x; A.col2.y = n2.y
            #coreVertices[i] = A.Solve(d) + m_centroid

        return True

%}
