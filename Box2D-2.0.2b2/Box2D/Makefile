TARGETS= Gen/float/libbox2d.a

CXXFLAGS+=	-g -O2

SOURCES = \
	./Dynamics/b2Body.cpp \
	./Dynamics/b2Island.cpp \
	./Dynamics/b2World.cpp \
	./Dynamics/b2ContactManager.cpp \
	./Dynamics/Contacts/b2Contact.cpp \
	./Dynamics/Contacts/b2PolyContact.cpp \
	./Dynamics/Contacts/b2CircleContact.cpp \
	./Dynamics/Contacts/b2PolyAndCircleContact.cpp \
	./Dynamics/Contacts/b2EdgeAndCircleContact.cpp \
	./Dynamics/Contacts/b2PolyAndEdgeContact.cpp \
	./Dynamics/Contacts/b2ContactSolver.cpp \
	./Dynamics/Controllers/b2BuoyancyController.cpp \
	./Dynamics/Controllers/b2ConstantAccelController.cpp \
	./Dynamics/Controllers/b2ConstantForceController.cpp \
	./Dynamics/Controllers/b2Controller.cpp \
	./Dynamics/Controllers/b2GravityController.cpp \
	./Dynamics/Controllers/b2TensorDampingController.cpp \
	./Dynamics/b2WorldCallbacks.cpp \
	./Dynamics/Joints/b2MouseJoint.cpp \
	./Dynamics/Joints/b2PulleyJoint.cpp \
	./Dynamics/Joints/b2Joint.cpp \
	./Dynamics/Joints/b2RevoluteJoint.cpp \
	./Dynamics/Joints/b2PrismaticJoint.cpp \
	./Dynamics/Joints/b2DistanceJoint.cpp \
	./Dynamics/Joints/b2GearJoint.cpp \
	./Dynamics/Joints/b2LineJoint.cpp \
	./Common/b2StackAllocator.cpp \
	./Common/b2Math.cpp \
	./Common/b2BlockAllocator.cpp \
	./Common/b2Settings.cpp \
	./Collision/b2Collision.cpp \
	./Collision/b2Distance.cpp \
	./Collision/Shapes/b2Shape.cpp \
	./Collision/Shapes/b2CircleShape.cpp \
	./Collision/Shapes/b2PolygonShape.cpp \
	./Collision/Shapes/b2EdgeShape.cpp \
	./Collision/b2TimeOfImpact.cpp \
	./Collision/b2PairManager.cpp \
	./Collision/b2CollidePoly.cpp \
	./Collision/b2CollideCircle.cpp \
	./Collision/b2BroadPhase.cpp 
#	./Contrib/b2Polygon.cpp \
#	./Contrib/b2Triangle.cpp


ifneq ($(INCLUDE_DEPENDENCIES),yes)

all:	
	@make --no-print-directory INCLUDE_DEPENDENCIES=yes $(TARGETS)

.PHONY:	clean
clean:
	rm -rf Gen

else

-include $(addprefix Gen/float/,$(SOURCES:.cpp=.d))
#-include $(addprefix Gen/fixed/,$(SOURCES:.cpp=.d))

endif

Gen/float/%.o:		%.cpp
	@mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -c -o $@ $<


Gen/float/libbox2d.a:	$(addprefix Gen/float/,$(SOURCES:.cpp=.o))
	ar cr $@ $^
	ranlib $@ 

Gen/float/%.d:		%.cpp
	@mkdir -p $(dir $@)
	c++ -MM -MT $(@:.d=.o) $(CXXFLAGS) -o $@ $<

