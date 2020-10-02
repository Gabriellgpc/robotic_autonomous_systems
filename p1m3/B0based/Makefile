COPSIM_BASEDIR = /home/lgabriel/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04

CXX = g++
CXXFLAGS = -O2 -I ${COPSIM_BASEDIR}/programming/b0RemoteApiBindings/cpp/msgpack-c/include \
					 -I ${COPSIM_BASEDIR}/programming/bluezero/include/b0/bindings
LDFLAGS =	-L ${COPSIM_BASEDIR}/programming/bluezero/build
LDLIBS = -lpthread	-lb0

.cpp:
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $< -o $@ b0RemoteApi.cpp $(LDLIBS)