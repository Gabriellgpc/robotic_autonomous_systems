COPSIM_BASEDIR = /home/lgabriel/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04

CXX = g++	-std=c++11
CXXFLAGS = -O2 -I ${COPSIM_BASEDIR}/programming/b0RemoteApiBindings/cpp/msgpack-c/include \
			   -I ${COPSIM_BASEDIR}/programming/bluezero/include/b0/bindings\
			   -I /usr/include/python3.8\
			   -I /usr/local/lib/python3.8/dist-packages/numpy/core/include
LDFLAGS =	-L ${COPSIM_BASEDIR}/programming/bluezero/build

LDLIBS = -lpthread	-lb0	-fopenmp -lpython3.8

OBJS	=	b0RemoteApi.o	utils.o	pid.o	kinematicsControllers.o

PROGRAM = meta3	plotTest

all:	${PROGRAM}	${OBJS}

meta3:	meta3.cpp ${OBJS}


pid.o:	pid.hpp	pid.cpp

kinemacticsControllers.o:	kinematicsControllers.hpp kinematicsControllers.cpp	pid.o

b0RemoteApi.o:	b0RemoteApi.h b0RemoteApi.cpp

utils.o:	utils.hpp utils.cpp

clean:
	rm *.o
	rm ${PROGRAM}

# .cpp:
# 	$(CXX) $(CXXFLAGS) $(LDFLAGS) $< -o $@ b0RemoteApi.cpp $(LDLIBS)