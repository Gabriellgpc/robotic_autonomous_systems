COPSIM_BASEDIR = /home/lgabriel/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04

CXX = g++
CXXFLAGS = -O2 -I ${COPSIM_BASEDIR}/programming/b0RemoteApiBindings/cpp/msgpack-c/include \
					 -I ${COPSIM_BASEDIR}/programming/bluezero/include/b0/bindings
LDFLAGS =	-L ${COPSIM_BASEDIR}/programming/bluezero/build
LDLIBS = -lpthread	-lb0

PROGRAM = meta3
OBJS	=	b0RemoteApi.o	utils.o	pid.o

all:	${PROGRAM}	${OBJS}

meta3:	meta3.cpp b0RemoteApi.o utils.o

pid.o:	pid.hpp	pid.cpp

b0RemoteApi.o:	b0RemoteApi.h b0RemoteApi.cpp

utils.o:	utils.hpp utils.cpp

clean:
	rm *.o
	rm ${PROGRAM}

# .cpp:
# 	$(CXX) $(CXXFLAGS) $(LDFLAGS) $< -o $@ b0RemoteApi.cpp $(LDLIBS)