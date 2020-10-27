COPSIM_BASEDIR = ${HOME}/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
BASEDIR = ../..

CXX = g++	-std=c++11
CXXFLAGS = -O2 -I ${COPSIM_BASEDIR}/programming/b0RemoteApiBindings/cpp/msgpack-c/include \
			   -I ${COPSIM_BASEDIR}/programming/bluezero/include/b0/bindings\
			   -I /usr/include/python3.8\
			   -I /usr/local/lib/python3.8/dist-packages/numpy/core/include\
			   -I ${BASEDIR}/include
LDFLAGS =	-L ${COPSIM_BASEDIR}/programming/bluezero/build \
			-L ${BASEDIR}/lib

LDLIBS = -lb0

OBJS = b0RemoteApi.o
LIBS = libb0RemoteApi.a

all:	${LIBS}

b0RemoteApi.o:	b0RemoteApi.h b0RemoteApi.cpp

libb0RemoteApi.a:	b0RemoteApi.o	-lb0
	ar r $@ $?

install:	${LIBS}
	cp -f *.a ${BASEDIR}/lib
	cp -f *.h ${BASEDIR}/include

clean:
	rm *.o *.a