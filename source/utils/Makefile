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

LDLIBS =	-fopenmp	-lpython3.8

OBJS = utils.o
LIBS = libutils.a

all:	${LIBS}

utils.o: utils.hpp utils.cpp

libutils.a:	${OBJS}
	ar r $@ $?

install:	${LIBS}
	cp -f *.a ${BASEDIR}/lib
	cp -f *.hpp ${BASEDIR}/include

clean:
	rm *.o *.a