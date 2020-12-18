COPSIM_BASEDIR = ${HOME}/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
BASEDIR = ../..

CXX = g++	-std=c++11
CXXFLAGS = -O2	-I ${BASEDIR}/include
LDFLAGS =	-L ${BASEDIR}/lib

LDLIBS =	-fopenmp	-lutils	-lmapping

OBJS = planning.o
LIBS = libplanning.a

all:	${LIBS}

manhattan.o: planning.hpp planning.cpp

libplanning.a:	${OBJS}
	ar r $@ $?

install:	${LIBS}
	cp -f *.a ${BASEDIR}/lib
	cp -f *.hpp ${BASEDIR}/include

clean:
	rm *.o *.a