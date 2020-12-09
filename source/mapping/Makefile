COPSIM_BASEDIR = ${HOME}/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
BASEDIR = ../..

CXX = g++	-std=c++11
CXXFLAGS = -O2 -I ${BASEDIR}/include
LDFLAGS =	

LDLIBS =	-fopenmp

OBJS = occupating_grid.o
LIBS = libmapping.a

all:	${LIBS}

occupating_grid.o:	occupating_grid.hpp occupating_grid.cpp

libmapping.a:	${OBJS}
	ar r $@ $?

install:	${LIBS}
	cp -f *.a ${BASEDIR}/lib
	cp -f *.hpp ${BASEDIR}/include

clean:
	rm *.o *.a