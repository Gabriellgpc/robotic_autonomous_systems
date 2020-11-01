COPSIM_BASEDIR = ${HOME}/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
BASEDIR = ../..

CXX = g++	-std=c++17
CXXFLAGS = -O2 -I ${COPSIM_BASEDIR}/programming/b0RemoteApiBindings/cpp/msgpack-c/include \
			   -I ${COPSIM_BASEDIR}/programming/bluezero/include/b0/bindings\
			   -I /usr/include/python3.8\
			   -I /usr/local/lib/python3.8/dist-packages/numpy/core/include\
			   -I ${BASEDIR}/include
LDFLAGS =	-L ${COPSIM_BASEDIR}/programming/bluezero/build\
		    -L ${BASEDIR}/lib

LDLIBS = -lpthread	-lb0	-fopenmp -lpython3.8	-lcontroller	-lutils	-lb0RemoteApi\
		 -lutil -lboost_iostreams -lboost_system -lboost_filesystem

PROGRAM = p2m1

all:	${PROGRAM}

p2m1.o:	p2m1.hpp	p2m1.cpp

%.o: %.cpp gnuplot-iostream.h
	@echo Compiling $@
	$(CXX) $(CXXFLAGS) -c $< -o $@

p2m1: p2m1.o
	@echo Linking $@
	$(CXX) -o $@ $^ $(LDFLAGS) ${LDLIBS}

# p2m1:	p2m1.o

clean:
	rm *.o
	rm ${PROGRAM}
