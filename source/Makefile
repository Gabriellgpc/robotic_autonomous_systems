COPSIM_BASEDIR = ${HOME}/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
BASEDIR = ..

all:	install

install:
	cd b0RemoteApi && ${MAKE} install
	cd utils && ${MAKE} install
	cd controllers && ${MAKE} install
	cp matplotlib-cpp/matplotlibcpp.h ${BASEDIR}/include 
	cp gnuplot-iostream/gnuplot-iostream.h ${BASEDIR}/include 

clean:
	cd b0RemoteApi && ${MAKE} clean
	cd utils && ${MAKE} clean
	cd controllers && ${MAKE} clean