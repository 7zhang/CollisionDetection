test.out: test.o loadstl.o trianglecollisiondetection.o 
	g++ -pg -O3 -o test.out test.o loadstl.o trianglecollisiondetection.o 
all: clean test.out
test.o: test.cpp loadstl.h common.h
	g++ -pg -O3 -c test.cpp
loadstl.o: loadstl.cpp loadstl.h common.h
	g++ -pg -O3 -c loadstl.cpp
trianglecollisiondetection.o: trianglecollisiondetection.cpp trianglecollisiondetection.h common.h 
	g++ -pg -O3 -c trianglecollisiondetection.cpp
clean:
	-rm test.out test.o loadstl.o trianglecollisiondetection.o 
