all: cdlib 

%.o: %.c
	$(CC) -c $(CFLAGS) $<

cdlib: loadstl.o triangle_cd.o volume.o

loadstl.o: loadstl.c loadstl.h geom.h cd.h
test2.o: test2.c loadstl.h geom.h cd.h triangle_cd.h volume.h
triangle_cd.o: triangle_cd.c triangle_cd.h geom.h cd.h cd_macro.h
volume.o: volume.c volume.h cd.h triangle_cd.h geom.h loadstl.h

clean:
	-rm *.o
