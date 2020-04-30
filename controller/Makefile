CC=gcc
CFLAGS=-Wall -Werror
OBJ1=libhighlevel_controller.o
OBJ2=pfm_controller.o
OBJ3=dwa_controller.o

# It just compiles the first one

libhighlevel_controller.so: $(OBJ2)
	gcc -shared -o $@ $(OBJ2)
$(OBJ2): highlevel_controller.c controller.h
	$(CC) -c -o $(OBJ2) -fpic $< $(CFLAGS)

pfm_controller.so: $(OBJ1)
	gcc -shared -o $@ $(OBJ1)
$(OBJ1): pfm_controller.c controller.h
	$(CC) -c -o $(OBJ1) -fpic $< $(CFLAGS)

dwa_controller.so: $(OBJ3)
	gcc -shared -o $@ $(OBJ3)
$(OBJ3): dwa_controller.c controller.h
	$(CC) -c -o $(OBJ3) -fpic $< $(CFLAGS)

