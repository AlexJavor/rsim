CC=gcc
CFLAGS=-Wall -Werror

OBJ1=pfm_controller.o
OBJ2=dwa_controller.o
OBJ3=highlevel_controller.o

# It just compiles the first one
default: libpfm_controller.so libdwa_controller.so libhighlevel_controller.so 

libpfm_controller.so: $(OBJ1)
	gcc -shared -o $@ $(OBJ1)
$(OBJ1): pfm_controller.c controller.h
	$(CC) -c -o $(OBJ1) -fpic $< $(CFLAGS)

libdwa_controller.so: $(OBJ2)
	gcc -shared -o $@ $(OBJ2)
$(OBJ2): dwa_controller.c dwa_controller.h
	$(CC) -c -o $(OBJ2) -fpic $< $(CFLAGS)

libhighlevel_controller.so: $(OBJ3)
	gcc -shared -o $@ $(OBJ3)
$(OBJ3): highlevel_controller.c controller.h
	$(CC) -c -o $(OBJ3) -fpic $< $(CFLAGS)

