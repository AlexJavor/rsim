CC=gcc
CFLAGS=-Wall -Werror
OBJ=libhighlevel_controller.o

libhighlevel_controller.so: $(OBJ)
	gcc -shared -o $@ $(OBJ)

$(OBJ): highlevel_controller.c controller.h
	$(CC) -c -o $(OBJ) -fpic $< $(CFLAGS)
