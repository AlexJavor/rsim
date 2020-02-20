CC=gcc
CFLAGS=-Wall -Werror
OBJ=libdummy_controller.o

libdummy_controller.so: $(OBJ)
	gcc -shared -o $@ $(OBJ)

$(OBJ): dummy_controller.c controller.h
	$(CC) -c -o $(OBJ) -fpic $< $(CFLAGS)
