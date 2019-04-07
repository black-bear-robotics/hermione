INC = -I ./Maestro/include/
CC=g++
CFLAGS= -c -Wall

all: main.cpp executable

executable: main.o
	$(CC) main.o -o test

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

clean:
	rm *.o

