#INC = -I. -I./Maestro-lib/include
LIB = ./libmaestro.a ./libcamodocal.a -L. -lmynteye
CFLAGS = -Wall -std=c++11

all: main

main: main.cpp main.h
	g++ $(CFLAGS) main.cpp -o main_exec $(LIB)

clean:
	rm *.o

