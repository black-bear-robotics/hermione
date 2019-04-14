INC = -I/usr/local/include
LIB = ./libmaestro.a -lmynteye
CFLAGS = -Wall -std=c++11 

all: main

main: main.cpp main.h
	g++ $(CFLAGS) $(INC) main.cpp -o main_exec  $(LIB) `pkg-config --cflags --libs opencv`

clean:
	rm *.o

