all: read_all compass

read_all: read_all.cpp
	$(CXX) read_all.cpp -o read_all -Wall

compass: compass.cpp compass.png
	$(CXX) compass.cpp -o compass $(PROGNAME) -Wall $(shell pkg-config --cflags sdl2) $(shell pkg-config --libs sdl2) -lSDL2_image

clean:
	rm -f compass read_all *.o

