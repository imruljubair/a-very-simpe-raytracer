CC=g++

CFLAGS=-c -std=c++0x -O3 -Wall
SOURCES=main.cpp Vec3f.cpp Camera.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=render

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm *.o $(EXECUTABLE)
