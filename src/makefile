CC = g++
CXXFLAGS = -g -pg -std=c++11 -Wall -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3  
LDFLAGS = -pg
EXEC = main
SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

$(EXEC): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $(EXEC) $(OBJECTS) -lstdc++

%.o: %.cpp
	$(CC) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(EXEC) $(OBJECTS)

.PHONY: clean

