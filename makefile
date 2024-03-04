CXX = g++
CXXFLAGS = -std=c++11 -Wall -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3
LDFLAGS = 
SRC = $(wildcard *.cpp)
OBJ = $(SRC:.cpp=.o)
EXEC = solver

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(EXEC)
