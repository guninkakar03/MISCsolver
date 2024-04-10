CXX = g++
CXXFLAGS = -g -std=c++11 -Wall -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3 -MMD -MP
CXXFLAGS += -O3 -flto
LDFLAGS = -flto
EXEC = main
SRCDIR = src
OBJDIR = obj

SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))
DEPS = $(OBJECTS:.o=.d)

$(EXEC): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(EXEC) $(OBJECTS) $(DEPS)

-include $(DEPS)

.PHONY: clean
