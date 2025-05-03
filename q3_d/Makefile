# Compiler and flags
CXX = g++
CXXFLAGS = --std=c++23 -Wall -Werror -pedantic

# Target executable
TARGET = sequencer

# Source files
SRCS = Sequencer.cpp
OBJS = $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

# Build the target
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Clean up
clean:
	rm -f $(TARGET) $(OBJS)
