# Compiler and flags
CXX     = g++
CC      = gcc

CXXFLAGS  = --std=c++23 -Wall -Werror -pedantic -pthread
CFLAGS    = -Wall -Werror -pedantic

# Target executable
TARGET = sequencer

# Sources
CPP_SRCS = Sequencer.cpp
C_SRCS   = tasks.c

# Objects
CPP_OBJS = $(CPP_SRCS:.cpp=.o)
C_OBJS   = $(C_SRCS:.c=.o)
OBJS     = $(CPP_OBJS) $(C_OBJS)

all: $(TARGET)

# link everything together
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lpigpio

# compile C++ sources
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# compile C sources
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)
