# Makefile for drone-sim

# Compiler and flags
CC       := gcc
CFLAGS   := -Wall -Wextra -O2 -std=gnu11 -pthread -lrt

# Libraries
LIBS     := -lpigpio

# Target
TARGET   := drone_sim
SRCS     := main.c
OBJS     := $(SRCS:.c=.o)

.PHONY: all clean

all: $(TARGET)

# Link
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

# Compile
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Remove binaries and objects
clean:
	rm -f $(TARGET) $(OBJS)

