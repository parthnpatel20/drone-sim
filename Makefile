CC = gcc
CFLAGS = -Wall -Wextra -O2 -pthread -lrt
TARGET = sensor

all: $(TARGET)

$(TARGET): main.c
	$(CC) $(CFLAGS) -o $(TARGET) main.c

clean:
	rm -f $(TARGET)
