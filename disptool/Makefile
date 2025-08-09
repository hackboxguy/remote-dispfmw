CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -I../common
LDFLAGS = -li2c

TARGET = disptool
SOURCES = disptool.c

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: clean
