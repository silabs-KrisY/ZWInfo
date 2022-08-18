TARGET ?= ZWUtil
CC = gcc
LD = ld
C_SRC = ZWUtil.c
CFLAGS = -g -Wall -Wextra
EXEDIR = exe

$(EXEDIR)/$(TARGET): $(C_SRC)
	mkdir -p $(EXEDIR)
	$(CC) $(DEBUG) -o $@ $^ $(CFLAGS)

debug: DEBUG = -DDEBUG

debug: $(EXEDIR)/$(TARGET)

clean:
	rm -f $(EXEDIR)/$(TARGET)
