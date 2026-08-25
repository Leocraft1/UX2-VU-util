CC = gcc
CFLAGS = -Wall -Wextra -O2 $(shell pkg-config --cflags libpipewire-0.3)
LDLIBS = $(shell pkg-config --libs libpipewire-0.3) -lusb-1.0 -lpthread -lm

SRC = main.c
BIN = ux2_vu_daemon

$(BIN): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f $(BIN)

.PHONY: clean