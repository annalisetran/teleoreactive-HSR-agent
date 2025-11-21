# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -pedantic -std=c11
CPPFLAGS += -I/usr/include/postgresql

# Libraries
LIBS = -lpq

# Source files
SRCS = wm_percepts.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable name
EXEC = wm_percepts

# Targets
all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(EXEC)
