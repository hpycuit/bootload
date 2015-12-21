EXE = bootloader
CC = gcc
OBJS = main.o crc32.o

#CFLAGS+=-std=gnu99

all:$(EXE)

$(EXE):$(OBJS)
	$(CC) $(CFLAGS) -o $(EXE) $(OBJS)

main.o:crc32.h

clean:
	rm $(EXE) $(OBJS)
