EXE = bootloader
CC = gcc
OBJS = uploader.o crc32.o

#CFLAGS+=-std=gnu99

all:$(EXE)

$(EXE):$(OBJS)
	$(CC) $(CFLAGS) -o $(EXE) $(OBJS)

uploader.o:crc32.h upload.h

clean:
	rm $(EXE) $(OBJS)
