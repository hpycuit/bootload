EXE = bootloader
CC = gcc
OBJS = uploader_main.o uploader.o crc32.o

all:$(EXE)

$(EXE):$(OBJS)
	$(CC) $(CFLAGS) -o $(EXE) $(OBJS)

#uploader.o:crc32.h upload.h

uploader_main.o:crc32.h upload.h

clean:
	rm $(EXE) $(OBJS)
