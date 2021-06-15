#PROG_FILE=jh7100-recover.c
PROG_NAME=jh7100-recover

CFLAGS := -Wall
EXEF := $(PROG_NAME)

.PHONY : all clean
all : $(EXEF)

clean:
	$(RM) *.o *.d $(EXEF)
