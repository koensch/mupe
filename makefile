# 2026-09-20 KS renamed serial.* fastnet.*
CC      = gcc
#CFLAGS  = -Wall -Wextra -std=c11 -pthread
CFLAGS  = -Wall -Wextra -std=gnu99 -pthread 
#LDFLAGS = -pthread -lcjson
LDFLAGS = -pthread -lcjson -lcrypto
GIT_VERSION := "$(shell git describe --abbrev=4 --dirty --always --tags)"

SRC = main.c \
      config.c \
      logger.c \
      app_state.c \
      fastnet.c \
      bgnet.c \
      ethernet.c \
      timer.c \
      webserver.c \
      nmea2000.c

H = config.h \
	logger.h \
	app_state.h \
	fastnet.h \
	bgnet.h \
	ethernet.h \
	timer.h \
	webserver.h \
	nmea2000.h

OBJ = $(SRC:.c=.o)

all: app

app: $(OBJ) $(H)
	$(CC) $(CFLAGS) -DGIT_VER=\"$(GIT_VERSION)\" -o app $(OBJ) $(LDFLAGS)

clean:
	rm -f $(OBJ) app
