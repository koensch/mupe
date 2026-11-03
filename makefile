# 2026-09-20 KS renamed serial.* fastnet.*
#      10-21 KS added GIT_VERSION stuff
#      11-01 KS renamed application binary 
#
CC      = gcc
GIT_VERSION = \"$(shell git describe --abbrev=4 --dirty --always --tags)\"

#CFLAGS  = -Wall -Wextra -std=c11 -pthread
CFLAGS  = -Wall -Wextra -std=gnu99 -pthread -D GIT_VER=$(GIT_VERSION)
#LDFLAGS = -pthread -lcjson
LDFLAGS = -pthread -lcjson -lcrypto

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

all: mupe

mupe: $(OBJ) $(H)
	$(CC) $(CFLAGS) -o mupe $(OBJ) $(LDFLAGS)

clean:
	rm -f $(OBJ) app
