# 2026-09-20 KS renamed serial.* fastnet.*
#      10-21 KS added GIT_VERSION stuff
#      11-01 KS renamed application binary 
#
CC      = gcc
GIT_VERSION = \"$(shell git describe --abbrev=4 --dirty --always --tags)\"

# compiler flags (debug version)
CFLAGS  = -Wall -Wextra -std=gnu99 -pthread -D GIT_VER=$(GIT_VERSION) -g -O2
# compiler flags (production version)
#CFLAGS  = -Wall -Wextra -std=gnu99 -pthread -D GIT_VER=$(GIT_VERSION)
#LDFLAGS = -pthread -lcjson -lcrypto
LDFLAGS = -pthread -lcjson -lcrypto

SRC = main.c \
      config.c \
      logger.c \
      app_state.c \
      fastnet.c \
      nmea0183.c \
      bgnet.c \
      ethernet.c \
      timer.c \
      webserver.c \
      nmea2000.c

H = config.h \
	logger.h \
	app_state.h \
	fastnet.h \
	nmea0183.h \
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
