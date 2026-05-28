CC      = gcc
#CFLAGS  = -Wall -Wextra -std=c11 -pthread
CFLAGS  = -Wall -Wextra -std=gnu99 -pthread 
#LDFLAGS = -pthread -lcjson
LDFLAGS = -pthread -lcjson -lcrypto

SRC = main.c \
      config.c \
      logger.c \
      app_state.c \
      serial.c \
	  bgnprotocol.c \
      ethernet.c \
      timer.c \
      webserver.c \
      nmea2000.c

H = config.h \
	logger.h \
	app_state.h \
	serial.h \
	bgnprotocol.h \
	ethernet.h \
	timer.h \
	webserver.h \
	nmea2000.h

OBJ = $(SRC:.c=.o)

all: app

app: $(OBJ) $(H)
	$(CC) $(CFLAGS) -o app $(OBJ) $(LDFLAGS)

clean:
	rm -f $(OBJ) app
