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
      ethernet.c \
      timer.c \
      webserver.c \
      nmea2000.c

OBJ = $(SRC:.c=.o)

all: app

app: $(OBJ)
	$(CC) $(CFLAGS) -o app $(OBJ) $(LDFLAGS)

clean:
	rm -f $(OBJ) app
