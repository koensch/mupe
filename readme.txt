- install C developemnt tools (if not already present on your linux machine)
- use your account on github
- check that your github is authorized on the computer you are using:   ssh -T git@github.com
- select the directory where you want the MarineGateway to be:          cd ~/workspace/ 
- install MarineGateway project (clone from github):                    git clone ......
- observe that the directory ~/workspace/MarineGateway exist            ls -l
- install cJSON project :                                               sudo apt-get -y install libcjson-dev
- install SSL development tools:                                        sudo apt-get install libssl-dev
- compile MarineGateway project:                                        make -B
- observe executable 'app' to be created:                               ls -l

project/
├── Makefile
├── config.json
├── main.c
├── config.h
├── config.c
├── logger.h
├── logger.c
├── app_state.h
├── app_state.c
├── serial.h
├── serial.c
├── ethernet.h
├── ethernet.c
├── timer.h
├── timer.c
├── webserver.h
├── webserver.c
├── nmea2000.h
└── nmea2000.c


3. ethernet.c — real TCP/UDP client
If protocol == "tcp": connect to remote_ip:remote_port, then:

periodically send "Ping\n"

read any incoming data and store it in shared state

If protocol == "udp": send UDP datagrams to remote_ip:remote_port, receive from same socket

777777777777777777777777777777

On a Linux system, what I gave you already lines up with your stack:

Threads: pthreads (pthread_create, pthread_join)

Serial: POSIX termios on /dev/tty*

Networking: POSIX sockets (socket, connect, send, recv, select, etc.)

JSON: cJSON linked via -lcjson

Build: Makefile using gcc

To get it running on Linux:

install json on a Linux system:

sudo apt-get install libcjson-dev      # Debian/Ubuntu
# or
sudo dnf install cjson-devel          # Fedora


set user rights to your serual port: sudo usermod -aG dialout "$USER"

$$$$$$$$$$$$$$$$ signal K paths:

Data Type	Signal K Path
GPS Position	navigation.position.latitude / navigation.position.longitude
Course Over Ground	navigation.courseOverGroundTrue
Boat Speed (Speed Through Water)	navigation.speedThroughWater
Depth (Below Transducer)	environment.depth.belowTransducer
True Wind Angle	environment.wind.angleTrueWater
True Wind Speed	environment.wind.speedTrue
Apparent Wind Angle	environment.wind.angleApparent
Apparent Wind Speed	environment.wind.speedApparent

1) Use Git (recommended)
Initialize a repo:

bash
git init
git add .
git commit -m "Initial working version"
Then whenever you break something:

bash
git restore <filename>
Or even:

bash
git reset --hard
Git is your safety net.

2) Keep a “golden copy” folder
Just duplicate the working project:

Code
project/
project_backup/
If something breaks, copy the file back.

3) Ask me for the file again
You can always say:

“Give me ethernet.c again”

And I’ll regenerate the exact version you need.

💬 And honestly?
You’ve built something impressive — a full Signal K + NMEA2000 + REST + WebSocket gateway in C.
That’s not trivial work. You’re doing great.

If you want, I can help you:

Add PGN 129029 (GNSS position with accuracy)

Add PGN 127250 (heading)

Add PGN 127245 (rudder angle)

Add PGN 127488 (engine parameters)

Add Signal K subscriptions (receive deltas)

Add TLS to the webserver

Add persistent storage (SQLite)

Just tell me what direction you want to go next.
