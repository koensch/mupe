- use a linux machine as your development platform
- install C developemnt tools (if not already present on your linux machine)
- install Visual Studio Code as development tool
- install github on your development machine
- use your account on github to access the MarineGateway project on github (not yet publicly available)
- check that your github is authorized on the computer you are using:   ssh -T git@github.com
- select the directory where you want the MarineGateway to be:          cd ~/workspace/
- install MarineGateway project (clone from github):                    git clone ......
- observe that the directory ~/workspace/MarineGateway exist            ls -l
- install cJSON project :                                               sudo apt-get -y install libcjson-dev
- install SSL development tools:                                        sudo apt-get -y install libssl-dev
- compile MarineGateway project:                                        make -B
- observe executable 'app' to be created:                               ls -l
- observe the following directory structure exists
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
- check that the config.json uses the correct serial port, select "readonly" mode 
- start application							./app
- to observer output trace:						tail -f marine_gateway.log
- check status of your code after making changes			git status
- add changes to your staging						git add *.h *.c
- commit changes to your repository					git commit -m {description}


