#include <signal.h>
#include <pigpio.h>
#include "ArduinoPiGpio.h"

volatile bool isSignalActionAsserted = false;

void signalActionHandler(int signal);

int main() {
	if(gpioInitialise() < 0) {
		return -1;
	}
	if(gpioSetSignalFunc(SIGINT, signalActionHandler) < 0) {
		return -1;
	}
	setup();
	while(true) {
		loop();
		if(isSignalActionAsserted) {
			break;
		}
	}
	gpioTerminate();
	return 0;
}

void signalActionHandler(int signal) {
        isSignalActionAsserted = true;
}
