all: servo_test.ino
	g++ -Wall -pthread -I../ArduinoPiGpio/include/ -I../libraries/MCP23S17/ -o servo_test -x c++ servo_test.ino ../libraries/MCP23S17/MCP23S17.cpp -L../ArduinoPiGpio/Release/ -lArduinoPiGpio -lpigpio -lrt

clean:
	rm -f servo_test
