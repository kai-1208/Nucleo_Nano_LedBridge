#include "NanoLedController.h"
#include "mbed.h"

static BufferedSerial nano_serial(PA_9, PA_10); 

void setupNanoLedController() {
    nano_serial.set_baud(9600);
}

void sendLedState(LedState state) {
    char c = static_cast<char>(state);
    nano_serial.write(&c, 1);
}