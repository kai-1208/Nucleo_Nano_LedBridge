#include "NanoLedController.h"

NanoLedController::NanoLedController(PinName tx_pin, PinName rx_pin)
    : _nano_serial(tx_pin, rx_pin) {
}

void NanoLedController::setup() {
    _nano_serial.set_baud(9600);
}

void NanoLedController::sendLedState(LedState state) {
    char c = static_cast<char>(state);
    _nano_serial.write(&c, 1);
}