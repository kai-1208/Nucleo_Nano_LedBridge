#include "mbed.h"
#include "NanoLedController.h"

int main() {
    // 最初に一度だけ、LEDコントローラーを初期化
    setupNanoLedController();
    BufferedSerial pc_serial(USBTX, USBRX, 9600);

    while (1) {
        // LEDState::
        // CommLost    = '1'
        // Normal      = '2'
        // Auto        = '3'
        // SemiAuto    = '4'
        // HighSpeed   = '5'
        // LowSpeed    = '6'

        pc_serial.write("1\n", 2);
        sendLedState(LedState::CommLost);
        ThisThread::sleep_for(3s);

        pc_serial.write("2\n", 2);
        sendLedState(LedState::Normal);
        ThisThread::sleep_for(3s);

        pc_serial.write("3\n", 2);
        sendLedState(LedState::Auto);
        ThisThread::sleep_for(3s);

        pc_serial.write("4\n", 2);
        sendLedState(LedState::SemiAuto);
        ThisThread::sleep_for(3s);

        pc_serial.write("5\n", 2);
        sendLedState(LedState::HighSpeed);
        ThisThread::sleep_for(3s);

        pc_serial.write("6\n", 2);
        sendLedState(LedState::LowSpeed);
        ThisThread::sleep_for(3s);
    }
}