#include "mbed.h"
#include "NanoLedController.h"

// LedState previous_state = LedState::Unknown;
// LedState current_state = LedState::Normal;

// void state_report_thread() {
//     if () {
//         current_state = LedState::CommLost;
//     } else if () {
//         current_state = LedState::Normal;
//     } else if () {
//         current_state = LedState::Auto;
//     } else if () {
//         current_state = LedState::SemiAuto;
//     } else if () {
//         current_state = LedState::HighSpeed;
//     } else if () {
//         current_state = LedState::LowSpeed;
//     }

//     if (current_state != previous_state) {
//         Led.sendLedState(current_state);
//         previous_state = current_state;
//     }
// }

int main() {
    // 最初に一度だけ、LEDコントローラーを初期化
    NanoLedController Led(PA_9, PA_10);
    Led.setup();
    BufferedSerial pc(USBTX, USBRX, 9600);

    while (1) {
        // LEDState::
        // CommLost    = '1' 通信遮断
        // Normal      = '2' 通常(通信OK)
        // Auto        = '3' 自動
        // SemiAuto    = '4' 半自動(センサ・システム動作中)
        // HighSpeed   = '5' 高速モード
        // LowSpeed    = '6' 低速モード
        
        // state_report_thread();

        pc.write("1\n", 2);
        Led.sendLedState(LedState::CommLost); // CommLostをarduino nanoの方に送ってる
        ThisThread::sleep_for(3s); // 3秒待機

        pc.write("2\n", 2);
        Led.sendLedState(LedState::Normal);
        ThisThread::sleep_for(3s);

        pc.write("3\n", 2);
        Led.sendLedState(LedState::Auto);
        ThisThread::sleep_for(3s);

        pc.write("4\n", 2);
        Led.sendLedState(LedState::SemiAuto);
        ThisThread::sleep_for(3s);

        pc.write("5\n", 2);
        Led.sendLedState(LedState::HighSpeed);
        ThisThread::sleep_for(3s);

        pc.write("6\n", 2);
        Led.sendLedState(LedState::LowSpeed);
        ThisThread::sleep_for(3s);
    }
}