/*
nucleo 1,2 それぞれ状態を検知
nucleo 1: 検知した状態をpcとのシリアル通信経由で、nucleo 2に状態を送信する
nucleo 2: 受信した状態と自身の状態をそれぞれnanoに送信し、2秒ごとにそれぞれの状態を光らせる
*/

#include "mbed.h"
#include "NanoLedController.h"
#include <vector>
#include <string>
#include <chrono>
#include <cstdio>
#include <cstdlib>

using namespace std::chrono;

// 最初に一度だけ、LEDコントローラーを初期化
NanoLedController Led(PA_9, PA_10);
BufferedSerial pc(USBTX, USBRX, 115200);

LedState previous_state = LedState::Unknown;
LedState current_state = LedState::Normal;

// nucleo 2 only
LedState previous_state_2 = LedState::Unknown;
LedState current_state_2 = LedState::Unknown;

CAN can1(PA_11, PA_12, (int)1e6);
CAN can2(PB_12, PB_13, (int)1e6);

bool can1_status = false;
bool can2_status = false;

bool serial_status = false;

// nucleo 2 only
bool display_status = true;



Timer serial_timer;

// nucleo 2 only
Timer state_toggle_timer;

// absolute encoderの検知には、そのまま!update()を使えばよさそう
// あとのAuto, SemiAuto, HighSpeed, LowSpeedは、すべてpcとのシリアル通信で状態を取得する
// CommLost, Auto, SemiAuto, HighSpeed, LowSpeedでなければNormal(通常モード)
// can通信の遮断検知には、既存のコードを活用する

void serial_state_thread() {
    if (pc.readable()) {
        char c;
        pc.read(&c, 1);
        serial_timer.reset();
    }
    if (serial_timer.elapsed_time() > 2s) {
        serial_status = false;
    } else {
        serial_status = true;
    }
}

void state_report_thread() {
    if (!can1_status || !can2_status || !serial_status || !frontRoger.update() || !rightRoger.update() || !leftRoger.update()) {
        current_state = LedState::CommLost;
    } else if (tokens[100] != "OK") { // pcから自動か聞く
        current_state = LedState::Auto;
    } else if (tokens[102] != "OK") { // pcから高速モードか聞く
        current_state = LedState::HighSpeed;
    } else if (tokens[103] != "OK") { // pcから低速モードか聞く
        current_state = LedState::LowSpeed; 
    } else { // なんもなければ
        current_state = LedState::Normal;
    }

    if (current_state != previous_state) {
        // Led.sendLedState(current_state);
        previous_state = current_state;

        // nucleo 1 only
        char state_char = static_cast<char>(current_state); 
        pc.write(&state_char, 1);
    }

    // nucleo 2 only
    if (current_state_2 != previous_state_2) {
        previous_state_2 = current_state_2;
    }
    bool state_changed = (current_state != previous_state || current_state_2 != previous_state_2);

    if (current_state == current_state_2) {
        if (state_changed) {
            Led.sendLedState(current_state);
        }
    } else {
        if (state_changed) {
            state_toggle_timer.reset();
            display_status = true;
            Led.sendLedState(current_state);
        }
        if (state_toggle_timer.read_ms() > 2000) {
            state_toggle_timer.reset();
        }
        if (display_status) {
            Led.sendLedState(current_state);
        } else {
            Led.sendLedState(current_state_2);
        }
        display_status = !display_status;
    }
}

int main() {
    Led.setup();
    while (1) {
        // LEDState::
        // CommLost    = '1' 通信遮断
        // Normal      = '2' 通常(通信OK)
        // Auto        = '3' 自動
        // SemiAuto    = '4' 半自動(センサ・システム動作中)
        // HighSpeed   = '5' 高速モード
        // LowSpeed    = '6' 低速モード
        
        state_report_thread();

        // pc.write("1\n", 2);
        // Led.sendLedState(LedState::CommLost); // CommLostをarduino nanoの方に送ってる
        // ThisThread::sleep_for(3s); // 3秒待機

        // pc.write("2\n", 2);
        // Led.sendLedState(LedState::Normal);
        // ThisThread::sleep_for(3s);

        // pc.write("3\n", 2);
        // Led.sendLedState(LedState::Auto);
        // ThisThread::sleep_for(3s);

        // pc.write("4\n", 2);
        // Led.sendLedState(LedState::SemiAuto);
        // ThisThread::sleep_for(3s);

        // pc.write("5\n", 2);
        // Led.sendLedState(LedState::HighSpeed);
        // ThisThread::sleep_for(3s);

        // pc.write("6\n", 2);
        // Led.sendLedState(LedState::LowSpeed);
        // ThisThread::sleep_for(3s);
    }
}