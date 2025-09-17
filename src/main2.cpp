#include "main.h"

BufferedSerial pc(USBTX, USBRX, 115200);

CAN can1(PA_11, PA_12, (int)1e6);
CAN can2(PB_12, PB_13, (int)1e6);

C610 DJI1(can1); // PWMピン指定
C610 DJI2(can2); // PWMピン指定

bool nucleo1_status = true;
bool can1_status = false;           // CAN1の通信ステータス
bool can2_status = false;           // CAN2の通信ステータス
bool last_can1_status = false;      // 前回のCAN1ステータス
bool last_can2_status = false;      // 前回のCAN2ステータス
bool dc_nucleo_can1_status = false; // DC nucleoのCAN1ステータス
auto last_can_status_report_time =
    HighResClock::now(); // 最後にステータスを報告した時刻

bool auto_status = false;
int auto_power[2] = {0, 0};
int box_rack[2] = {0};               // ボックスラックの状態
int sucker_lift[2] = {0};            // サッカーラックの状態
int tire_current[4] = {0};           // タイヤの現在の電流値
int tire_target[4] = {0};            // タイヤの目標の電流値
float stick[3] = {0.0f, 0.0f, 0.0f}; // スティックの値

int kasoku = 200;

DigitalIn left_box_rack(PC_1), right_box_rack(PC_2), left_sucker_limit(PC_3),
    right_sucker_limit(PC_4);
PID left_floor_pid(1.5, 0.3, 0.003, PID::Mode::VELOCITY);
PID right_floor_pid(1.5, 0.3, 0.003, PID::Mode::VELOCITY);
PID left_sucker_rack_pid(0.5, 2.5, 0.009, PID::Mode::VELOCITY);
PID right_sucker_rack_pid(0.5, 2.5, 0.009, PID::Mode::VELOCITY);
PID front_roger_vel_pid(1.5, 0.1, 0.000, PID::Mode::VELOCITY);
PID tire_pid[4] = {
    PID(1.7, 0.0, 0.0, PID::Mode::VELOCITY), // PIDコントローラの初期化
    PID(1.7, 0.0, 0.0, PID::Mode::VELOCITY),
    PID(1.3, 0.0, 0.0, PID::Mode::VELOCITY),
    PID(1.3, 0.0, 0.0, PID::Mode::VELOCITY)};

DigitalOut led(LED1);

NanoLedController Led(PA_0, PA_0);
LedState previous_state = LedState::Unknown;
LedState current_state = LedState::Normal;
LedState previous_state2 = LedState::Unknown;
LedState current_state2 = LedState::Normal;

bool display_status = true;

void move_aa()
{
    while (1)
    {
        static auto last_time = std::chrono::high_resolution_clock::now();
        auto current_time = std::chrono::high_resolution_clock::now();
        last_time = current_time;

        // デッドゾーン処理：各stickが -0.08～0.08 の範囲なら 0.0 にする
        for (int i = 0; i < 3; ++i)
        {
            if (stick[i] > -0.08f && stick[i] < 0.08f)
            {
                stick[i] = 0.0f;
            }
        }
        // PIDコントローラに加速度制御された目標値を設定
        if (auto_status)
        {
            tire_target[0] = ((auto_power[0] - stick[1] + auto_power[1]) * ROTATE);
            tire_target[1] = ((-auto_power[0] - stick[1] + auto_power[1]) * ROTATE);
            tire_target[2] = ((auto_power[0] + stick[1] + auto_power[1]) * ROTATE);
            tire_target[3] = ((-auto_power[0] + stick[1] + auto_power[1]) * ROTATE);
        }
        else
        {
            tire_target[0] = ((stick[0] - stick[1] + stick[2]) * ROTATE);
            tire_target[1] = ((-stick[0] - stick[1] + stick[2]) * ROTATE);
            tire_target[2] = ((stick[0] + stick[1] + stick[2]) * ROTATE);
            tire_target[3] = ((-stick[0] + stick[1] + stick[2]) * ROTATE);
        }

        for (int i = 0; i < 4; ++i)
        {
            if (tire_target[i] > tire_current[i])
            {
                tire_current[i] += kasoku;
                if (tire_current[i] - tire_target[i] > kasoku)
                {
                    tire_current[i] = tire_target[i];
                }
            }
            else if (tire_target[i] < tire_current[i])
            {
                tire_current[i] -= kasoku;
                if (tire_current[i] - tire_target[i] < -kasoku)
                {
                    tire_current[i] = tire_target[i];
                }
            }
            tire_pid[i].set_goal(tire_current[i]);
        }
        ThisThread::sleep_for(5ms);
    }
}

void receive_uart_thread()
{
    static char linebuf[512];
    size_t idx = 0;

    while (true)
    {
        char c;
        // 1バイト読み込む（ノンブロッキング処理）
        if (pc.read(&c, 1) > 0)
        {
            // 終端文字 '|' を受信した場合
            if (c == '|')
            {
                linebuf[idx] = '\0'; // C言語文字列として終端

                if (idx > 0)
                { // 空のメッセージは無視
                    std::string data(linebuf);

                    // 受信データをカンマで分割してトークンのベクターにする（最適化）
                    std::vector<std::string> tokens;
                    tokens.reserve(25); // 事前にメモリを確保
                    std::stringstream ss(data);
                    std::string token;
                    while (std::getline(ss, token, ','))
                    {
                        tokens.push_back(std::move(token));
                    }

                    // コマンド処理関数を呼び出す（デバッグ出力を削減）
                    process_command(tokens);
                }

                idx = 0; // バッファをリセット
            }
            // 終端文字以外を受信した場合
            else
            {
                if (idx < sizeof(linebuf) - 1)
                {
                    linebuf[idx++] = c; // バッファに文字を追加
                }
                else
                {
                    // バッファオーバーフロー対策
                    idx = 0;
                }
            }
        } else {
            // データがない場合は短時間スリープ
            ThisThread::sleep_for(1ms);
        }
    }
}

void can_send_thread()
{
    while (true)
    {
        DJI1.set_power(8, 1000);

        // CAN1とCAN2の送信ステータスをチェック
        bool can1_success = DJI1.send_message();
        ThisThread::sleep_for(15ms);
        bool can2_success = DJI2.send_message();

        // 各CANの送信結果を記録
        can1_status = can1_success;
        can2_status = can2_success;

        ThisThread::sleep_for(15ms);
    }
}

void can_status_monitor_thread()
{
    while (true)
    {
        auto now_time = HighResClock::now();
        auto time_since_last_report =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now_time - last_can_status_report_time)
                .count();

        // 1秒経過したか、またはステータスが変化した場合に報告
        if (time_since_last_report >= 1000 || can1_status != last_can1_status ||
            can2_status != last_can2_status)
        {
            // 6桁のランダムな数値を生成
            int random6_can1 = 100000 + (std::rand() % 900000);
            int random6_can2 = 100000 + (std::rand() % 900000);

            // CAN1ステータス報告
            const char *can1_status_str = can1_status ? "OK" : "NG";
            char can1_status_buf[128];
            sprintf(can1_status_buf,
                    "2,nucleo2,web_server_node,%d,2,DJI-nucleo-can1,%s|",
                    random6_can1, can1_status_str);
            pc.write(can1_status_buf, strlen(can1_status_buf));

            // CAN2ステータス報告
            const char *can2_status_str = can2_status ? "OK" : "NG";
            char can2_status_buf[128];
            sprintf(can2_status_buf,
                    "2,nucleo2,web_server_node,%d,2,DJI-nucleo-can2,%s|",
                    random6_can2, can2_status_str);
            pc.write(can2_status_buf, strlen(can2_status_buf));

            // 最後の報告時刻とステータスを更新
            last_can_status_report_time = now_time;
            last_can1_status = can1_status;
            last_can2_status = can2_status;
        }

        ThisThread::sleep_for(100ms); // 100ms間隔でチェック
    }
}

void pid_control()
{
    front_roger_vel_pid.enable_anti_windup(true);
    front_roger_vel_pid.set_output_limits(-16000, 16000);
    front_roger_vel_pid.set_gravity_offset(300);
    left_floor_pid.set_output_limits(-16000, 16000);
    right_floor_pid.set_output_limits(-16000, 16000);
    left_floor_pid.set_min_drive_power(3000);
    right_floor_pid.set_min_drive_power(4500);
    left_sucker_rack_pid.set_output_limits(-16000, 16000);
    right_sucker_rack_pid.set_output_limits(-16000, 16000);
    left_sucker_rack_pid.set_min_drive_power(1000);
    right_sucker_rack_pid.set_min_drive_power(900);
    tire_pid[0].set_min_drive_power(100);
    tire_pid[1].set_min_drive_power(100);
    tire_pid[2].set_min_drive_power(100);
    tire_pid[3].set_min_drive_power(100);
    tire_pid[0].set_output_limits(-16000, 16000);
    tire_pid[1].set_output_limits(-16000, 16000);
    tire_pid[2].set_output_limits(-16000, 16000);
    tire_pid[3].set_output_limits(-16000, 16000);
    auto pre_time = HighResClock::now();
    while (true)
    {
        if (left_box_rack.read() == 0)
        {
            left_floor_pid.set_goal(0);
        }
        else
        {
            left_floor_pid.set_goal(box_rack[0]);
        }
        if (right_box_rack.read() == 0)
        {
            right_floor_pid.set_goal(0);
        }
        else
        {
            right_floor_pid.set_goal(box_rack[1]);
        }

        // if(right_sucker_rack_pid.get_goal()>0){
        //   led = 1;
        // }
        // else{
        //   led = 0;
        // }
        // if (left_sucker_limit.read() == 0)
        // {
        //   left_sucker_rack_pid.set_goal(0);
        // }
        // else
        // {
        //   left_sucker_rack_pid.set_goal(sucker_lift[0]);
        // }
        // if (right_sucker_limit.read() == 0)
        // {
        //   right_sucker_rack_pid.set_goal(0);
        // }
        // else
        // {
        //   right_sucker_rack_pid.set_goal(-sucker_lift[1]);
        // }
        auto now_time = HighResClock::now();
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(now_time -
                                                                         pre_time)
                       .count() /
                   1000000.0f;
        left_floor_pid.set_dt(dt);
        right_floor_pid.set_dt(dt);
        left_sucker_rack_pid.set_dt(dt);
        right_sucker_rack_pid.set_dt(dt);
        front_roger_vel_pid.set_dt(dt);
        for (int i = 0; i < 4; i++)
        {
            tire_pid[i].set_dt(dt);
        }
        pre_time = now_time;

        // if (rightRoger.update()) {
        //   static float last_rad = 0.00f;
        //   int32_t pos = rightRoger.get_position();
        //   // カスケード制御：位置 → 速度目標 → モータ出力
        //   float target_vel = right_roger_pos_pid.do_pid(pos);
        //   right_roger_vel_pid.set_goal(target_vel);
        //   float rad = rightRoger.get_angle().rad();
        //   float angular_velocity = (rad - last_rad) / dt;        // 角速度
        //   [rad/s] float rpm = angular_velocity * 60.0f / (2.0f * M_PI);  // rpm
        //   [回転/分] float motor_output = right_roger_vel_pid.do_pid(rpm); char
        //   buf[64];
        //   {
        //     static bool seeded = false;
        //     if (!seeded) {
        //       seeded = true;
        //       std::srand((unsigned)std::chrono::high_resolution_clock::now()
        //                      .time_since_epoch()
        //                      .count());
        //     }
        //     int random6 = 100000 + (std::rand() % 900000);
        //     sprintf(buf, "2,nucleo2,nucleo1,%d,2,rightRoger,%d|", random6,
        //             (int)motor_output);
        //     pc.write(buf, strlen(buf));
        //   }
        //   last_rad = rad;
        // } else {
        //   char buf[64];i
        //   {
        //     static bool seeded = false;
        //     if (!seeded) {
        //       seeded = true;
        //       std::srand((unsigned)std::chrono::high_resolution_clock::now()
        //                      .time_since_epoch()
        //                      .count());
        //     }
        //     int random6 = 100000 + (std::rand() % 900000);
        //     sprintf(buf, "2,nucleo2,nucleo1,%d,2,rightRoger,%d|", random6, 0);
        //     pc.write(buf, strlen(buf));
        //   }
        // }
        // if (leftRoger.update()) {
        //   static float last_rad = 0.00f;
        //   int32_t pos = leftRoger.get_position();
        //   // カスケード制御：位置 → 速度目標 → モータ出力
        //   float target_vel = left_roger_pos_pid.do_pid(pos);i
        //   left_roger_vel_pid.set_goal(target_vel);
        //   float rad = leftRoger.get_angle().rad();
        //   float angular_velocity = (rad - last_rad) / dt;        // 角速度
        //   [rad/s] float rpm = angular_velocity * 60.0f / (2.0f * M_PI);  // rpm
        //   [回転/分] float motor_output = left_roger_vel_pid.do_pid(rpm); char
        //   buf[64];
        //   {
        //     static bool seeded = false;
        //     if (!seeded) {
        //       seeded = true;
        //       std::srand((unsigned)std::chrono::high_resolution_clock::now()
        //                      .time_since_epoch()
        //                      .count());
        //     }
        //     int random6 = 100000 + (std::rand() % 900000);
        //     sprintf(buf, "2,nucleo2,nucleo1,%d,2,leftRoger,%d|", random6,
        //             (int)motor_output);
        //     pc.write(buf, strlen(buf));
        //   }
        //   last_rad = rad;
        // } else {
        //   char buf[64];
        //   {
        //     static bool seeded = false;
        //     if (!seeded) {
        //       seeded = true;
        //       std::srand((unsigned)std::chrono::high_resolution_clock::now()
        //                      .time_since_epoch()
        //                      .count());
        //     }
        //     int random6 = 100000 + (std::rand() % 900000);
        //     sprintf(buf, "2,nucleo2,nucleo1,%d,2,leftRoger,%d|", random6, 0);
        //     pc.write(buf, strlen(buf));
        //   }
        // }
        // if (frontRoger.update()) {
        //   static float last_rad = 0.00f;
        //   int32_t pos = frontRoger.get_position();
        //   // カスケード制御：位置 → 速度目標 → モータ出力
        //   float target_vel = front_roger_pos_pid.do_pid(pos / 3);
        //   front_roger_vel_pid.set_goal(target_vel * 19 * 3);
        //   float motor_output = front_roger_vel_pid.do_pid(DJI2.get_rpm(5));
        //   DJI2.set_power(5, motor_output);
        // } else {
        //   DJI2.set_power(5, 0);
        // }

        DJI2.set_power(7, left_floor_pid.do_pid(DJI2.get_rpm(7)));
        DJI2.set_power(6, right_floor_pid.do_pid(DJI2.get_rpm(6)));
        DJI1.set_power(3, left_sucker_rack_pid.do_pid(DJI1.get_rpm(3)));
        DJI1.set_power(4, right_sucker_rack_pid.do_pid(DJI1.get_rpm(4)));
        DJI2.set_power(1, tire_pid[0].do_pid(DJI2.get_rpm(1)));
        DJI2.set_power(2, tire_pid[1].do_pid(DJI2.get_rpm(2)));
        DJI2.set_power(3, tire_pid[2].do_pid(DJI2.get_rpm(3)));
        DJI2.set_power(4, tire_pid[3].do_pid(DJI2.get_rpm(4)));
        DJI2.set_power(5, front_roger_vel_pid.do_pid(DJI2.get_rpm(5)));
        // char buf[64];
        // sprintf(buf, "rpm:%d,goal:%d\n", (int)DJI2.get_rpm(5),
        //         (int)front_roger_vel_pid.get_goal());
        // pc.write(buf, strlen(buf));
        ThisThread::sleep_for(10ms);
    }
}

// neopixel
void state_report_thread() {
    while (true) {
        if (!can1_status || !can2_status) {
            current_state = LedState::CommLost;
        } else if (auto_status) {
            current_state = LedState::Auto;
        } else if (kasoku == 300) {
            current_state = LedState::HighSpeed;
        } else if (kasoku == 200) {
            current_state = LedState::LowSpeed;
        } else {
            current_state = LedState::Normal;
        }

        if (current_state != previous_state) {
            previous_state = current_state;
        }

        bool state_changed = (current_state != previous_state || current_state2 != previous_state2);

        if (current_state == current_state2) {
            if (!state_changed) {
                Led.sendLedState(current_state);
            }
        } else {
            if (state_changed) {
                state_toggle_timer.reset();
                display_status = true;
                Led.sendLedState(current_state);
            }
            if (state_toggle_timer.elapsed_time() > 2s) {
                state_toggle_timer.reset();
            }
            if (display_status) {
                Led.sendLedState(current_state);
            } else {
                Led.sendLedState(current_state2);
            }
            display_status = !display_status;
        }

        ThisThread::sleep_for(50ms); // ついかしました
    }
}

int main() {
    left_box_rack.mode(PullUp);
    right_box_rack.mode(PullUp);
    left_sucker_limit.mode(PullUp);
    right_sucker_limit.mode(PullUp);
    Thread receive_thread;
    receive_thread.start(receive_uart_thread);
    // PIDの周期を設定
    Thread can_thread;
    can_thread.start(can_send_thread);
    Thread pid_thread;
    pid_thread.start(pid_control);
    Thread move;
    move.start(move_aa);
    Thread can_status_thread;
    can_status_thread.start(can_status_monitor_thread);
    Thread serial_state_threa;
    serial_state_threa.start(serial_state_thread);
    Thread state_report_threa;
    state_report_threa.start(state_report_thread);
    // Thread pid_thread;
    // // pid_thread.start(pid_control);
    while (1)
    {
    }
}