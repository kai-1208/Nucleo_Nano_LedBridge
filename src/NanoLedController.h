#ifndef NANO_LED_CONTROLLER_H
#define NANO_LED_CONTROLLER_H

enum class LedState : char {
    CommLost    = '1',
    Normal      = '2',
    Auto        = '3',
    SemiAuto    = '4',
    HighSpeed   = '5',
    LowSpeed    = '6'
};

/**
 * @brief
 */
void setupNanoLedController();

/**
 * @brief
 * 
 * @param state 送信したい状態(LedState列挙型)
 */
void sendLedState(LedState state);

#endif // NANO_LED_CONTROLLER_H