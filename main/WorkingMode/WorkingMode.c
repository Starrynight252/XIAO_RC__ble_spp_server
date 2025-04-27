#include "WorkingMode.h"

// 通用发送缓存，最大
#define TX_BUF_SIZE 64
// 需要发送的数组
uint8_t device_info_buf[TX_BUF_SIZE];

// 通用工作模式
void general_working_mode(uint16_t *keypad_status)
{
    // 需要发送的大小
    int8_t bufsize = 0;

    printf("->>>>keypad_status%u\n", *keypad_status);

    if (keypad_status != NULL)
    {

        device_info_buf[bufsize++] = 0xff;

        device_info_buf[bufsize++] = 0;
        device_info_buf[bufsize++] = 0;
        device_info_buf[bufsize++] = 0;
        device_info_buf[bufsize++] = 0; // 4->5

        // Rtrigger|Rshoulder|Ltrigger|Lshoulder|....
        device_info_buf[bufsize++] =
            ((KEY_IS_PRESSED_BOOL(*keypad_status, START_KEY)) << 6) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, HOME_KEY)) << 5) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, SELECT_KEY)) << 4) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, L1_KEY)) << 3) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, L2_KEY)) << 2) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, R1_KEY)) << 1) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, R2_KEY)) << 0);

        //(Key.UP << 7)  | (Key.DO << 6)  | (Key.L << 5)    | (Key.R << 4)    | (Key.A << 3) | (Key.B << 2) | (Key.X << 1) | (Key.Y << 0);
        device_info_buf[bufsize++] =
            ((KEY_IS_PRESSED_BOOL(*keypad_status, UP_KEY)) << 7) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, DOWN_KEY)) << 6) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, LEFT_KEY)) << 5) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, RIGHT_KEY)) << 4) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, A_KEY)) << 3) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, B_KEY)) << 2) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, X_KEY)) << 1) |
            ((KEY_IS_PRESSED_BOOL(*keypad_status, Y_KEY)) << 0);

        device_info_buf[bufsize] =
            (int8_t)(device_info_buf[0]+
                     device_info_buf[1] +
                     device_info_buf[2] +
                     device_info_buf[4] +
                     device_info_buf[5] +
                     device_info_buf[6]);

        ble_send_int_array(device_info_buf, bufsize);
    }
}

// 工作模式 1
void working_mode_one()
{
}