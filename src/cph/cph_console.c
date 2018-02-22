#include "cph_console.h"

uint8_t uc_char;
uint8_t uc_flag;

uint8_t cph_console_tick(void)
{
    uc_flag = uart_read(CONSOLE_UART, &uc_char);
    if (!uc_flag) {
        switch (uc_char) {
            case 'l':
            return CS_PWMMIN;
            case 'm':
            return CS_PWMMID;
            case 'h':
            return CS_PWMMAX;
            case '-':
            return CS_PWMSTEPDEC;
            case '+':
            return CS_PWMSTEPINC;
            default:
            return CS_NOINPUT;              
        }
    } else {
        return CS_NOINPUT;
    }
}