#include <asf.h>

int main(void)
{
    sysclk_init();
    board_init();
    delay_init();


    while(1) {

        if(ioport_get_pin_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
            ioport_toggle_pin_level(LED0_GPIO);
            delay_ms(100); 
        }

    }
}