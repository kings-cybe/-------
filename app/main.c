#include <stdint.h>
#include "console.h"
#include "tim_delay.h"

extern void board_lowlevel_init(void);
extern void bootloader_main(void);

int main(void)
{
    board_lowlevel_init();
    console_init();
    tim_delay_init();

    bootloader_main();

    // extern void JumpApp(uint32_t base);
    // JumpApp(APP_BASE_ADDRESS);

    return 0;
}

first
second
third

add key1
add key2