#include <stdio.h>
#include "pico/stdlib.h"


int main()
{
    stdio_init_all();
    char i = 0;

    while (true) {
        printf("Hello, world! %d\n",i);
        i++;
        sleep_ms(10);
    }
}
