//This is a sample program for connecting raspbeery pi pico to mcp23s17-E/SP.
//The datasheet of mcp23s17-E/SP is https://akizukidenshi.com/goodsaffix/mcp23017_mcp23s17.pdf
//The datasheet of raspberry pi pico is https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf
//The link of Hardware APIs of pico sdk is https://www.raspberrypi.com/documentation/pico-sdk/hardware.html
//how to build a development environment for raspberry pi pico　is https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16 //Connect to SO of mcp23s17-E/SP(Pin 14)
#define PIN_CS   17 //Connect to CS of mcp23s17-E/SP(Pin 11)
#define PIN_SCK  18 //Connect to SCK of mcp23s17-E/SP(Pin 12)
#define PIN_MOSI 19 //Connect to SI of mcp23s17-E/SP(Pin 13)
#define PIN_RESET 20 //Connect to RESET of mcp23s17-E/SP(Pin 18)
#define HIGH 1
#define LOW 0
#define IOCON_ADDR 0x0A
#define IOCON_SETTINGS 0b00111100 //BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,Unimplemented
#define IODIRA_ADDR 0x00 //IODIR 1 as input, 0 as output
#define OLATA_ADDR 0x14
#define DEVICE_OPCODE_W 0b01000000 //The address of mcp23s17-E/SP is 0x00
#define DEVICE_OPCODE_R 0b01000001 //The address of mcp23s17-E/SP is 0x00


void write_registers(uint8_t opcode, uint8_t address, uint8_t data){
    uint8_t spi_command[3];
    gpio_put(PIN_CS, LOW);
    sleep_ms(1);
    spi_command[0] = opcode;
    spi_command[1] = address;
    spi_command[2] = data;
    spi_write_blocking(SPI_PORT,spi_command,sizeof(spi_command));
    gpio_put(PIN_CS, HIGH);
    sleep_ms(1);
}

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_RESET,   GPIO_FUNC_SIO);
    
    // set GPIO dirction
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    // Reset mcp23s17-E/SP
    gpio_put(PIN_RESET, LOW);
    sleep_ms(100);
    gpio_put(PIN_RESET, HIGH);
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_put(PIN_CS, HIGH);
    sleep_ms(100);
    //set IOCON
    write_registers(DEVICE_OPCODE_W, IOCON_ADDR, IOCON_SETTINGS);
    //set GPIO A direction to output
    write_registers(DEVICE_OPCODE_W, IODIRA_ADDR, 0x00);
    uint8_t count=0;
    while (true) {
        //Set the GPIO A value to count
        write_registers(DEVICE_OPCODE_W, OLATA_ADDR, count);
        sleep_ms(100);
        count++;
    }
}
