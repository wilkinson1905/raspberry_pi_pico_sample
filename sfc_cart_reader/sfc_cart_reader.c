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
#define PIN_MISO 4 //Connect to SO of mcp23s17-E/SP(Pin 14)
#define PIN_CS   5 //Connect to CS of mcp23s17-E/SP(Pin 11)
#define PIN_SCK  2 //Connect to SCK of mcp23s17-E/SP(Pin 12)
#define PIN_MOSI 3 //Connect to SI of mcp23s17-E/SP(Pin 13)
#define PIN_RESET 6 //Connect to RESET of mcp23s17-E/SP(Pin 18)
#define HIGH 1
#define LOW 0
#define IOCON_ADDR 0x0A
#define IOCON_SETTINGS 0b00111100 //BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,Unimplemented
#define IODIRA_ADDR 0x00 //This is IODIR Address of A. IODIR 1 as input, 0 as output
#define IODIRB_ADDR 0x01 //This is IODIR Address of B.IODIR 1 as input, 0 as output
#define GPIOA_ADDR 0x12
#define GPIOB_ADDR 0x13
#define OLATA_ADDR 0x14
#define OLATB_ADDR 0x15


void write_registers(uint8_t hw_address, uint8_t address, uint8_t data){
    uint8_t spi_command[3];
    uint8_t opcode = 0b01000000|hw_address << 1;
    gpio_put(PIN_CS, LOW);
    sleep_ms(1);
    spi_command[0] = opcode;
    spi_command[1] = address;
    spi_command[2] = data;
    spi_write_blocking(SPI_PORT,spi_command,sizeof(spi_command));
    gpio_put(PIN_CS, HIGH);
    sleep_ms(1);
}
uint8_t read_registers(uint8_t hw_address, uint8_t address){
    uint8_t spi_command[3];
    uint8_t spi_received_data[3];
    uint8_t opcode = 0b01000001|hw_address << 1;
    gpio_put(PIN_CS, LOW);
    sleep_ms(1);
    spi_command[0] = opcode;
    spi_command[1] = address;
    spi_command[2] = 0x00;
    spi_write_read_blocking(SPI_PORT, spi_command, spi_received_data, sizeof(spi_command));
    gpio_put(PIN_CS, HIGH);
    sleep_ms(1);
    return spi_received_data[2];
}

void set_address(uint32_t address){
    write_registers(0x00, OLATA_ADDR, address & 0xFF);
    write_registers(0x00, OLATB_ADDR, (address >> 8)&0xFF);
    write_registers(0x01, OLATA_ADDR, (address >> 16)&0xFF);
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
    write_registers(0x00, IOCON_ADDR, IOCON_SETTINGS);
    write_registers(0x01, IOCON_ADDR, IOCON_SETTINGS);
    //set GPIO A direction to output
    write_registers(0x00, IODIRA_ADDR, 0x00);
    write_registers(0x01, IODIRA_ADDR, 0x00);
    write_registers(0x00, IODIRB_ADDR, 0x00);
    write_registers(0x01, IODIRB_ADDR, 0xFF);

    uint8_t ic1_GPA;
    uint8_t count=0;

    while (true) {
        set_address(0xFFC0 + count);
        ic1_GPA = read_registers(0x01, GPIOB_ADDR);
        printf("%X : %c\n", 0xFFC0 + count,ic1_GPA);
        sleep_ms(100);
        count++;
        if(count > 21) break;
    }
    while(true){}
}
