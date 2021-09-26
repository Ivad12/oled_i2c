#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* Hardware description */
#define SSD1306_ADDRESS     0x3C
#define SSD1306_ROWS        32 
#define SSD1306_COLUMNS     128
#define SSD1306_PAGE_START  0
#define SSD1306_PAGE_STOP   ((SSD1306_ROWS / 8) - 1)
#define SSD1306_COL_START   0
#define SSD1306_COL_STOP    (SSD1306_COLUMNS - 1)

/* SSD1306 commands - see datasheet */
#define SSD1306_CTRLBYTE_CMD        0x00    /* indicates following bytes are commands */
#define SSD1306_CTRLBYTE_DATA       0x40    /* indicates following bytes are data */

/* Fundamental Command Table (p. 28) */
#define SSD1306_SETCONTRAST         0x81    // double-byte command to set contrast (1-256)
#define SSD1306_ENTIREDISPLAY_ON    0xA5    // set entire display on
#define SSD1306_ENTIREDISPLAY_OFF   0xA4    // use RAM contents for display
#define SSD1306_SETINVERT_ON        0xA7    // invert RAM contents to display
#define SSD1306_SETINVERT_OFF       0xA6    // normal display
#define SSD1306_SETDISPLAY_OFF      0xAE    // display OFF (sleep mode)
#define SSD1306_SETDISPLAY_ON       0xAF    // display ON (normal mode)

/* Scrolling Command Table (p. 28-30) */
#define SSD1306_SCROLL_SETUP_H_RIGHT    0x26    // configure right horizontal scroll
#define SSD1306_SCROLL_SETUP_H_LEFT     0x27    // configure left horizontal scroll
#define SSD1306_SCROLL_SETUP_HV_RIGHT   0x29    // configure right & vertical scroll
#define SSD1306_SCROLL_SETUP_HV_LEFT    0x2A    // configure left & vertical scroll
#define SSD1306_SCROLL_SETUP_V          0xA3    // configure vertical scroll area
#define SSD1306_SCROLL_DEACTIVATE       0x2E    // stop scrolling
#define SSD1306_SCROLL_ACTIVATE         0x2F    // start scrolling

// Addressing Setting Command Table (pp. 30-31)
#define SSD1306_PAGE_COLSTART_LOW   0x00    // set lower 4 bits of column start address by ORing 4 LSBs
#define SSD1306_PAGE_COLSTART_HIGH  0x10    // set upper 4 bits of column start address by ORing 4 LSBs
#define SSD1306_PAGE_PAGESTART      0xB0    // set page start address by ORing 4 LSBs
#define SSD1306_SETADDRESSMODE      0x20    // set addressing mode (horizontal, vertical, or page)
#define SSD1306_SETCOLRANGE         0x21    // send 2 more bytes to set start and end columns for hor/vert modes
#define SSD1306_SETPAGERANGE        0x22    // send 2 more bytes to set start and end pages

// Hardware Configuration Commands (p. 31)
#define SSD1306_SETSTARTLINE        0x40    // set RAM display start line by ORing 6 LSBs
#define SSD1306_COLSCAN_ASCENDING   0xA0    // set column address 0 to display column 0
#define SSD1306_COLSCAN_DESCENDING  0xA1    // set column address 127 to display column 127
#define SSD1306_SETMULTIPLEX        0xA8    // set size of multiplexer based on display height (31 for 32 rows)
#define SSD1306_COMSCAN_ASCENDING   0xC0    // set COM 0 to display row 0
#define SSD1306_COMSCAN_DESCENDING  0xC8    // set COM N-1 to display row 0
#define SSD1306_VERTICALOFFSET      0xD3    // set display vertical shift
#define SSD1306_SETCOMPINS          0xDA    // set COM pin hardware configuration

// Timing and Driving Scheme Settings Commands (p. 32)
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5    // set display clock divide ratio and frequency
#define SSD1306_SETPRECHARGE        0xD9    // set pre-charge period
#define SSD1306_SETVCOMLEVEL        0xDB    // set V_COMH voltage level
#define SSD1306_NOP                 0xE3    // no operation

// Charge Pump Commands (p. 62)
#define SSD1306_SETCHARGEPUMP       0x8D    // enable / disable charge pump

/* new font table designed for ssd1306 */
static const uint8_t ssd1306_font6x8 [][6]=
 {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sp
   0x00, 0x00, 0x00, 0x2f, 0x00, 0x00, // !
   0x00, 0x00, 0x07, 0x00, 0x07, 0x00, // "
   0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14, // #
   0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12, // $
   0x00, 0x23, 0x13, 0x08, 0x64, 0x62, // %
   0x00, 0x36, 0x49, 0x55, 0x22, 0x50, // &
   0x00, 0x00, 0x05, 0x03, 0x00, 0x00, // '
   0x00, 0x00, 0x1c, 0x22, 0x41, 0x00, // (
   0x00, 0x00, 0x41, 0x22, 0x1c, 0x00, // )
   0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, // *
   0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, // +
   0x00, 0x00, 0x00, 0xA0, 0x60, 0x00, // ,
   0x00, 0x08, 0x08, 0x08, 0x08, 0x08, // -
   0x00, 0x00, 0x60, 0x60, 0x00, 0x00, // .
   0x00, 0x20, 0x10, 0x08, 0x04, 0x02, // /
   0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
   0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, // 1
   0x00, 0x42, 0x61, 0x51, 0x49, 0x46, // 2
   0x00, 0x21, 0x41, 0x45, 0x4B, 0x31, // 3
   0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, // 4
   0x00, 0x27, 0x45, 0x45, 0x45, 0x39, // 5
   0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
   0x00, 0x01, 0x71, 0x09, 0x05, 0x03, // 7
   0x00, 0x36, 0x49, 0x49, 0x49, 0x36, // 8
   0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, // 9
   0x00, 0x00, 0x36, 0x36, 0x00, 0x00, // :
   0x00, 0x00, 0x56, 0x36, 0x00, 0x00, // ;
   0x00, 0x08, 0x14, 0x22, 0x41, 0x00, // <
   0x00, 0x14, 0x14, 0x14, 0x14, 0x14, // =
   0x00, 0x00, 0x41, 0x22, 0x14, 0x08, // >
   0x00, 0x02, 0x01, 0x51, 0x09, 0x06, // ?
   0x00, 0x32, 0x49, 0x59, 0x51, 0x3E, // @
   0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C, // A
   0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, // B
   0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, // C
   0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, // D
   0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, // E
   0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, // F
   0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A, // G
   0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, // H
   0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, // I
   0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, // J
   0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, // K
   0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, // L
   0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, // M
   0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, // N
   0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, // O
   0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, // P
   0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
   0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, // R
   0x00, 0x46, 0x49, 0x49, 0x49, 0x31, // S
   0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, // T
   0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, // U
   0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, // V
   0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, // W
   0x00, 0x63, 0x14, 0x08, 0x14, 0x63, // X
   0x00, 0x07, 0x08, 0x70, 0x08, 0x07, // Y
   0x00, 0x61, 0x51, 0x49, 0x45, 0x43, // Z
   0x00, 0x00, 0x7F, 0x41, 0x41, 0x00, // [
   0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55, // 55
   0x00, 0x00, 0x41, 0x41, 0x7F, 0x00, // ]
   0x00, 0x04, 0x02, 0x01, 0x02, 0x04, // ^
   0x00, 0x40, 0x40, 0x40, 0x40, 0x40, // _
   0x00, 0x00, 0x01, 0x02, 0x04, 0x00, // '
   0x00, 0x20, 0x54, 0x54, 0x54, 0x78, // a
   0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, // b
   0x00, 0x38, 0x44, 0x44, 0x44, 0x20, // c
   0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, // d
   0x00, 0x38, 0x54, 0x54, 0x54, 0x18, // e
   0x00, 0x08, 0x7E, 0x09, 0x01, 0x02, // f
   0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C, // g
   0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, // h
   0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, // i
   0x00, 0x40, 0x80, 0x84, 0x7D, 0x00, // j
   0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, // k
   0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, // l
   0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, // m
   0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, // n
   0x00, 0x38, 0x44, 0x44, 0x44, 0x38, // o
   0x00, 0xFC, 0x24, 0x24, 0x24, 0x18, // p
   0x00, 0x18, 0x24, 0x24, 0x18, 0xFC, // q
   0x00, 0x7C, 0x08, 0x04, 0x04, 0x08, // r
   0x00, 0x48, 0x54, 0x54, 0x54, 0x20, // s
   0x00, 0x04, 0x3F, 0x44, 0x40, 0x20, // t
   0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C, // u
   0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, // v
   0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, // w
   0x00, 0x44, 0x28, 0x10, 0x28, 0x44, // x
   0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C, // y
   0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, // z
   0x00, 0x00, 0x08, 0x77, 0x00, 0x00, // {
   0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, // |
   0x00, 0x00, 0x77, 0x08, 0x00, 0x00, // }
   0x00, 0x10, 0x08, 0x10, 0x08, 0x00, // ~
   0x14, 0x14, 0x14, 0x14, 0x14, 0x14, // horiz lines // DEL
 };

/* library functions definition */
void ssd1306_send_cmdlist(const uint8_t* list, size_t lenght);
void ssd1306_send_data(const uint8_t* data, size_t lenght);
void ssd1306_init();
void ssd1306_refresh();

uint8_t ssd1306_draw_pixel(uint16_t x, uint16_t y, uint8_t value);

/* static global variables */
static uint8_t i2c_output_buffer[1024]; /* i2c outgoing buffer */
static uint8_t ssd1306_vram[SSD1306_ROWS / 8][SSD1306_COLUMNS]; /* vram buffer */

static const uint8_t ssd1306_cmdlist_init[] = {
    SSD1306_SETDISPLAY_OFF,         /* turn off display */
    SSD1306_SETDISPLAYCLOCKDIV,     /* set clock: */
    0x80,                           /* Fosc = 8, divide ratio = 0 + 1 */
    SSD1306_SETMULTIPLEX,           /* sets multiplex ratio of 31 */
    (SSD1306_ROWS - 1),             /* COM0 to COM31 on the display */
    SSD1306_VERTICALOFFSET,         /* display vertical offset: */
    0,                              /* no vertical offset */
    SSD1306_SETSTARTLINE | 0x00,    /* RAM start line at 0 */
    SSD1306_SETCHARGEPUMP,          /* charge pump */
    0x14,                           /* charge pump ON (0x10 for OFF) */
    SSD1306_SETADDRESSMODE,         /* addressing mode */
    0x00,                           /* horizontal addressing mode */
    SSD1306_COLSCAN_ASCENDING,      /* dont flip comumns */
    SSD1306_COMSCAN_ASCENDING,      /* don't flip rows (pages) */
    SSD1306_SETCOMPINS,             /* set COM pins */
    0x02,                           /* sequential pin mode */
    SSD1306_SETCONTRAST,            /* set contrast */
    0x7F,                           /* datasheet default */
    SSD1306_SETPRECHARGE,           /* set precharge period */
    0xF1,                           /* phase1 = 15, phase2 = 1 */ 
    SSD1306_SETVCOMLEVEL,           /* set VCOMH deselect level */
    0x40,                           /* ????? (0,2,3) */
    SSD1306_ENTIREDISPLAY_OFF,      /* use RAM contents for display */
    SSD1306_SETINVERT_OFF,          /* no inversion */
    SSD1306_SCROLL_DEACTIVATE,      /* no scrolling */
    SSD1306_SETDISPLAY_ON,          /* turn on display */
};

void ssd1306_send_cmdlist(const uint8_t* list, size_t lenght){
    i2c_output_buffer[0] = SSD1306_CTRLBYTE_CMD;
    memcpy(i2c_output_buffer + sizeof(uint8_t), list, lenght);
    i2c_write_blocking(i2c0, SSD1306_ADDRESS, i2c_output_buffer, lenght + 1, false);
}

void ssd1306_send_data(const uint8_t* data, size_t lenght){
    i2c_output_buffer[0] = SSD1306_CTRLBYTE_DATA;
    memcpy(i2c_output_buffer + sizeof(uint8_t), data, lenght);
    i2c_write_blocking(i2c0, SSD1306_ADDRESS, i2c_output_buffer, lenght + 1, false);    
}

/* commands for horizontal addressing mode */
void ssd1306_set_page_address(uint8_t page_start, uint8_t page_end){

}

void ssd1306_set_column_address(uint8_t column_start, uint8_t column_end){

}

void ssd1306_init(){
    ssd1306_send_cmdlist(ssd1306_cmdlist_init, sizeof(ssd1306_cmdlist_init));
    ssd1306_refresh();    
}

void ssd1306_refresh(){ /* sets whole vram to 0s */
    ssd1306_send_data((uint8_t*)ssd1306_vram, 512);
}

/* Draws character from font table, starting from page 0 column 0 */
void ssd1306_draw_character(uint8_t c){
    static uint8_t updated_vram_page;
    static uint8_t char_byte; /* 1 portion of 6x8 character */ 

    /* draw character to the vram */
    for(int i = 0; i < 6; i++){
        char_byte = ssd1306_font6x8[c][i];
        ssd1306_send_data(&char_byte, 1);
    }    
}

/* default i2c example program */
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() {
    /* enable UART so we can print status output */
    stdio_init_all();
    /* sleep_ms(5000); /* wait 5 sec for everything to initialise */

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    /* This example will use I2C0 on the default SDA and SCL pins (GPIO 4, GPIO 5 on a Pico, 5,6 physically) */
    i2c_init(i2c0, 400000); /* 400khz baud rate */
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        /* Perform a 1-byte dummy read from the probe address. If a slave
        acknowledges this address, the function returns the number of bytes
        transferred. If the address byte is ignored, the function returns
        -1. */

        /* Skip over any reserved addresses. */
        int ret;
        uint8_t rxdata = 0;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }

    printf("Done.\n");



    /* my code starts here */

    ssd1306_init();
     static uint8_t config[6];
    config[0] = SSD1306_SETPAGERANGE;
    config[1] = 0;
    config[2] = 3;
    config[3] = SSD1306_SETCOLRANGE;
    config[4] = 0;
    config[5] = 128 - 1;
    ssd1306_send_cmdlist(config, sizeof(config));
   
    const char my_name1[] = "This ";
    const char my_name2[] = "works ";
    const char my_name3[] = "perfectly !!!";

    while(1){
        
        for(int i = 0; i < strlen(my_name1); i++){
            ssd1306_draw_character(my_name1[i] - 32);
        }

        sleep_ms(2000);

        for(int i = 0; i < strlen(my_name2); i++){
            ssd1306_draw_character(my_name2[i] - 32);
        }

        sleep_ms(2000);

        for(int i = 0; i < strlen(my_name3); i++){
            ssd1306_draw_character(my_name3[i] - 32);
        }   

        sleep_ms(10000);         
    }

    return 0;
#endif
}
