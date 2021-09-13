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


/* library functions declaration */
void ssd1306_send_cmdlist(const uint8_t* list, size_t lenght);
void ssd1306_send_data(const uint8_t* data, size_t lenght);
void ssd1306_init();
void ssd1306_refresh();

uint8_t ssd1306_draw_pixel(uint16_t x, uint16_t y, uint8_t value);

/* library function implementation */
static uint8_t i2c_output_buffer[1024]; /* i2c outgoing buffer */
static uint8_t ssd1306_vram[SSD1306_COLUMNS][SSD1306_ROWS / 8] = { 0 };

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
        SSD1306_COLSCAN_DESCENDING,     /* flip columns */
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

void ssd1306_refresh(){
    ssd1306_send_data((uint8_t*)ssd1306_vram, 512);
}

uint8_t ssd1306_draw_pixel(uint16_t x, uint16_t y, uint8_t value){ /* dunno how this works really */
    /* static variables used only in the function calls multiple times */
    static uint8_t page;
    static uint8_t config[6];
    static uint8_t updated_vram_page;

    // ensure pixel location is valid
    if (x >= SSD1306_COLUMNS)   return 1;
    if (y >= SSD1306_ROWS)      return 2;

    page = y >> 3; /* selects proper page ( y / 8) */
    
    /* prolly can be more optimised D: */
    config[0] = SSD1306_SETPAGERANGE;
    config[1] = page;
    config[2] = page;
    config[3] = SSD1306_SETCOLRANGE;
    config[4] = x;
    config[5] = x;

    /* send the config over display to select proper page */
    ssd1306_send_cmdlist(config, sizeof(config));
    
    /* draw the pixel to VRAM, keeping all other VRAM information */    
    if(value){
        ssd1306_vram[page][x] |=   0x01 << (y & 0x07);
    } else {
        ssd1306_vram[page][x] &= ~(0x01 << (y & 0x07));
    }

    /* 1 page has 8 bits or a uint8_t * 127 columns,
    selecting the proper page and column, writing there
    will update the 8 bits in that page on that column */


    /* updates display VRAM with the changed byte only */
    updated_vram_page = ssd1306_vram[page][x];
    ssd1306_send_data(&updated_vram_page, sizeof(updated_vram_page));

    return 0;
}

void ssd1306_draw_pixel2(uint8_t x, uint8_t y, uint8_t value){
    if(value){
        ssd1306_vram[x][y] |=   0x01 << (y & 0x07);
    } else {
        ssd1306_vram[x][y] &= ~(0x01 << (y & 0x07));
    }    
}

void ssd1306_init(){
    ssd1306_send_cmdlist(ssd1306_cmdlist_init, sizeof(ssd1306_cmdlist_init));
    memset((uint8_t*)ssd1306_vram, 0x00, 512);
    memset(i2c_output_buffer, 0x00, sizeof(i2c_output_buffer));
    ssd1306_refresh();
}

/* default i2c example program */
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() {
    /* enable UART so we can print status output */
    stdio_init_all();
    sleep_ms(5000); /* wait 5 sec for everything to initialise */

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    // This example will use I2C0 on the default SDA and SCL pins (GPIO 4, GPIO 5 on a Pico, 5,6 physically)
    i2c_init(i2c0, 400000); // 100khz baud rate
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
   
    while(1){
        for(int i = 0; i < 120; i++){
            for(int j = 0; j < 32; j++){
                ssd1306_draw_pixel(i, j, 1);
            }
        }

        for(int i = 0; i < 120; i++){
            for(int j = 0; j < 32; j++){
                ssd1306_draw_pixel(i, j, 0);
            }
        }        
    }

    return 0;
#endif
}