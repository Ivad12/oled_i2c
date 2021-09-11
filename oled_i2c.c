#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define OLED_SCREEN_WIDTH   128
#define OLED_SCREEN_HEIGHT  32

/* Control byte ------------------------------------------------------------------ */

#define OLED_CMD                     0x00    /* Co = 0, D/C# = 0 */
#define OLED_DATA                    0x40    /* Co = 0, D/C# = 1 */

/* Fundamental Command Table (1 byte) -------------------------------------------- */

#define OLED_SET_CONTRAST           0x81
#define OLED_RESUME_TO_RAM          0xA4
#define OLED_ENTIRE_DISPLAY_ON      0xA5
#define OLED_SET_NORMAL_DISPLAY     0xA6
#define OLED_SET_INVERSE_DISPLAY    0xA7
#define OLED_SET_DISPLAY_OFF        0xAE
#define OLED_SET_DISPLAY_ON         0xAF

/* ------------------------------------------------------------------------------ */

#define OLED_ACTIVATE_SCROLL        0x2E
#define OLED_DEACTIVATE_SCROLL      0x2F
#define OLED_SET_COMPINS            0xDA
#define OLED_SET_COM_OUTPUT_SCANDIR 0xC8
#define OLED_SET_SEGMENT_REMAP      0xA0
#define OLED_SET_MEMORY_MODE        0x20
#define OLED_ENABLE_CHARGEPUMP      0x8D
#define OLED_SET_MULTIPLEX_RATIO    0xA8
#define OLED_SET_DISPLAY_OFFSET     0xD3
#define OLED_SET_START_LINE         0x40


#define OLED_SET_CLKDIV_AND_FOSC    0xD5
#define OLED_SET_PRECHARGE_PERIOD   0xD9
#define OLED_SET_VCOM_DESEL_LEVEL   0xDB
#define OLED_NOP                    0xE3

#define OLED_ENABLE_CHARGEPUMP      0x8D

// Hardware description
#define SSD1306_I2C_ADDRESS 0x3C    // default I2C address
#define SSD1306_ROWS        32      // number of rows on display
#define SSD1306_COLUMNS     128     // number of columns on display
#define SSD1306_PAGE_START  0
#define SSD1306_PAGE_STOP   ((SSD1306_ROWS / 8) - 1)
#define SSD1306_COL_START   0
#define SSD1306_COL_STOP    (SSD1306_COLUMNS - 1)
// SSD1306 Commands - see Datasheet
#define SSD1306_CMD_START   0x00    // indicates following bytes are commands
#define SSD1306_DATA_START  0x40    // indicates following bytes are data
// Fundamental Command Table (p. 28)
#define SSD1306_SETCONTRAST         0x81    // double-byte command to set contrast (1-256)
#define SSD1306_ENTIREDISPLAY_ON    0xA5    // set entire display on
#define SSD1306_ENTIREDISPLAY_OFF   0xA4    // use RAM contents for display
#define SSD1306_SETINVERT_ON        0xA7    // invert RAM contents to display
#define SSD1306_SETINVERT_OFF       0xA6    // normal display
#define SSD1306_SETDISPLAY_OFF      0xAE    // display OFF (sleep mode)
#define SSD1306_SETDISPLAY_ON       0xAF    // display ON (normal mode)
// Scrolling Command Table (pp. 28-30)
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
// function definitions

/* 
Before every command a control byte must be sent.
Some commands require multiple operands !!!
Below is typical init sequence for the display. 
*/

const uint8_t oled_addr = 0x3C;      /* display i2c address */
const uint8_t ctrl_byte_cmd = 0x00;  /* Co = 0, D/C# = 0 */
const uint8_t ctrl_byte_data = 0x40; /* Co = 0, D/C# = 1 */
uint8_t cmd_buffer[] = { 0 }; /* buffer for commands, just before getting send over i2c */
uint8_t out_buffer[1024] = { 0 };

static uint8_t ssd1306_vram[SSD1306_ROWS / 8][SSD1306_COLUMNS] = { 0 };

void ssd1306_refresh(){             /* sends VRAM to the screen GDDRAM via out_buffer */
    out_buffer[0] = OLED_DATA;
    memcpy(out_buffer + sizeof(uint8_t), ssd1306_vram, sizeof(ssd1306_vram));
    i2c_write_blocking(i2c0, oled_addr, out_buffer, sizeof(out_buffer),false);
}

static const uint8_t ssd1306_cmdlist_init[] = {
        SSD1306_CMD_START,              // start commands
        SSD1306_SETDISPLAY_OFF,         // turn off display
        SSD1306_SETDISPLAYCLOCKDIV,     // set clock:
        0x80,                           // Fosc = 8, divide ratio = 0+1
        SSD1306_SETMULTIPLEX,           // display multiplexer:
        (SSD1306_ROWS - 1),             // number of display rows
        SSD1306_VERTICALOFFSET,         // display vertical offset:
        0,                              // no offset
        SSD1306_SETSTARTLINE | 0x00,    // RAM start line 0
        SSD1306_SETCHARGEPUMP,          // charge pump:
        0x14,                           // charge pump ON (0x10 for OFF)
        SSD1306_SETADDRESSMODE,         // addressing mode:
        0x00,                           // horizontal mode
        SSD1306_COLSCAN_DESCENDING,     // flip columns
        SSD1306_COMSCAN_ASCENDING,      // don't flip rows (pages)
        SSD1306_SETCOMPINS,             // set COM pins
        0x02,                           // sequential pin mode
        SSD1306_SETCONTRAST,            // set contrast
        0x00,                           // minimal contrast
        SSD1306_SETPRECHARGE,           // set precharge period
        0xF1,                           // phase1 = 15, phase2 = 1
        SSD1306_SETVCOMLEVEL,           // set VCOMH deselect level
        0x40,                           // ????? (0,2,3)
        SSD1306_ENTIREDISPLAY_OFF,      // use RAM contents for display
        SSD1306_SETINVERT_OFF,          // no inversion
        SSD1306_SCROLL_DEACTIVATE,      // no scrolling
        SSD1306_SETDISPLAY_ON,          // turn on display (normal mode)
};

void ssd1306_drawPixel(uint16_t x, uint16_t y, uint8_t value){

    // send configuration message
    uint8_t configMsg[] = {
        SSD1306_CMD_START,           // start commands
        SSD1306_SETPAGERANGE,        // set page range:
        y >> 3,                      //   y / 8
        y >> 3,                      //   y / 8
        SSD1306_SETCOLRANGE,         // set column range:
        x,                           //   x
        x                            //   x
    };

    i2c_write_blocking(i2c0, oled_addr, configMsg, sizeof(configMsg), false);
 
    
    uint8_t dataMsg[] = {
        SSD1306_DATA_START,         // start data
        value << (y & 0x07)         //   y % 8
    };

    i2c_write_blocking(i2c0, oled_addr, dataMsg, sizeof(dataMsg), false); 
}


void oled_send_command_single(uint8_t cmd){ /* raboti */
    cmd_buffer[0] = 0x00;
    cmd_buffer[1] = cmd;
    i2c_write_blocking(i2c0, oled_addr, cmd_buffer, 2, false);
}

void oled_send_command_list(const uint8_t* list, size_t lenght){
    i2c_write_blocking(i2c0, oled_addr, list, lenght, false);
}

void oled_send_bitmap(uint8_t* bitmap, size_t lenght){
    bitmap[0] = 0x40; /* following data is GDDRAM data */
    i2c_write_blocking(i2c0, oled_addr, bitmap, lenght, false);
}

void oled_init(){
    oled_send_command_list(ssd1306_cmdlist_init, sizeof(ssd1306_cmdlist_init));
}



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
    i2c_init(i2c0, 100000); // 100khz baud rate
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
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

    
    oled_init();
    memset(ssd1306_vram, 0xFF, sizeof(ssd1306_vram));  /* fill entire screen */
    ssd1306_refresh();  
   
    while(1){         
        memset(ssd1306_vram, 0xFF, sizeof(ssd1306_vram));  /* fill entire screen */
        ssd1306_refresh();
        sleep_ms(1000);
        memset(ssd1306_vram, 0x00, sizeof(ssd1306_vram));  /* empty entire screen */
        ssd1306_refresh();
        sleep_ms(1000);
    }

    return 0;
#endif
}