
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "hardware/i2c.h"




static uint8_t i2c_output_buffer[1024]; /* i2c outgoing buffer */
static uint8_t ssd1306_vram[512];

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
    memcpy(i2c_output_buffer + 1, list, lenght);
    i2c_write_blocking(i2c0, SSD1306_ADDRESS, i2c_output_buffer, lenght + 1, false);
}

void ssd1306_send_data(const uint8_t* data, size_t lenght){
    i2c_output_buffer[0] = SSD1306_CTRLBYTE_DATA;
    memcpy(i2c_output_buffer + 1, data, lenght);
    i2c_write_blocking(i2c0, SSD1306_ADDRESS, i2c_output_buffer, lenght + 1, false);    
}

void ssd1306_refresh(){
    ssd1306_send_data(ssd1306_vram, sizeof(ssd1306_vram));
}

void ssd1306_draw_pixel(uint16_t x, uint16_t y, uint8_t value){
    /* send configuration message */
    uint8_t cfg[] = {
        SSD1306_SETPAGERANGE,        /* sets page range */
        y >> 3,                      //   y / 8 - fast division
        y >> 3,                      //   y / 8 - fast division
        SSD1306_SETCOLRANGE,         // sets column range:
        x,                           //   x
        x                            //   x
    };

    ssd1306_send_cmdlist(cfg, sizeof(cfg)); 
    
    uint8_t data = value << (y & 0x07);  /*   y % 8 */

    ssd1306_send_data(&data, sizeof(data));
}

void ssd1306_fill_vram(uint8_t value){
    memset(ssd1306_vram, value, sizeof(ssd1306_vram));
}

void ssd1306_init(){
    ssd1306_send_cmdlist(ssd1306_cmdlist_init, sizeof(ssd1306_cmdlist_init));
    memset(ssd1306_vram, 0x00, sizeof(ssd1306_vram));
    memset(i2c_output_buffer, 0x00, sizeof(i2c_output_buffer));
}

