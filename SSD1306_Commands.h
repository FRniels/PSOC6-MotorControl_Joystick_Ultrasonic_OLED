#ifndef SSD1306_COMMANDS_H_
#define SSD1306_COMMANDS_H_

extern const uint8_t ssd1306_init_commands[33];

// INIT COMMANDS IN ORDER
#define ANNOUNCE_CMD_BYTE     	 	   (uint8_t)0x00 // 1
#define DISP_OFF_CMD          	 	   (uint8_t)0xAE // 2
#define SET_MUX_RATIO_CMD     	  	   (uint8_t)0xA8 // 3
#define RESET_MUX_RATIO_CMD   	  	   (uint8_t)0x3F // 4
#define SET_DISP_OFFSET_CMD       	   (uint8_t)0xD3 // 5
#define RESET_DISP_OFFSET_CMD     	   (uint8_t)0x00 // 6
#define RESET_DISP_START_LINE_CMD 	   (uint8_t)0x40 // 7
#define RESET_SEGMENT_REMAP_CMD        (uint8_t)0xA1 // 8
#define RESET_COM_OUT_SCAN_DIRECT_CMD  (uint8_t)0xC8 // 9
#define SET_COM_HARDW_CONFIG_CMD       (uint8_t)0xDA // 10
#define RESET_COM_HARDW_CONFIG_CMD     (uint8_t)0x12 // 11
#define SET_CONTRAST_CONTROL_CMD       (uint8_t)0x81 // 12
#define RESET_CONTRAST_CONTROL_CMD     (uint8_t)0x7F // 13
#define RESUME_PIXS_IN_RAM_CMD         (uint8_t)0xA4 // 14
#define SET_DISP_NORMAL_MODE_CMD       (uint8_t)0xA6 // 15
#define SET_DISP_CLK_OSC_FREQ_CMD      (uint8_t)0xD5 // 16
#define RESET_DISP_CLK_OSC_FREQ_CMD    (uint8_t)0x80 // 17
#define SET_PRECHARGE_CMD              (uint8_t)0xD9 // 18
#define RESET_PRECHARGE_CMD            (uint8_t)0x22 // 19
#define SET_VCOMH_DESELECT_LEVEL_CMD   (uint8_t)0xDB // 20
#define RESET_VCOMH_DESELECT_LEVEL_CMD (uint8_t)0x20 // 21
#define SET_CHARGE_PUMP_CMD			   (uint8_t)0x8D // 22
#define ENABLE_CHARGE_PUMP_CMD         (uint8_t)0x14 // 23
#define SET_MEMORY_ADDRESSING_MODE_CMD (uint8_t)0x20 // 24
#define SET_HORIZONTAL_ADDRESSING_CMD  (uint8_t)0x00 // 25
#define SET_COLUMN_ADDRESS_CMD         (uint8_t)0x21 // 26
#define COLUMN_MIN_START_ADDRESS_CMD   (uint8_t)0x00 // 27
#define COLUMN_MAX_END_ADDRESS_CMD     (uint8_t)0x7F // 28
#define SET_PAGE_ADDRESS_CMD           (uint8_t)0x22 // 29
#define PAGE_MIN_START_ADDRESS_CMD     (uint8_t)0x00 // 30
#define PAGE_MAX_END_ADDRESS_CMD       (uint8_t)0x07 // 31
#define DISP_ON_CMD                    (uint8_t)0xAF // 32
#define ALL_PIXS_ON_CMD                (uint8_t)0xA5 // 33

#define ANNOUNCE_GDDRAM_DATA           (uint8_t)0x40


#endif /* SSD1306_COMMANDS_H_ */
