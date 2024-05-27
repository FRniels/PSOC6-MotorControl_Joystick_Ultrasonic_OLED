#include "cyhal.h"
#include <SSD1306_Commands.h>

const uint8_t ssd1306_init_commands[33] = { // INIT COMMANDS IN ORDER FOR EASY USE IN A LOOP
		ANNOUNCE_CMD_BYTE,
		DISP_OFF_CMD,
		SET_MUX_RATIO_CMD,
		RESET_MUX_RATIO_CMD,
		SET_DISP_OFFSET_CMD,
		RESET_DISP_OFFSET_CMD,
		RESET_DISP_START_LINE_CMD,
		RESET_SEGMENT_REMAP_CMD,
		RESET_COM_OUT_SCAN_DIRECT_CMD,
		SET_COM_HARDW_CONFIG_CMD,
		RESET_COM_HARDW_CONFIG_CMD,
		SET_CONTRAST_CONTROL_CMD,
		RESET_CONTRAST_CONTROL_CMD,
		RESUME_PIXS_IN_RAM_CMD,
		SET_DISP_NORMAL_MODE_CMD,
		SET_DISP_CLK_OSC_FREQ_CMD,
		RESET_DISP_CLK_OSC_FREQ_CMD,
		SET_PRECHARGE_CMD,
		RESET_PRECHARGE_CMD,
		SET_VCOMH_DESELECT_LEVEL_CMD,
		RESET_VCOMH_DESELECT_LEVEL_CMD,
		SET_CHARGE_PUMP_CMD,
		ENABLE_CHARGE_PUMP_CMD,
		SET_MEMORY_ADDRESSING_MODE_CMD,
		SET_HORIZONTAL_ADDRESSING_CMD,
		SET_COLUMN_ADDRESS_CMD,
		COLUMN_MIN_START_ADDRESS_CMD,
		COLUMN_MAX_END_ADDRESS_CMD,
		SET_PAGE_ADDRESS_CMD,
		PAGE_MIN_START_ADDRESS_CMD,
		PAGE_MAX_END_ADDRESS_CMD,
		DISP_ON_CMD,
		ALL_PIXS_ON_CMD
};