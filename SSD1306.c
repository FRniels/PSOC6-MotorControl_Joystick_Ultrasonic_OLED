#include "cyhal.h"
#include "cy_retarget_io.h"

#include <SSD1306_Commands.h>
#include <SSD1306_Types.h>
#include <SSD1306_Settings.h>
#include <Font.h>

void SSD1306_Init(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool generate_stop_condition)
{
	uint8_t data_size = (sizeof(ssd1306_init_commands) / sizeof(ssd1306_init_commands[0])) - 2; // SKIP THE LAST 2 COMMANDS => TO DO: DIVIDE THE COMMAND ARRAY IN PARTS SORTED BY ACTION
	// printf("Send init command. commands size: %d\n\r", (sizeof(ssd1306_init_commands) / sizeof(ssd1306_init_commands[0])) - 2);
	cyhal_i2c_master_write(i2c_master_obj, slave_addr, ssd1306_init_commands, data_size, 10u, generate_stop_condition);
}

void SSD1306_Send_Command_Data(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* command_buffer, uint16_t data_to_send_size /*bool is_cmd_announce_byte_sent, bool generate_stop_condition*/)
{
	cyhal_i2c_master_write(i2c_master_obj, slave_addr, command_buffer, data_to_send_size, 10u, false);
}

void SSD1306_Disp_On(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition) // TO DO: FIX HARDCODING CMD ARRAY INDEX
{
	// ONLY WORKS WHEN FIRST RESENDING THE ANNOUNCE COMMAND

	uint8_t data_size = 0;
	uint8_t commands[2];

	//if(is_cmd_announce_byte_sent)
	//{
		//data_size = 1;
		//commands[0] = ssd1306_init_commands[23]; // TO DO: FIX HARDCODING CMD ARRAY INDEX
	//}
	//else
	//{
		data_size = 2;// First send the command announce byte before sending the 'disp on' cmd
		commands[0] = ssd1306_init_commands[0];
		commands[1] = ssd1306_init_commands[31];
	//}

	// printf("Send DISP ON command. commands size: %d\n\r", data_size);
	cyhal_i2c_master_write(i2c_master_obj, slave_addr, commands, data_size, 10u, generate_stop_condition);
}

void SSD1306_Disp_Off(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition)
{
	// ONLY WORKS WHEN FIRST RESENDING THE ANNOUNCE COMMAND
	uint8_t data_size = 2;
	uint8_t commands[data_size];
	commands[0] = ssd1306_init_commands[0];
	commands[1] = ssd1306_init_commands[1];
	cyhal_i2c_master_write(i2c_master_obj, slave_addr, commands, data_size, 10u, generate_stop_condition);
}

void SSD1306_Light_All_Pixs(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition) // TO DO: FIX HARDCODING CMD ARRAY INDEX
{
	// ONLY WORKS WHEN FIRST RESENDING THE ANNOUNCE COMMAND

	uint8_t data_size = 0;
	uint8_t commands[2];

	//if(is_cmd_announce_byte_sent)
	//{
		//data_size = 1;
		//commands[0] = ssd1306_init_commands[24];
	//}
	//else
	//{
		data_size = 2;// First send the command announce byte before sending the 'disp on' cmd
		commands[0] = ssd1306_init_commands[0];
		commands[1] = ssd1306_init_commands[32];
	//}

	// printf("Send ALL PIXS ON command. commands size: %d\n\r", data_size);
	cyhal_i2c_master_write(i2c_master_obj, slave_addr, commands, data_size, 10u, generate_stop_condition);
}

void SSD1306_Send_GDDRAM_data(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, uint16_t data_to_send_size /*bool is_cmd_announce_byte_sent, bool generate_stop_condition*/) // Image data
{
	cyhal_i2c_master_write(i2c_master_obj, slave_addr, gddram_buffer, data_to_send_size, 10u, false); // original: 10u, changed to:0 and now back to 10u => see if it still works correctly
}

void SSD1306_Display_GDDRAM(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition)
{
	// ONLY WORKS WHEN FIRST RESENDING THE ANNOUNCE COMMAND
	uint8_t data_size = 2;
	uint8_t commands[data_size];
	commands[0] = ssd1306_init_commands[0];
	commands[1] = ssd1306_init_commands[13];
	cyhal_i2c_master_write(i2c_master_obj, slave_addr, commands, data_size, 10u, generate_stop_condition);
}

void SSD1306_Clear_GDDRAM(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, pixel_state_t pix_state /* bool is_cmd_announce_byte_sent, bool generate_stop_condition */)
{
	gddram_buffer[0] = (uint8_t)ANNOUNCE_GDDRAM_DATA;

	uint8_t buffer_byte = (uint8_t)0x00; // Default b0000 0000 => All pixels off
	if(pix_state == PIXEL_ON)
		buffer_byte = (uint8_t)0xFF;     // b1111 1111 => All pixels on

	for(int i = 1; i < FRAME_BUFFER_SIZE; ++i)
			gddram_buffer[i] = buffer_byte; // Set all pixels in the GDDRAM to '0' or '1' depending on the pix_state the user passed

	SSD1306_Send_GDDRAM_data(i2c_master_obj, slave_addr, gddram_buffer, FRAME_BUFFER_SIZE);
}

void SSD1306_Set_Page_Address(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, Page_Address page_address) // Page0   - Page7
{
	uint8_t commands[4];
	commands[0] = ANNOUNCE_CMD_BYTE;
	commands[1] = SET_PAGE_ADDRESS_CMD;

	if(page_address.start >= PAGE_MIN_START_ADDRESS_CMD && page_address.start < PAGE_MAX_END_ADDRESS_CMD + 1)
	{
		commands[2] = page_address.start;
	}
	else
	{
		printf("Page start needs to be between 0 and 7! page start passed: %d\n\r", page_address.start);
		return;
	}

	if(page_address.start <= page_address.end)
	{
		if(page_address.end >= PAGE_MIN_START_ADDRESS_CMD && page_address.end < PAGE_MAX_END_ADDRESS_CMD + 1)
		{
			commands[3] = page_address.end;
		}
		else
		{
			printf("Page end needs to be between 0 and 7! page end passed: %d\n\r", page_address.end);
			return;
		}
	}
	else
	{
		printf("Page end needs to be equal to or larger than the page start! Page start: %d, Page end passed: %d\n\r", page_address.start, page_address.end);
		return;
	}

	SSD1306_Send_Command_Data(i2c_master_obj, slave_addr, commands, 4);
}

void SSD1306_Set_Column_Address(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, Page_Address page_address, Column_Address column_address) // Column0 - Column127
{
	uint8_t commands[4];
	commands[0] = ANNOUNCE_CMD_BYTE;
	commands[1] = SET_COLUMN_ADDRESS_CMD;

	if(column_address.start >= COLUMN_MIN_START_ADDRESS_CMD && column_address.start < COLUMN_MAX_END_ADDRESS_CMD + 1)
	{
		commands[2] = column_address.start;
	}
	else
	{
		printf("Column start needs to be between 0 and 127! Column start passed: %d\n\r", column_address.start);
		return;
	}

	if(column_address.start <= column_address.end || page_address.start < page_address.end) // COLUMN END CAN BE SMALLER THAN COLUMN START => BUT THE END PAGE NEEDS TO BE LARGER THAN THE STARTING PAGE !
	{
		if(column_address.end >= COLUMN_MIN_START_ADDRESS_CMD && column_address.end < COLUMN_MAX_END_ADDRESS_CMD + 1)
		{
			commands[3] = column_address.end;
		}
		else
		{
			printf("Column end needs to be between 0 and 127! Column end passed: %d\n\r", column_address.end);
			return;
		}
	}
	else
	{
		printf("Column end needs to be equal to or larger than the Column start or you need to move to the next page! Column start: %d, Column end passed: %d\n\r", column_address.start, column_address.end);
		return;
	}

	SSD1306_Send_Command_Data(i2c_master_obj, slave_addr, commands, 4);
}

void Draw_Pacman(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, const uint8_t* pacman_data, uint16_t data_to_send_size /*bool is_cmd_announce_byte_sent, bool generate_stop_condition*/)
{
	gddram_buffer[0] = (uint8_t)ANNOUNCE_GDDRAM_DATA;

	for(int i = 1; i < data_to_send_size; ++i) // Copy pacman data to the frame buffer => EXPENSIVE because i copy the whole buffer, but to simulate drawing to specific parts of the frame buffer
		gddram_buffer[i] = pacman_data[i];

	SSD1306_Send_GDDRAM_data(i2c_master_obj, slave_addr, gddram_buffer, data_to_send_size);
}

void Draw_String_8_x_8_Char(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, const uint8_t* string, uint16_t data_to_send_size)
{
	// NOT IMPLEMENTED
}

void Draw_All_Single_8_x_8_Digits(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer)
{
	gddram_buffer[0] = (uint8_t)ANNOUNCE_GDDRAM_DATA;

	int gddram_buffer_index = 1; // TO DO: PLACE CHECKS FOR OVERLOW!

	// WRITE 0 1 2 3 4 5 6 7 8 9 to the buffer
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '0' to buffer
		gddram_buffer[gddram_buffer_index++] = char_0[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '1' to buffer
		gddram_buffer[gddram_buffer_index++] = char_1[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '2' to buffer
		gddram_buffer[gddram_buffer_index++] = char_2[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '3' to buffer
		gddram_buffer[gddram_buffer_index++] = char_3[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '4' to buffer
		gddram_buffer[gddram_buffer_index++] = char_4[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '5' to buffer
		gddram_buffer[gddram_buffer_index++] = char_5[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '6' to buffer
		gddram_buffer[gddram_buffer_index++] = char_6[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '7' to buffer
		gddram_buffer[gddram_buffer_index++] = char_7[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '8' to buffer
		gddram_buffer[gddram_buffer_index++] = char_8[i];
	for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '9' to buffer
		gddram_buffer[gddram_buffer_index++] = char_9[i];

	SSD1306_Send_GDDRAM_data(i2c_master_obj, slave_addr, gddram_buffer, 1 + (10 * FONT_CHAR_8_X_8_COLUMN_COUNT)); // +1 for the announcement byte
}

bool Float_To_String_2Decimals(float float_num, uint8_t float_string[static 6], size_t float_str_len) // RETURN THE GGDRAM BUFFER INDEX FOLLWING THE LAST CHARACTER WRITEN TO THE GGDDRAM BUFFER
{
	if(!float_string)
	{
		printf("Error: Float_To_String_2Decimals() => Passed string buffer is a null pointer\n\r");
		return false;
	}

	if(float_str_len < 6)
	{
		printf("Error: Float_To_String_2Decimals() => Passed string buffer length should be equal to or larger than 6\n\r");
		return false;
	}

	// printf("Turn %.2f to a string\n\r", float_num);
	uint32_t whole_number = (uint32_t)float_num;
	uint8_t whole_number_digit_count = 0;

	// Count digits
	if(whole_number < 10)
		whole_number_digit_count = 2; // Pad the whole number part of the float with a zero. Example: 0.25 => 00.25
	else
		for(uint32_t whole_num = whole_number; whole_num > 0; whole_num /= 10)
			++whole_number_digit_count;

	// printf("Whole number digit part of the float: %d, digit count: %d\n\r", whole_number, whole_number_digit_count); // WORKS CORRECTLY

	const uint8_t decimal_point_count = 1;
	const uint8_t decimal_count = 2;
	if((whole_number_digit_count + decimal_point_count + decimal_count) > float_str_len) // Abort the float conversion when the user passes a larger float than can be parsed in the passed string buffer
	{
		printf("Error: Float_To_String_2Decimals() => float argument has a larger length than the passed string buffer can hold!\n\r");
		return false;
	}

	uint32_t whole_number_copy = (uint32_t)float_num;
	int float_string_index = whole_number_digit_count - 1; // Starting with the index of the least significant digit of the whole number part of the float
	while(float_string_index >= 0)
	{
		// GET NUMBER IN REVERSE ORDER
		// Example loop 1.) 68 % 10 = 8;              loop 2.) 6 % 10 = 6
		float_string[float_string_index] = whole_number_copy % 10;
		// Example loop 1.) (uint32_t)(68 / 10) = 6;  loop 2.) (uint32_t)(6 / 10) = 0;
		whole_number_copy /= 10;
		--float_string_index;
	}

	float_string[whole_number_digit_count] = '.';

	uint8_t first_decimal = (uint8_t)((float_num - whole_number) * 10); // Extract the first decimal: distance % whole_number returns all decimals => Example: 2.15 - 2 = 0.15 => * 10 to shift the first decimal to the left and a cast to extract.
	float_string[whole_number_digit_count + 1] = first_decimal;
	uint8_t second_decimal = (uint8_t)((((float_num - whole_number) * 10) - first_decimal) * 10); // Same method as the first decimal
	float_string[whole_number_digit_count + 2] = second_decimal;

	// TO DO: ADD '\0' char to the byte array
	float_string[whole_number_digit_count + 3] = '\0';

	// !!! SEEMS TO WORK JUST FINE !!!
	// for(int i = 0; i < float_str_len; ++i)
		// printf("%c", float_string[i] + '0');
	// printf("\n\r");

	return true;
}

void Write_Float_String_To_GDDRAM_Buffer(uint8_t* gddram_buffer, uint16_t* buffer_start_index, uint8_t* float_string, uint8_t float_str_len)
{
	for(uint8_t digit = 0; digit < float_str_len; ++digit)
	{
		switch(float_string[digit])
		{
			case 0:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '0' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_0[i];
				break;
			case 1:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '1' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_1[i];
				break;
			case 2:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '2' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_2[i];
				break;
			case 3:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '3' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_3[i];
				break;
			case 4:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '4' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_4[i];
				break;
			case 5:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '5' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_5[i];
				break;
			case 6:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '6' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_6[i];
				break;
			case 7:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '7' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_7[i];
				break;
			case 8:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '8' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_8[i];
				break;
			case 9:
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '9' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_9[i];
				break;
			case '.':
				for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char ',' to buffer
					gddram_buffer[(*buffer_start_index)++] = char_comma[i];
				break;
			default:
				break;
		}
	}
}

bool Draw_Ultrasonic_Sensor_Distance(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, float distance)
{
	// TO DO: ADD NULL CHECK FOR MASTER OBJ AND GDDRAM BUFFER

	gddram_buffer[0] = (uint8_t)ANNOUNCE_GDDRAM_DATA;

	// uint8_t distance_string[4]; // x.xxf: Sensor can only measure up to 4.00m
	// bool is_float_converted = Float_To_String_2Decimals( distance, distance_string, 4);	// Only draw the string on the screen if the float value was successfully converted
	uint8_t distance_string[6]; // x.xxf: Sensor can only measure up to 4.00m
	bool is_float_converted = Float_To_String_2Decimals( distance, distance_string, 6);	// Only draw the string on the screen if the float value was successfully converted

	if(is_float_converted)
	{
		uint16_t gddram_buffer_index = 1;
		// Write_Float_String_To_GDDRAM_Buffer(gddram_buffer, &gddram_buffer_index, distance_string, 4);

		// TO DO: LOOP OVER distance_string until the '\0' char is reached and count the bytes and pass it to Write_Float_String_To_GDDRAM_Buffer. (should always be 5 in the case of the ultasonic)
		Write_Float_String_To_GDDRAM_Buffer(gddram_buffer, &gddram_buffer_index, distance_string, 5);

		for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char 'm' to buffer
			gddram_buffer[gddram_buffer_index++] = char_m[i];

		// SSD1306_Send_GDDRAM_data(i2c_master_obj, slave_addr, gddram_buffer, 1 + (4 * FONT_CHAR_8_X_8_COLUMN_COUNT) + (1 * FONT_CHAR_8_X_8_COLUMN_COUNT)); // 1 byte announcement byte + 4 * 8byte char for the float string "x.xx" + 8byte 'm'

		// TO DO: PASS THE BYTE COUNT OF THE BYTE ARRAY INSTEAD OF THE HARDCODED 5
		SSD1306_Send_GDDRAM_data(i2c_master_obj, slave_addr, gddram_buffer, 1 + (5 * FONT_CHAR_8_X_8_COLUMN_COUNT) + (1 * FONT_CHAR_8_X_8_COLUMN_COUNT)); // 1 byte announcement byte + 5 * 8byte char for the float string "xx.xx" + 8byte 'm'
		return true;
	}
	return false;
}

bool Draw_Motor_Duty_Cycle(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, float duty_cycle)
{
	// TO DO: ADD NULL CHECK FOR MASTER OBJ AND GDDRAM BUFFER
	gddram_buffer[0] = (uint8_t)ANNOUNCE_GDDRAM_DATA;

	uint8_t duty_cycle_string[6]; // xx.xxf: Value between 00.00 and 100.0 but never seem to get 100.0 but 99.99 as max
	bool is_float_converted = Float_To_String_2Decimals(duty_cycle, duty_cycle_string, 6); // Only draw the string on the screen if the float value was successfully converted => TO DO / BUG: FIX NUMBERS BELOW 10.00 => NEEDS TO HAVE A LEADING 0

	if(is_float_converted)
	{
		uint16_t gddram_buffer_index = 1;
		Write_Float_String_To_GDDRAM_Buffer(gddram_buffer, &gddram_buffer_index, duty_cycle_string, 5);

		for(int i = 0; i < FONT_CHAR_8_X_8_COLUMN_COUNT; ++i) // Copy char '%' to buffer
			gddram_buffer[gddram_buffer_index++] = char_percentage[i];

		SSD1306_Send_GDDRAM_data(i2c_master_obj, slave_addr, gddram_buffer, 1 + (5 * FONT_CHAR_8_X_8_COLUMN_COUNT) + (1 * FONT_CHAR_8_X_8_COLUMN_COUNT)); // 1 byte announcement byte + 5 * 8byte char for the float string "xx.xx" + 8byte '%'
		return true;
	}
	return false;
}
