#ifndef SSD1306_H_
#define SSD1306_H_

#include "cyhal.h"
#include <SSD1306_Settings.h>
#include <SSD1306_Types.h>

uint8_t gddram_buffer[FRAME_BUFFER_SIZE];

void SSD1306_Init(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool generate_stop_condition);
void SSD1306_Send_Command_Data(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* command_buffer, uint16_t data_to_send_size /*bool is_cmd_announce_byte_sent, bool generate_stop_condition*/);
void SSD1306_Disp_On(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition);
void SSD1306_Disp_Off(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition);
void SSD1306_Light_All_Pixs(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition);  // After this command SSD1306_Display_GDDRAM needs to be called if one wishes to display the data stored in the GDDRAM again
void SSD1306_Send_GDDRAM_data(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, uint16_t data_to_send_size /*bool is_cmd_announce_byte_sent, bool generate_stop_condition*/); // Image data
void SSD1306_Display_GDDRAM(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, bool is_cmd_announce_byte_sent, bool generate_stop_condition);  // When the GDDRAM is not cleared first, it can contain noise! (pixels that still light up but were not set to '1' by the programmer)
void SSD1306_Clear_GDDRAM(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, pixel_state_t pix_state /* bool is_cmd_announce_byte_sent, bool generate_stop_condition */);
void SSD1306_Set_Page_Address(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, Page_Address page_address); // Page0   - Page7
void SSD1306_Set_Column_Address(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, Page_Address page_address, Column_Address column_address); // Column0 - Column127

void Draw_Pacman(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, const uint8_t* pacman_data, uint16_t data_to_send_size /*bool is_cmd_announce_byte_sent, bool generate_stop_condition*/);
void Draw_String_8_x_8_Char(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, const uint8_t* string, uint16_t data_to_send_size); // !!!! NOT IMPLEMENTED !!!!
void Draw_All_Single_8_x_8_Digits(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer);

// TO DO: CHANGE NAME TO Float_2Decimals_To_Byte_Arr()
bool Float_To_String_2Decimals(float float_num, uint8_t float_string[static 6], size_t float_str_len); // Expects a uint8_t string buffer of at least 6 bytes => COMPILER WILL NOT THROW AN ERROR IF NOT RESPECTED => the static keyword makes the compiler throw a warning
// void Write_Float_String_To_GDDRAM_Buffer(uint8_t* gddram_buffer, uint16_t* buffer_start_index, uint8_t float_string[4]);
void Write_Float_String_To_GDDRAM_Buffer(uint8_t* gddram_buffer, uint16_t* buffer_start_index, uint8_t* float_string, uint8_t float_str_len);
bool Draw_Ultrasonic_Sensor_Distance(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, float distance);
bool Draw_Motor_Duty_Cycle(cyhal_i2c_t* i2c_master_obj, uint8_t slave_addr, uint8_t* gddram_buffer, float distance);

#endif /* SSD1306_H_ */
