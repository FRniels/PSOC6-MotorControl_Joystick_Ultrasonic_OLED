#ifndef SSD1306_SETTINGS_H_
#define SSD1306_SETTINGS_H_

#define I2C_MASTER_FREQUENCY           400000u // I²C fast mode: 400KHz
#define SDD1306_128_x_64_GDDRAM_SIZE   1024
#define FRAME_BUFFER_SIZE              (uint16_t)(1 + SDD1306_128_x_64_GDDRAM_SIZE) // 1025bytes: 1 byte ANNOUNCE_GDDRAM_DATA command followed by 1024bytes pixel data

#endif /* SSD1306_SETTINGS_H_ */
