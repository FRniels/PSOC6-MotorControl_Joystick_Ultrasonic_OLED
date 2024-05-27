#ifndef SSD1306_TYPES_H_
#define SSD1306_TYPES_H_

typedef enum SSD1306_Drawing_state_t
{
	NOT_DRAWING, IS_DRAWING
} SSD1306_Drawing_state_t;

typedef enum pixel_state_t
{
	PIXEL_OFF = 0, PIXEL_ON = 1
} pixel_state_t;

typedef struct Page_Address
{
	uint8_t start;
	uint8_t end;
} Page_Address;

typedef struct Column_Address
{
	uint8_t start;
	uint8_t end;
} Column_Address;

#endif /* SSD1306_TYPES_H_ */
