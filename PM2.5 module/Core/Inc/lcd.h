/*
 * lcd.h
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f4xx_hal.h"
#include "string.h"
#include "main.h"

// #define LCD20xN 		// For 20xN LCDs
#define LCD16xN			// For 16xN LCDs

// For row start addresses
extern const uint8_t ROW_16[];
extern const uint8_t ROW_20[];
/************************************** GPIO Pin Define **************************************/
//see pin mapping of 401RE.
//D4: PB5
#define D4_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_5
//D5: PB4
#define D5_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_4
//D6: PB10
#define D6_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_10
//D7:PA8
#define D7_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_8
//RS->D8: PA9
#define RS_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_9
//EN->D9:PC7
#define EN_GPIO_Port GPIOC
#define EN_Pin GPIO_PIN_7
//Backlight->D10: PB6
#define BL_GPIO_Port GPIOB
#define BL_Pin GPIO_PIN_6

/************************************** Command register **************************************/
#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x04
#define OPT_S	0x01					// Shift entire display to right
#define OPT_INC 0x02					// Cursor increment

#define DISPLAY_ON_OFF_CONTROL 0x08
#define OPT_D	0x04					// Turn on display
#define OPT_C	0x02					// Turn on cursor
#define OPT_B 	0x01					// Turn on cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
#define OPT_SC 0x08
#define OPT_RL 0x04

#define FUNCTION_SET 0x20
#define OPT_DL 0x10						// Set interface data length
#define OPT_N 0x08						// Set number of display lines
#define OPT_F 0x04						// Set alternate font

#define SET_DDRAM_ADDR 0x80				// Set DDRAM address


/************************************** Helper macros **************************************/
#define DELAY(X) HAL_Delay(X)


/************************************** LCD defines **************************************/
#define LCD_NIB 4
#define LCD_BYTE 8
#define LCD_DATA_REG 1
#define LCD_COMMAND_REG 0


/************************************** LCD typedefs **************************************/
#define Lcd_PortType GPIO_TypeDef*
#define Lcd_PinType uint16_t

typedef enum {
	LCD_4_BIT_MODE,
	LCD_8_BIT_MODE
} Lcd_ModeTypeDef;


typedef struct {
	Lcd_PortType * data_port;
	Lcd_PinType * data_pin;

	Lcd_PortType rs_port;
	Lcd_PinType rs_pin;

	Lcd_PortType en_port;
	Lcd_PinType en_pin;

	Lcd_ModeTypeDef mode;

} Lcd_HandleTypeDef;


/************************************** Public functions **************************************/
void Lcd_mode_select(Lcd_HandleTypeDef * lcd);
void Lcd_int(Lcd_HandleTypeDef * lcd, int number);
void Lcd_string(Lcd_HandleTypeDef * lcd, char * string);
void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col);
Lcd_HandleTypeDef Lcd_create(
		Lcd_PortType port[], Lcd_PinType pin[],
		Lcd_PortType rs_port, Lcd_PinType rs_pin,
		Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode);



#endif /* LCD_H_ */
