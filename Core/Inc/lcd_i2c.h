#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include "stm32f4xx_hal.h"

// Funkcje obsługi wyświetlacza
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendString(char *str);
void LCD_SendInt(int number);
void LCD_SendFloat(float number, int decimal_places);

#endif /* INC_LCD_I2C_H_ */
