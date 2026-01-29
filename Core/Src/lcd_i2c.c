#include "lcd_i2c.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1; // Używamy Twojego I2C1 (PB6/PB7)

// Adres I2C wyświetlacza.
// Najczęstsze to 0x27 (<< 1 = 0x4E) lub 0x3F (<< 1 = 0x7E)
// Jeśli nie zadziała, spróbuj zmienić 0x27 na 0x3F
#define LCD_ADDR (0x27 << 1)

void LCD_Write_Nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = nibble << 4;
    data |= rs; // RS bit
    data |= 0x08; // Backlight ON (D3 bit na PCF8574)

    uint8_t packet[1];

    // Enable Pulse
    packet[0] = data | 0x04; // En = 1
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, packet, 1, 10);
    HAL_Delay(1);

    packet[0] = data; // En = 0
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, packet, 1, 10);
}

void LCD_SendCmd(uint8_t cmd) {
    uint8_t upper = (cmd >> 4) & 0x0F;
    uint8_t lower = cmd & 0x0F;
    LCD_Write_Nibble(upper, 0);
    LCD_Write_Nibble(lower, 0);
}

void LCD_SendData(uint8_t data) {
    uint8_t upper = (data >> 4) & 0x0F;
    uint8_t lower = data & 0x0F;
    LCD_Write_Nibble(upper, 1);
    LCD_Write_Nibble(lower, 1);
}

void LCD_Init(void) {
    HAL_Delay(50);
    // Inicjalizacja 4-bitowa (magiczna sekwencja HD44780)
    LCD_Write_Nibble(0x03, 0);
    HAL_Delay(5);
    LCD_Write_Nibble(0x03, 0);
    HAL_Delay(1);
    LCD_Write_Nibble(0x03, 0);
    LCD_Write_Nibble(0x02, 0); // Przejście w tryb 4-bit

    // Konfiguracja
    LCD_SendCmd(0x28); // Function set: 4-bit, 2 line, 5x8 dots
    LCD_SendCmd(0x0C); // Display on, Cursor off
    LCD_SendCmd(0x06); // Entry mode: Increment cursor
    LCD_Clear();
}

void LCD_Clear(void) {
    LCD_SendCmd(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    addr += col;
    LCD_SendCmd(addr);
}

void LCD_SendString(char *str) {
    while (*str) LCD_SendData(*str++);
}

void LCD_SendInt(int number) {
	char buffer[16];
	sprintf(buffer, "%d", number);
	LCD_SendString(buffer);
}

void LCD_SendFloat(float number, int decimal_places) {
    char buffer[16];
    // Prosta obsługa float (jeśli sprintf %f jest wyłączone w IDE)
    int int_part = (int)number;
    int frac_part = (int)((number - int_part) * (decimal_places == 1 ? 10 : 100));
    if(frac_part < 0) frac_part *= -1;

    sprintf(buffer, "%d.%0*d", int_part, decimal_places, frac_part);
    LCD_SendString(buffer);
}
