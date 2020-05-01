
#ifndef LCD_H_
#define LCD_H_
//------------------------------------------------
#include "stm32f1xx_hal.h"
//------------------------------------------------
#define e_set() LCD_WriteByteI2CLCD(portlcd |= 0x04) // установка линии Е в 1
#define e_reset() LCD_WriteByteI2CLCD(portlcd &= ~0x04) // установка линии Е в 0
#define rs_set() LCD_WriteByteI2CLCD(portlcd |= 0x01) // установка линии RS в 1
#define rs_reset() LCD_WriteByteI2CLCD(portlcd &= ~0x01) // установка линии RS в 0
#define setled() LCD_WriteByteI2CLCD(portlcd |= 0x08) // установка линии P3 в 1
#define setwrite() LCD_WriteByteI2CLCD(portlcd &= ~0x02) // установка линии RW в 0
//------------------------------------------------
void LCD_ini(void); // инициализация дисплея
void LCD_Clear(void); // очистка дисплея
void LCD_SendChar(char ch); // вывод символа
void LCD_String(char* st); // вывод строки
void LCD_SetPos(uint8_t x, uint8_t y); // задать позицию для вывода
//------------------------------------------------
#endif /* LCD_H_ */


Format!Style:
C++ online code formatter © 2014 by KrzaQ

Powered by vibe.d, the D language and clang-format
