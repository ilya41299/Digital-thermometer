#ifndef LCD_H_
#define LCD_H_
#include "stm32f1xx_hal.h"
#include <stdint.h>
// обозначим LCDPORT порт А
#define LCDPORT GPIOA
// выбор регистра данных или команд
#define RS ((uint16_t)(1 << 8))
// активация синхронизации
#define EN ((uint16_t)(1 << 9))
// передача данных - 1
#define DATA 1
// передача команд - 0
#define COM 0

// функция вывода символа и отправки команд
void LCD(uint8_t RS_level, uint8_t byte);
// функция вывода строки
void LCD_STRING(const char* message);
// настройка порта А
void lcd_ini(void);
#endif /* LCD_H_ */
