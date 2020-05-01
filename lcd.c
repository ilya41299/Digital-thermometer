#include "lcd.h"
#define SLAVE_ADDRES_LCD 0x40 // адрес дисплея
//------------------------------------------------
uint8_t buf[1] = { 0 };
extern I2C_HandleTypeDef hi2c1; // указатель на шину I2C
uint8_t portlcd = 0; //ячейка для хранения данных порта микросхемы расширения
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
    micros *= (SystemCoreClock / 1000000) / 5;
    while (micros--)
        ;
}
//------------------------------------------------
void LCD_WriteByteI2CLCD(uint8_t bt)
{
    buf[0] = bt;
    // стандартная функция библиотеки HAL ( указатель на шину I2C, адрес, буфер, размер, задержка)
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_ADDRES_LCD, buf, 1, 1000);
}
//------------------------------------------------
void sendhalfbyte(uint8_t c)
{
    c <<= 4; // сдвигаем байт на 4 влево
    e_set(); // включаем линию E
    DelayMicro(50); // ожидаем 45-50 мкс
    LCD_WriteByteI2CLCD(portlcd | c); // посылаем сдвинутый байт
    e_reset(); // выключаем линию E
    DelayMicro(50); // ожидаем 45-50 мкс
}
//------------------------------------------------
void sendbyte(uint8_t c, uint8_t mode) // функция передачи байта.
{
    if (mode == 0)
        rs_reset(); // передача данных
    else
        rs_set(); // инaче - передача команды
    uint8_t hc = 0;
    hc = c >> 4;
    sendhalfbyte(hc); // передача старщей тетрады байта
    sendhalfbyte(c); // передача младшей тетрады байта
}
//------------------------------------------------
void LCD_Clear(void)
{
    sendbyte(0x01, 0); // подача команды очистки дисплея
    HAL_Delay(2);
}
//------------------------------------------------
void LCD_SendChar(char ch)
{
    sendbyte(ch, 1); // передача символа
}
//------------------------------------------------
void LCD_String(char* st)
{
    uint8_t i = 0;
    while (st[i] != 0) {
        sendbyte(st[i], 1);
        i++;
    }
}
//------------------------------------------------
void LCD_SetPos(uint8_t x, uint8_t y)
{
    switch (y) {
    case 0:
        sendbyte(x | 0x80, 0); // 0x80 - бит обращения к DDRAM
        HAL_Delay(1);
        break;
    case 1:
        sendbyte((0x40 + x) | 0x80, 0); //
        HAL_Delay(1);
        break;
    }
}
//------------------------------------------------
void LCD_ini(void)
{
    HAL_Delay(15); //
    sendhalfbyte(0x03);
    HAL_Delay(5);
    sendhalfbyte(0x03);
    DelayMicro(100);
    sendhalfbyte(0x02);
    HAL_Delay(1);
    // далее 4-битный режим
    sendbyte(0x28, 0); // 4бит-режим (DL=0) и 2 линии (N=1)
    HAL_Delay(1);
    sendbyte(0x08, 0); // выключение дисплея
    HAL_Delay(1);
    sendbyte(0x01, 0); // очистка дисплея
    HAL_Delay(1);
    sendbyte(0x06, 0); //курсор (хоть он у нас и невидимый) будет двигаться влево
    HAL_Delay(1);
    sendbyte(0x0C, 0); // дисплей включаем (D=1)
    HAL_Delay(1);
    setled(); // включаем подсветку
    setwrite(); // переводим в режим записи
}
