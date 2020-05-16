#include "lcd.h"

void LCD(uint8_t RS_level, uint8_t byte)
{
    if (RS_level == COM)
    {
        LCDPORT->ODR &= ~(RS);
    }
    else if (RS_level == DATA)
    {
        LCDPORT->ODR |= RS;
    }
    LCDPORT->ODR &= ~((uint16_t)0x00FF); // установка нуля
    LCDPORT->ODR |= (int16_t)byte; // выставляем поданный байт
    HAL_Delay(1);
    LCDPORT->ODR |= EN; // включаем синхронизацию
    HAL_Delay(1);
    LCDPORT->ODR &= ~(EN); // выключаем синхронизацию
    HAL_Delay(1);
}

void LCD_STRING(const char* message)
{
    uint8_t i = 0;
    while ((i < 20) & (message[i] != 0))
    {
        LCD(DATA, message[i]);
        i++;
    }
}
void lcd_ini()
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // включаем тактирование
    // настраеваем конфиг.регистры порта на вывод данных
    GPIOA->CRL |= (GPIO_CRL_MODE0_1 | 
									 GPIO_CRL_MODE1_1 | 
									 GPIO_CRL_MODE2_1 | 
									 GPIO_CRL_MODE3_1 | 
	                 GPIO_CRL_MODE4_1 | 
	                 GPIO_CRL_MODE5_1 | 
	                 GPIO_CRL_MODE6_1 | 
	                 GPIO_CRL_MODE7_1);
    GPIOA->CRH |= (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1);
    // установка 8-битного режима
    LCD(COM, 0x30);
    HAL_Delay(5);
    LCD(COM, 0x30);
    HAL_Delay(1);
    LCD(COM, 0x30);
    HAL_Delay(1);
    LCD(COM, 0x3C); // установка параметров интерфейса (4 строки)
    HAL_Delay(2);
    LCD(COM, 0x0C); // включаем дисплей без курсора
    HAL_Delay(2);
    LCD(COM, 0x01); // очищаем его
    HAL_Delay(2);
    LCD(COM, 0x80); // устанавливаем в начало первой строки
    HAL_Delay(2);
}
