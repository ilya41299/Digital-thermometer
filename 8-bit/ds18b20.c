#include "ds18b20.h"
//--------------------------------------------------
uint8_t LastDeviceFlag;
uint8_t LastDiscrepancy; // последнее несовпадение
uint8_t ROM_NO[8]; // ROM-код текущего датчика
extern uint8_t Dev_ID[8][8]; // массив серийных номеров датчиков
extern uint8_t Dev_Cnt; // число датчиков на шине
//--------------------------------------------------
// функция задержки
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
    micros *= (SystemCoreClock / 1000000) / 9;
    while (micros--)
        ;
}
//--------------------------------------------------
// инициализация 11-го порта для взаимодействия с датчиками
void port_init(void)
{
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11); // разинициализация порта
    GPIOB->CRH |= GPIO_CRH_MODE11; // установка режима выхода 50 МГц
    GPIOB->CRH |= GPIO_CRH_CNF11_0; // устанавливаем режим открытого коллектрора
    GPIOB->CRH &= ~GPIO_CRH_CNF11_1; // иначе говоря open-drain
}
//--------------------------------------------------
uint8_t ds18b20_Reset(void)
{
    uint16_t status;
    GPIOB->ODR &= ~GPIO_ODR_ODR11; // установка низкого уровня
    DelayMicro(485); // задержка как минимум на 480 микросекунд
    GPIOB->ODR |= GPIO_ODR_ODR11; // установка высокого уровня
    DelayMicro(65); // задержка как минимум на 60 микросекунд
    status = GPIOB->IDR & GPIO_IDR_IDR11; // проверяем уровень
    DelayMicro(500); // задержка как минимум на 480 микросекунд
    //(на всякий случай подождём побольше, так как могут быть неточности в задержке)
    return (status ? 1 : 0); // вернём результат
}
//-----------------------------------------------
uint8_t ds18b20_ReadBit(void)
{
    uint8_t bit = 0;
    GPIOB->ODR &= ~GPIO_ODR_ODR11; // устанавливаем низкий уровень
    DelayMicro(2); // ждём 2 мкс
    GPIOB->ODR |= GPIO_ODR_ODR11; // устанавливаем высокий уровень
    DelayMicro(13); // ждём оставшиеся 13 мкс из 15 мкс
    // из регистра IDR считываем сигнал с шины
    bit = (GPIOB->IDR & GPIO_IDR_IDR11 ? 1 : 0); // проверяем уровень порта
    DelayMicro(45); // небольшая задержка перед подачей новой команды
    return bit;
}
//-----------------------------------------------
uint8_t ds18b20_ReadByte(void)
{
    uint8_t data = 0; // байт для приёма данных
    for (uint8_t i = 0; i <= 7; i++)
        data += ds18b20_ReadBit() << i; // побитно считываем данные из датчика
    return data;
}
//-----------------------------------------------
void ds18b20_WriteBit(uint8_t bit)
{
    GPIOB->ODR &= ~GPIO_ODR_ODR11; // устанавливаем низкий уровень
    DelayMicro(bit ? 3 : 65); // в зависимости от записываемого бита выбираем задержку
    GPIOB->ODR |= GPIO_ODR_ODR11; // устанавливаем высокий уровень
    DelayMicro(bit ? 65 : 3); // в зависимости от записываемого бита выбираем задержку
}
//-----------------------------------------------
void ds18b20_WriteByte(uint8_t dt)
{
    for (uint8_t i = 0; i < 8; i++) {
        ds18b20_WriteBit(dt >> i & 1); // побитово подаём на запись байт
        // небольшая задержка
        DelayMicro(5);
    }
}
//-----------------------------------------------
// считывание всех ROM-кодов
uint8_t ds18b20_SearhRom(uint8_t* Addr)
{
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number, search_result;
    uint8_t id_bit, cmp_id_bit;
    uint8_t rom_byte_mask, search_direction;
    //проинициализируем переменные
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1; // маска для побитового сдвига при чтении битов
    search_result = 0;
    if (!LastDeviceFlag) // если не все УИ считаны
    {
        ds18b20_Reset(); // перезагружаем датчики
        ds18b20_WriteByte(0xF0); // посылаем команду на чтение ROM-кодов
    }
    do {
        id_bit = ds18b20_ReadBit(); // читаем бит идентификатора
        cmp_id_bit = ds18b20_ReadBit(); // читаем бит совместности
        if ((id_bit == 1) && (cmp_id_bit == 1)) // невозможный случай
            break;
        else {
            if (id_bit != cmp_id_bit) // если пришёл корректный сигнал
                // задаём соответствующий считанный бит
                search_direction = id_bit; // 01 или 10, соот-но общий 0 или 1
            else // пришёл 00 - конфликт
            {
                // если это несовпадение не новое (не последнее)
                if (id_bit_number < LastDiscrepancy)
                    // то копируем результат из предыдущего прохода
                    search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                else
                    search_direction = (id_bit_number == LastDiscrepancy);
                if (search_direction == 0) {
                    last_zero = id_bit_number;
                }
            }
            if (search_direction == 1) // записываем по маске рассматриваемый бит
                ROM_NO[rom_byte_number] |= rom_byte_mask; // единицу
            else
                ROM_NO[rom_byte_number] &= ~rom_byte_mask; // или ноль
            ds18b20_WriteBit(search_direction); // посылаем сигнал совпадающего бита
            id_bit_number++; // итерируем счетчик битов
            rom_byte_mask <<= 1; // сдвигаем маску
            if (rom_byte_mask == 0) // если мы считали байт ROM-кода
            {
                rom_byte_number++; // переходим к следующему байту
                rom_byte_mask = 1; // обновляем маску
            }
        }
    } while (rom_byte_number < 8); // считываем байты с 0 до 7 в цикле

    if (!(id_bit_number < 65)) {
        // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
        LastDiscrepancy = last_zero;
        // check for last device
        if (LastDiscrepancy == 0)
            LastDeviceFlag = 1;
        search_result = 1;
    }
    if (!search_result || !ROM_NO[0]) // обнуляем переменные
    {
        LastDiscrepancy = 0;
        LastDeviceFlag = 0;
        search_result = 0;
    }
    else {
        for (int i = 0; i < 8; i++)
            Addr[i] = ROM_NO[i];
    }
    return search_result; // возвращаем бит, удачно ли прошло чтение
}
//-----------------------------------------------
uint8_t ds18b20_init(uint8_t mode)
{
    int i = 0, j = 0;
    uint8_t dt[8]; // код одного датчика
    if (mode == SKIP_ROM) {
        if (ds18b20_Reset())
            return 1;
        //SKIP ROM
        ds18b20_WriteByte(0xCC);
        //WRITE SCRATCHPAD
        ds18b20_WriteByte(0x4E);
        //TH REGISTER 125 градусов 0111 1101
        ds18b20_WriteByte(0x7D);
        //TL REGISTER - 55 градусов 1011 0111
        ds18b20_WriteByte(0xB7);
        //Resolution 12 bit
        ds18b20_WriteByte(RESOLUTION_12BIT);
    }
    else {
        for (i = 1; i <= 8; i++) {
            if (ds18b20_SearhRom(dt)) // поиск ROM-кода
            {
                Dev_Cnt++; // инкрементируем число датчиков
                memcpy(Dev_ID[Dev_Cnt - 1], dt, sizeof(dt)); // сохраняем найденный ROM
            }
            else
                break;
        }
        for (i = 1; i <= Dev_Cnt; i++) {
            if (ds18b20_Reset())
                return 1;
            //Match Rom
            ds18b20_WriteByte(0x55);
            for (j = 0; j <= 7; j++) {
                ds18b20_WriteByte(Dev_ID[i - 1][j]);
            }
            //WRITE SCRATCHPAD
            ds18b20_WriteByte(0x4E);
            //TH REGISTER 125 градусов 0111 1101
            ds18b20_WriteByte(0x7D);
            //TL REGISTER - 55 градусов 1011 0111
            ds18b20_WriteByte(0xB7);
            //Resolution 12 bit
            ds18b20_WriteByte(RESOLUTION_12BIT);
        }
    }
    return 0;
}
//----------------------------------------------------------
void ds18b20_MeasureTemperCmd(uint8_t mode, uint8_t DevNum)
{
    int i = 0;
    ds18b20_Reset(); // перезагружаем датчики
    if (mode == SKIP_ROM) // если датчик один
    {
        //SKIP ROM
        ds18b20_WriteByte(0xCC);
    }
    else {
        //Match Rom
        ds18b20_WriteByte(0x55); // команда совпадения адреса
        for (i = 0; i <= 7; i++) // подаём ROM-код очередного датчика
        {
            ds18b20_WriteByte(Dev_ID[DevNum][i]);
        }
    }
    //CONVERT T
    ds18b20_WriteByte(0x44); // команда записи температуры в 2 байтный регистр
}
//----------------------------------------------------------
void ds18b20_ReadStratcpad(uint8_t mode, uint8_t* Data, uint8_t DevNum)
{
    uint8_t i;
    ds18b20_Reset(); // перезагружаем датчики
    if (mode == SKIP_ROM) {
        //SKIP ROM
        ds18b20_WriteByte(0xCC);
    }
    else {
        //Match Rom
        ds18b20_WriteByte(0x55); // оставляем активным один датчик
        for (i = 0; i < 8; i++) {
            ds18b20_WriteByte(Dev_ID[DevNum][i]);
        }
    }
    //READ SCRATCHPAD
    ds18b20_WriteByte(0xBE); // подаем команду на чтение температуры из регистров
    for (i = 0; i < 8; i++) {
        Data[i] = ds18b20_ReadByte(); // считываем данные
    }
}
//----------------------------------------------------------
uint8_t ds18b20_GetSign(uint16_t dt)
{
    //Проверим 11-й бит
    if (dt & (1 << 11))
        return 1;
    else
        return 0;
}
//----------------------------------------------------------
uint8_t ds18b20_Convert(uint16_t dt)
{
    uint8_t t;
    t = (uint8_t)((dt & 0x07FF) >> 4); //отборосим знаковые и дробные биты

    //Прибавим дробную часть
    // t += (float)(dt&0x000F) / 16.0f;
    return t;
}
//----------------------------------------------------------
