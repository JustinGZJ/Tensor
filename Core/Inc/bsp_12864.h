//
// Created by justin on 2021/12/2.
//

#ifndef TENSOR_BSP_12864_H
#define TENSOR_BSP_12864_H
#include "i2c.h"
#define I2C_X           hi2c1
#define OLED_ADDR       0x78
#define CMD_CTRL        0X00
#define DATA_CTRL       0X40
void OLED_Write_Cmd(uint8_t cmd);
void OLED_Write_Data(uint8_t data);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Clear(void);
void OLED_Init(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_Show_String(uint8_t x, uint8_t y, uint8_t *ch);
//void OLED_Show_CN(uint8_t x, uint8_t y, uint8_t n);
#endif //TENSOR_BSP_12864_H
