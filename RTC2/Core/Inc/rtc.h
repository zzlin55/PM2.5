/*
 * rtc.h
 *
 * Author: Clifford Zhan
 * Date: 6.12.2019
 * Version: v1.0
 *
 * Description: Get Real Time Clock Data.
 *
 * Instruction:
 * 1. Change RTC slave address in RTC_ADDR, Change register address in RTC_READ_ADDR.
 * 2. Set corresponded I2C by changing I2CX
 * 3. set number of register to read from RTC (Usually 7)
 * 3. Enable I2C peripheral.
 */
#include "main.h"
#define RTC_ADDR 0x32<<1
#define RTC_READ_ADDR 0x00
#define I2CX hi2c1
#define RTC_REG_NUM 7

I2C_HandleTypeDef I2CX;

uint8_t DecToBcd(uint8_t value);
uint8_t BcdToDec(uint8_t val);


typedef struct {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t week;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
}rtc_HandleTypeDef;


rtc_HandleTypeDef ReadRTC();
void GetRawRTC(I2C_HandleTypeDef *i2c,uint8_t* rawdata);
void WriteRTC(	uint8_t second,
				uint8_t minute,
				uint8_t hour,
				uint8_t week,
				uint8_t day,
				uint8_t month,
				uint16_t year);
void WriteRTCEnable();
void WriteRTCDisable();
