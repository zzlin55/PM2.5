/*
 * RTC API:
 *
 * rtc_HandleTypeDef ReadRTC()
 *
 * WriteRTC(uint8_t year,
 * 			uint8_t month,
 * 			uint8_t day,
 * 			uint8_t week,
 * 			uint8_t hour,
 * 			uint8_t minute,
 * 			uint8_t second)
 */
#include "rtc.h"
#include "main.h"
#include "retarget.h"
#include <stdio.h>


/* Decimal to BCD*/
uint8_t DecToBcd(uint8_t val){
	return ((val/10 *16)+(val%10));
}

/* BCD to Decimal*/
uint8_t BcdToDec(uint8_t val)
{
  return( (val/16*10) + (val%16) );
}

/*
 * Read Data from RTC
 */
rtc_HandleTypeDef ReadRTC()
{
	rtc_HandleTypeDef rtc;
	uint8_t raw_data_from_rtc[7];
	GetRawRTC(&I2CX,raw_data_from_rtc);
	rtc.second = BcdToDec(raw_data_from_rtc[0]);
	rtc.minute = BcdToDec(raw_data_from_rtc[1]);
	raw_data_from_rtc[2] = (raw_data_from_rtc[2] & 0x7f);
	raw_data_from_rtc[2] = (((raw_data_from_rtc[2] & 0xf0) >> 4) * 10) + (raw_data_from_rtc[2] & 0x0f);
	rtc.hour = raw_data_from_rtc[2];
	rtc.week = BcdToDec(raw_data_from_rtc[3])+1;//add 1 since week range from 0-6
	rtc.day = BcdToDec(raw_data_from_rtc[4]);
	rtc.month = BcdToDec(raw_data_from_rtc[5]);
	rtc.year = (uint16_t) BcdToDec(raw_data_from_rtc[6]);
	rtc.year += 2000;
	/*
	if(DebugLevel>=DEBUG_MODULE){
		printf("RTC processed data:\n\r");
		printf("Second: %d\r\n",rtc.second);
		printf("Minutes: %d\r\n",rtc.minute);
		printf("Hour: %d\r\n",rtc.hour);
		printf("Week: %d\r\n",rtc.week);
		printf("Day: %d\r\n",rtc.day);
		printf("Month: %d\r\n",rtc.month);
		printf("Year: %d\r\n",rtc.year);
	}
	*/
	//Return rtc handler
	return rtc;
}
/*
 * Receive Raw Data from RTC.
 */
void GetRawRTC(I2C_HandleTypeDef *i2c,uint8_t* rawdata)
{
	HAL_StatusTypeDef I2Cflag;
	I2Cflag = HAL_I2C_Master_Transmit(i2c, RTC_ADDR, RTC_READ_ADDR, 1, HAL_MAX_DELAY);
	if (I2Cflag!=HAL_OK)
	{
		//if(DebugLevel>=DEBUG_MODULE){
		//	printf("RTC Module Writing Error\n\r");
		//}
	}
	I2Cflag = HAL_I2C_Master_Receive(i2c, RTC_ADDR, rawdata, RTC_REG_NUM, HAL_MAX_DELAY);
	if (I2Cflag!=HAL_OK)
	{
		//if(DebugLevel>=DEBUG_MODULE){
		//	printf("RTC Module Reading Error\n\r");
		//}
	}
	else
	{
		/*
		if(DebugLevel>=INFO){
			uint8_t i;
			printf("The Raw Data From RTC is:\r\n");
			for (i=0;i<RTC_REG_NUM;i++){
				printf("Raw:%x\r\n",*(rawdata+i));
			}
			printf("\r\n");
		}
		*/
	}
}
/**
  * @brief USART2 Initialization Function
  * @param second,minute,hour,week,day,mont,hyear
  * @retval None
  */
void WriteRTC(	uint8_t second,
				uint8_t minute,
				uint8_t hour,
				uint8_t week,
				uint8_t day,
				uint8_t month,
				uint16_t year)
{
	uint8_t writedata[RTC_REG_NUM+1];
	HAL_StatusTypeDef I2Cflag;

	if (year>=2000)
	{
		year = (uint8_t)(year -2000);
	}
	hour = hour+80; //set 24 hours format.
	writedata[0] = 0x00;
	writedata[1] = DecToBcd(second);
	writedata[2] = DecToBcd(minute);
	writedata[3] = DecToBcd(hour);
	writedata[4] = DecToBcd(week-1);
	writedata[5] = DecToBcd(day);
	writedata[6] = DecToBcd(month);
	writedata[7] = DecToBcd(year);

	WriteRTCEnable();
	I2Cflag = HAL_I2C_Master_Transmit(&I2CX, RTC_ADDR, writedata, RTC_REG_NUM+1, HAL_MAX_DELAY);
	if (I2Cflag!=HAL_OK)
	{
		//if(DebugLevel>=DEBUG_MODULE){
		//	printf("RTC Module Writing Error\n\r");
		//}
	}
	WriteRTCDisable();
}
/*
 * Write RTC enable bit WRTC1 ,WRTC2,WRTC3.
 * address:
 * 		WRTC1: 0x10 bit 7
 * 		WRTC2: 0x0f bit 2
 * 		WRTC3: 0x0f bit 7
 * To Enable RTC writing:
 * 		Set WRTC1 to 1 first,then set WRTC2 and WRTC3 to 1
 * To Disable RTC writing:
 * 		Set WRTC2 and WRTC3 to 0 first, then set WRTC1 to 0
 */
void WriteRTCEnable()
{
	HAL_StatusTypeDef I2Cflag;
	uint8_t WRTC1_ADDR = 0x10, WRTC23_ADDR = 0x0f;
	uint8_t WRTC1_SET[2] ={WRTC1_ADDR,0x80};
	uint8_t WRTC23_SET[2] = {WRTC23_ADDR,0x84};
	I2Cflag = HAL_I2C_Master_Transmit(&I2CX, RTC_ADDR, WRTC1_SET, 2, HAL_MAX_DELAY);
	I2Cflag = HAL_I2C_Master_Transmit(&I2CX, RTC_ADDR, WRTC23_SET, 2, HAL_MAX_DELAY);
	if (I2Cflag!=HAL_OK)
	{
		//if(DebugLevel>=DEBUG_MODULE){
		//	printf("RTC Module Writing Error\n\r");
		//}
	}
}

/*
 * Write RTC Disable
 */
void WriteRTCDisable()
{
	HAL_StatusTypeDef I2Cflag;
	uint8_t WRTC1_ADDR = 0x10, WRTC23_ADDR = 0x0f;
	uint8_t WRTC1_SET[2] ={WRTC1_ADDR,0x00};
	uint8_t WRTC23_SET[2] = {WRTC23_ADDR,0x00};
	I2Cflag = HAL_I2C_Master_Transmit(&I2CX, RTC_ADDR, WRTC1_SET, 2, HAL_MAX_DELAY);
	I2Cflag = HAL_I2C_Master_Transmit(&I2CX, RTC_ADDR, WRTC23_SET, 2, HAL_MAX_DELAY);
	if (I2Cflag!=HAL_OK)
	{
		//if(DebugLevel>=DEBUG_MODULE){
		//	printf("RTC Module Writing Error\n\r");
		//}
	}
}
