#include "PMmodule.h"

uint8_t PMchecksum(uint8_t *buf,uint8_t buflen)
{
	//uint8_t buflen = strlen(buf);
	uint8_t rxerror = 1;
	uint16_t checksum = 0;
	uint8_t i;
	for (i=0;i<buflen;i++){
		checksum = checksum+buf[i];
	}
	if (buflen ==31){
		checksum = checksum+0x42; //uncomment if buf does not add 0x42
	}
	if (checksum==((buf[buflen-2]<<8)+buf[buflen-1]))
	{
		rxerror = 0;
		return rxerror;
	}
	else
	{
		printf("PM serial reading Checksum Error");
		return rxerror;
	}
	return rxerror;
}

uint16_t transmitPM01(uint8_t *thebuf)
{
	uint16_t PM01Val;
	PM01Val=((thebuf[4]<<8) + thebuf[5]); //count PM1.0 value of the air detector module
	return PM01Val;
}

//transmit PM Value to PC
uint16_t transmitPM2_5(uint8_t *thebuf)
{
	uint16_t PM2_5Val;
    PM2_5Val=((thebuf[6]<<8) + thebuf[7]);//count PM2.5 value of the air detector module
	return PM2_5Val;
}

//transmit PM Value to PC
uint16_t transmitPM10(uint8_t *thebuf)
{
	uint16_t PM10Val;
    PM10Val=((thebuf[8]<<8) + thebuf[9]); //count PM10 value of the air detector module
	return PM10Val;
}
