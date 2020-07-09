#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "pca9685.h"

void setupAI(pca9685_t& pca9685)
{
	const int I2C_PORT = pca9685.i2c_port;
	int mode1;

	mode1 = wiringPiI2CReadReg8(I2C_PORT, MODE1)& 0xFF; // I2C_PORT에 있는 MODE1 레지스터를 8bit 값으로 읽음
	wiringPiI2CWriteReg8(I2C_PORT, MODE1, mode1|AI); // I2C_PORT에 있는 MODE1 레지스터의 AI비트를 1로 설정
}

void setFreq(pca9685_t& pca9685, int frequency)
{
	const int I2C_PORT = pca9685.i2c_port;
	int mode1;
	int prescale; //분 주기

	mode1 = wiringPiI2CReadReg8(I2C_PORT, MODE1)& 0xFF; // I2C_PORT에 있는 MODE1 레지스터를 8bit 값으로 읽음
	wiringPiI2CWriteReg8(I2C_PORT, MODE1, mode1|SLEEP); // I2C_PORT에 있는 MODE1 레지스터의 SLEEP비트를 1로 설정 (오실레이터가 멈춘다)

	prescale = (int)(25000000.0f/(4096*frequency)); // prescale 공식 ( 오실레이터가 멈춰있을때(SLEEP이 1일 때) 설정 가능하다)
	wiringPiI2CWriteReg8(I2C_PORT, PRE_SCALE, prescale); // PRE_SCALE 레지스터에 분주기 값을 쓴다.

	mode1 = wiringPiI2CReadReg8(I2C_PORT, MODE1)& 0xFF;
	wiringPiI2CWriteReg8(I2C_PORT, MODE1, mode1&~SLEEP); // I2C_PORT에 있는 MODE1 레지스터의 SLEEP비트를 0으로 설정 (오실레이터 구동)

	delay(1); //잠깐 쉬어주고

	mode1 = wiringPiI2CReadReg8(I2C_PORT, MODE1)& 0xFF;
	wiringPiI2CWriteReg8(I2C_PORT, MODE1, mode1|RESTART); // 모든 PWM 채널 재시작 (출력 수행 가능)
}

void setDuty(pca9685_t& pca9685, const int pin, const int duty_cycle)
{
	const int I2C_PORT = pca9685.i2c_port;
	const int chan = pin*4; //?
	const int duty_off = duty_cycle&0x1FFF; //duty_cycle값의 하위 13비트 값만 받는다. 

	wiringPiI2CWriteReg16(I2C_PORT, LED0_OFF_L+chan, duty_off);
}