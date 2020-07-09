#ifndef __PCA9685_H__
#define __PCA9685_H__

#define MODE1 0x00 // MODE1 레지스터의 주소값 정의
#define PRE_SCALE 0xFE // PRE_SCALE 레지스터의 주소값 정의
#define LED0_ON_L 0x06 // LED0_ON_L 레지스터의 주소값 정의
#define LED0_OFF_L 0x08 // LED0_OFF_L 레지스터의 주소값 정의

#define AI 0x20 // MODE1 레지스터의 AI비트를 정의 : 0x20 -> 32 -> 2^5 = 5bit 
#define SLEEP 0x10 // MODE1 레지스터의 SLEEP비트를 정의 : 0x10 -> 16 -> 2^4 = 4bit 
#define RESTART 0x80 // MODE1 레지스터의 RESTART비트를 정의 : 0x80 -> 128 -> 2^7 = 7bit 

typedef struct { int i2c_addr, i2c_port; } pca9685_t; // i2c_addr : PCA9685 장치의 i2c 주소값, i2c_port : PCA9685 읽거나 쓸 때 사용하는 지시번호

void setupAI(pca9685_t&); // MODE1 레지스터의 AI비트를 1로 설정
void setFreq(pca9685_t&, const int); // PCA9685 장치의 PWM 주파수를 설정하고 내부 오실레이터를 재가동 (분주기 적용을 위해)
void setDuty(pca9685_t&, const int, const int); // PWM 파형의 한 주기에 대한 LOW구간의 시작 COUNTER 값을 설정하는 함수 (Duty의 범위 설정?)

#endif