#ifndef __00_DRONE_H__
#define __00_DRONE_H__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include "pca9685.h"

#define I2C_SLAVE 0x0703

//// 변수 형 정의(구조체) ////
typedef struct { int port; } i2c_t; // port = I2C 디바이스의 핸들값
typedef struct { uint8_t i2c_addr, PWR_MGMT_1, GYRO_XOUT_H, i2c_port; } mpu6050_t; // i2c_addr = 디바이스의 주소, i2c_port = 디바이스의 핸들값, 나머지 = MPU6050의 레지스터 주소
typedef struct { int16_t x, y, z; } gyro_raw_t; // 16비트의 원시 자이로값
typedef struct { int16_t x, y, z; } gyro_offset_t; // 0에서 떨어진 기준값들의 평균 자이로값(오차)
typedef struct { int16_t x, y, z; } gyro_adj_t; // 16비트의 보정 자이로값
typedef struct { double roll, pitch, yaw; } gyro_rate_t; // 실수 형의 각속도값
typedef struct { unsigned long t_prev, t_now; double t_period; } dt_t; // t_prev 이전 자이로를 읽은 시간, t_now 현재 자이로를 읽은 시간, t_period 주기
typedef struct { double roll, pitch, yaw; } gyro_angle_t; // RPY 세 방향에 대한 회전각
typedef struct { double roll, pitch, yaw; } target_angle_t;
typedef struct { double roll, pitch, yaw; } balancing_force_t; // 세 방향에 대한 균형힘
typedef struct { double value; } throttle_t; // 모터 기본 속도
typedef struct { double a,b,c,d; } motor_speed_t; // 보정된 모터 a,b,c,d 각각의 속도
typedef struct { int serial_port; } hm10_t;
// typedef struct { int dummy; } pca9685_t;
typedef struct { int a, b, c ,d; } motor_t; // 모터의 핀 번호를 저장.

//// 함수 원형 선언 ////
void init(i2c_t&); // I2C 장치를 초기화

void init(i2c_t&, mpu6050_t&); // I2C 디바이스에 연결된 MPU6050디바이스를 초기화(균형계)
void read(mpu6050_t&, gyro_raw_t&); // mpu6050 디바이스로 부터 원시 자이로값을 얻어온다.
void get(mpu6050_t&, gyro_offset_t&); // 자이로 오차 평균값(offset)을 구하는 함수
void calc(gyro_adj_t&, gyro_raw_t&, gyro_offset_t&); // 원시 자이로값(raw)과 자이로 오차평균을 이용하여 보정자이로(adj) 값을 계산
void calc(gyro_rate_t&, gyro_adj_t&); // 보정자이로(adj) 값을 이용하여 회전 각속도(rate)를 구한다.

void init(dt_t&); // 주기시간을 초기화
void calc(dt_t&); // 주기시간을 계산
void calc(gyro_angle_t&, gyro_rate_t&, dt_t&); // 회전각속도(rate)와 주기(dt)를 이용하여 회전각(angle)을 구한다
void calc(balancing_force_t&, target_angle_t&, gyro_angle_t&); // 목표각도(target_angle), 회전각도(gyro_angle)를 이용하여 균형을 잡기위한 힘을 계산 
void distribute(motor_speed_t&, throttle_t&, balancing_force_t&); // throttle, 균형 힘을 이용하여 모터속도를 계산

void init(hm10_t&); // HM10을 초기화 - 사용자로 부터 throttle입력을 받을 준비한다.
void check(hm10_t&, throttle_t&, target_angle_t&); // HM10으로부터 throttle값과 목표 각도 값을 얻어온다.

void init(i2c_t&, pca9685_t&); // I2C 디바이스에 연결된 PCA9685디바이스를 초기화(모터)
void update(pca9685_t&, motor_t&, motor_speed_t&); // 모터속도 값을 이용하여 PCA9685 디바이스에 연결된 모터출력 값 갱신
void add(balancing_force_t&, gyro_rate_t&); // 회전 각속도(rate)를 추가해 균형 힘 값 계산
void add(balancing_force_t&, target_angle_t&, gyro_angle_t&, dt_t&); // 목표각도, 회전각도, 주기를 이용하여 드론 조종 시 반응을 빠르게함

void print(gyro_raw_t&);
void println(void);
void print(gyro_offset_t&);
void print(gyro_adj_t&);
void print(gyro_rate_t&);
void print(dt_t&);
void print(gyro_angle_t&);
void print(balancing_force_t&);
void print(motor_speed_t&);

#endif