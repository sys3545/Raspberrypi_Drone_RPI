#include "_00_drone.h"

void init(i2c_t& i2c, pca9685_t& pca9685) // I2C 디바이스에 연결된 PCA9685디바이스를 초기화(모터)
{
	pca9685.i2c_port = i2c.port;
	ioctl(pca9685.i2c_port, I2C_SLAVE, pca9685.i2c_addr);

	setupAI(pca9685);
	setFreq(pca9685, 1000);
}
 
void update(pca9685_t& pca9685, motor_t& motor, motor_speed_t& motor_speed) // 모터 속도 값을 이용하여 PCA9685 디바이스에 연결된 모터출력 값 갱신
{
	int duty_cycle_a = (int)(motor_speed.a*4095.0/250);
	int duty_cycle_b = (int)(motor_speed.b*4095.0/250);
	int duty_cycle_c = (int)(motor_speed.c*4095.0/250);
	int duty_cycle_d = (int)(motor_speed.d*4095.0/250);

	ioctl(pca9685.i2c_port, I2C_SLAVE, pca9685.i2c_addr);

	setDuty(pca9685, motor.a, duty_cycle_a);
	setDuty(pca9685, motor.b, duty_cycle_b);
	setDuty(pca9685, motor.c, duty_cycle_c);
	setDuty(pca9685, motor.d, duty_cycle_d);
}
 