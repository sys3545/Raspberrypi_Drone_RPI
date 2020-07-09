#include "_00_drone.h"
#define NSAMPLES 1000

void init(i2c_t& i2c) // I2C 장치를 초기화
{
	i2c.port = open("/dev/i2c-1", O_RDWR);
	if(i2c.port<0){
		fprintf(stderr, "Unable to open device : %s\n", strerror(errno));
		exit(-1);
	}
}

void init(i2c_t& i2c, mpu6050_t& mpu6050) // I2C 디바이스에 연결된 MPU6050디바이스를 초기화(균형계)
{
	mpu6050.i2c_port = i2c.port;
	ioctl(mpu6050.i2c_port, I2C_SLAVE, mpu6050.i2c_addr); // 접근할 I2C 슬레이브를 MPU6050으로 설정
	wiringPiI2CWriteReg8(mpu6050.i2c_port, mpu6050.PWR_MGMT_1, 0); // MPU를 깨운다 (SLEEP을 0으로)
}

void read(mpu6050_t& mpu6050, gyro_raw_t& gyro_raw) // mpu6050 디바이스로 부터 원시 자이로값을 얻어온다.
{
	const int I2C_PORT = mpu6050.i2c_port;
	const int GYRO_XH = mpu6050.GYRO_XOUT_H;

	ioctl(I2C_PORT, I2C_SLAVE, mpu6050.i2c_addr);

	gyro_raw.x = (wiringPiI2CReadReg8(I2C_PORT, GYRO_XH+0)&0xFF)<<8;
	gyro_raw.x |= wiringPiI2CReadReg8(I2C_PORT, GYRO_XH+1)&0xFF;
	gyro_raw.y = (wiringPiI2CReadReg8(I2C_PORT, GYRO_XH+2)&0xFF)<<8;
	gyro_raw.y |= wiringPiI2CReadReg8(I2C_PORT, GYRO_XH+3)&0xFF;
	gyro_raw.z = (wiringPiI2CReadReg8(I2C_PORT, GYRO_XH+4)&0xFF)<<8;
	gyro_raw.z |= wiringPiI2CReadReg8(I2C_PORT, GYRO_XH+5)&0xFF;
}

void get(mpu6050_t& mpu6050, gyro_offset_t& gyro_offset) // 자이로 오차 평균값(offset)을 구하는 함수
{
	gyro_raw_t gyro_raw;
	int32_t sumGyX=0, sumGyY=0, sumGyZ=0;

	for(int i=0;i<NSAMPLES;i++){ // NSAMPLE = 평균을 구하기 위해 반복하는 횟수
		read(mpu6050, gyro_raw);

		sumGyX += gyro_raw.x;
		sumGyY += gyro_raw.y;
		sumGyZ += gyro_raw.z;
		delay(1);
	}

	gyro_offset.x = (double)sumGyX/NSAMPLES; //평균
	gyro_offset.y = (double)sumGyY/NSAMPLES;
	gyro_offset.z = (double)sumGyZ/NSAMPLES;
}

void calc(gyro_adj_t& gyro_adj, gyro_raw_t& gyro_raw, gyro_offset_t& gyro_offset) // 원시 자이로값(raw)과 자이로 오차평균을 이용하여 보정자이로(adj) 값을 계산
{
	gyro_adj.x = gyro_raw.x - gyro_offset.x; //원시값에서 평균값을 빼준다.
	gyro_adj.y = gyro_raw.y - gyro_offset.y;
	gyro_adj.z = gyro_raw.z - gyro_offset.z;
}

void calc(gyro_rate_t& gyro_rate, gyro_adj_t& gyro_adj) // 보정자이로(adj) 값을 이용하여 회전 각속도(rate)를 구한다.
{
	gyro_rate.roll = gyro_adj.y/131.0; // 보정값에서 131을 나누면 각속도
	gyro_rate.pitch = gyro_adj.x/131.0;
	gyro_rate.yaw = gyro_adj.z/131.0;
}

void init(dt_t& dt) // 주기시간을 초기화
{
	dt.t_prev = micros(); //마이크로 초단위의 시간을 구하는 함수
}

void calc(dt_t& dt) // 주기시간을 계산
{
	dt.t_now = micros();
	dt.t_period = (dt.t_now - dt.t_prev)/1000000.0;
	dt.t_prev = dt.t_now;
}

void calc(gyro_angle_t& gyro_angle, gyro_rate_t& gyro_rate, dt_t& dt) // 회전각속도(rate)와 주기(dt)를 이용하여 회전각(angle)을 구한다
{
	gyro_angle.roll += gyro_rate.roll*dt.t_period;
	gyro_angle.pitch += gyro_rate.pitch*dt.t_period;
	gyro_angle.yaw += gyro_rate.yaw*dt.t_period;

	extern throttle_t throttle;
	if(throttle.value ==0){ // throttle이 0일때 회전각도를 누적하여 구한 각을 0으로 해줘야 불필요한 움직임이 없다.
		gyro_angle.pitch =0;
		gyro_angle.roll = 0;
		gyro_angle.yaw =0;
	}
}