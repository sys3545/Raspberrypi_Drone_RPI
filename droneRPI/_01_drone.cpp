#include "_00_drone.h"

//// 변수 선언(초기화)////
i2c_t i2c;
mpu6050_t mpu6050={
	.i2c_addr = 0x68,
	.PWR_MGMT_1 = 0x6b,
	.GYRO_XOUT_H = 0x43,
};
gyro_raw_t gyro_raw;
gyro_offset_t gyro_offset;
gyro_adj_t gyro_adj;
gyro_rate_t gyro_rate;
dt_t dt;
gyro_angle_t gyro_angle;
target_angle_t target_angle = {
	.roll = 5.0,
	.pitch = 0.4,
	.yaw = 0.0,
};
balancing_force_t balancing_force;
throttle_t throttle={
	.value = 0.0,
};
motor_speed_t motor_speed;
hm10_t hm10;
pca9685_t pca9685={
	.i2c_addr =0x40,
};
motor_t motor = {
	.a = 0,
	.b = 2,
	.c = 1,
	.d = 3,
};

//// 전체 문맥 구성 ////
int main()
{
	wiringPiSetup(); // 셋업 (시작용)
	init(i2c);
	init(i2c, mpu6050);
	get(mpu6050, gyro_offset); // 자이로 오차평균을 구함 
	init(dt);
	init(hm10); //hm10 초기화 , throttle 입력 받을 준비 on
	init(i2c, pca9685);

	while(true){

		read(mpu6050, gyro_raw); 
		calc(gyro_adj, gyro_raw, gyro_offset); // 보정자이로값 계산
		calc(gyro_rate, gyro_adj); // 회전 각속도 계산 
		calc(dt); // 주기 계산 
		calc(gyro_angle, gyro_rate, dt); // 회전각도 계산
		calc(balancing_force, target_angle, gyro_angle); // 균형힘 계산 
		//add(balancing_force, gyro_rate); // 균형힘 값에 회전각속도 보정 추가
		//add(balancing_force, target_angle, gyro_angle, dt); //반응을 빠르게
		distribute(motor_speed, throttle, balancing_force); //모터 속도 계산
		check(hm10, throttle, target_angle); // throttle과 target_angle를 받아옴
		//update(pca9685, motor, motor_speed); // 모터출력값 갱신 (모터 회전)
		
		/*static int cnt;
		cnt++;
		if(cnt >= 800){
			ioctl(pca9685.i2c_port, I2C_SLAVE, pca9685.i2c_addr);
			setDuty(pca9685, motor.a, 0);
			setDuty(pca9685, motor.b, 0);
			setDuty(pca9685, motor.c, 0);
			setDuty(pca9685, motor.d, 0);

			if(cnt >=900)break;
		}*/
		static int cnt;
		cnt++;
		if(cnt%100 != 0) continue;
	
		//print(gyro_raw);
		//print(gyro_offset);
		//print(gyro_adj);
		//print(gyro_rate);
		//print(dt);
		//print(gyro_angle);
		print(balancing_force);
		add(balancing_force, gyro_rate);
		print(balancing_force);
		//print(motor_speed);
		println();
	}

	/*ioctl(pca9685.i2c_port, I2C_SLAVE, pca9685.i2c_addr);
			setDuty(pca9685, motor.a, 0);
			setDuty(pca9685, motor.b, 0);
			setDuty(pca9685, motor.c, 0);
			setDuty(pca9685, motor.d, 0);*/
}
