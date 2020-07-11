#include "_00_drone.h"

void calc(balancing_force_t& balancing_force, target_angle_t& target_angle, gyro_angle_t& gyro_angle) // 목표각도(target_angle), 회전각도(gyro_angle)를 이용하여 균형을 잡기위한 힘을 계산 
{
	double angle_error_roll = target_angle.roll - gyro_angle.roll;
	double angle_error_pitch = target_angle.pitch - gyro_angle.pitch;
	double angle_error_yaw = target_angle.yaw - gyro_angle.yaw;

	balancing_force.roll = 1*angle_error_roll; // 1은 Kp 증폭상수 , 힘보정
	balancing_force.pitch = 1*angle_error_pitch;
	balancing_force.yaw = 1*angle_error_yaw;
}

void distribute(motor_speed_t& motor_speed, throttle_t& throttle, balancing_force_t& balancing_force) // throttle, 균형 힘을 이용하여 모터속도를 계산
{
	motor_speed.a = throttle.value + 10 + balancing_force.yaw + balancing_force.pitch + balancing_force.roll;
	motor_speed.b = throttle.value - balancing_force.yaw + balancing_force.pitch - balancing_force.roll;
	motor_speed.c = throttle.value + balancing_force.yaw - balancing_force.pitch - balancing_force.roll;
	motor_speed.d = throttle.value + 10 - balancing_force.yaw - balancing_force.pitch + balancing_force.roll;  

	if(motor_speed.a < 0) motor_speed.a = 0; // 모터의 속도는 0~250으로 맞춘다. 이를 통해 모터로 가는 값이 0~4095로 정해진다.
	if(motor_speed.a > 250) motor_speed.a = 250; 
	if(motor_speed.b < 0) motor_speed.b = 0;
	if(motor_speed.b > 250) motor_speed.b = 250; 
	if(motor_speed.c < 0) motor_speed.c = 0;
	if(motor_speed.c > 250) motor_speed.c = 250; 
	if(motor_speed.d < 0) motor_speed.d = 0;
	if(motor_speed.d > 250) motor_speed.d = 250; 

	if(throttle.value == 0) { // throttle이 0일때 모든 모터를 0으로 맞춰준다.
		motor_speed.a = 0;
		motor_speed.b = 0;
		motor_speed.c = 0;
		motor_speed.d = 0;
	}
}

void add(balancing_force_t& balancing_force, gyro_rate_t& gyro_rate) // 회전 각속도(rate)를 추가해 균형 힘 값 계산
{
	balancing_force.pitch += 0.5*(-gyro_rate.pitch);
	balancing_force.roll += 0.5*(-gyro_rate.roll);
	balancing_force.yaw += 0.5*(-gyro_rate.yaw);
} 
void add(balancing_force_t& balancing_force, target_angle_t& target_angle, gyro_angle_t& gyro_angle, dt_t& dt) // 목표각도, 회전각도, 주기를 이용하여 드론 조종 시 반응을 빠르게함
{
	static double res_force_pitch;
	static double res_force_roll;
	static double res_force_yaw;

	double angle_error_pitch = target_angle.pitch - gyro_angle.pitch;
	double angle_error_roll = target_angle.roll - gyro_angle.roll;
	double angle_error_yaw = target_angle.yaw - gyro_angle.yaw;

	res_force_pitch += 1* angle_error_pitch*dt.t_period;
	res_force_roll += 1* angle_error_roll*dt.t_period;
	res_force_yaw += 0* angle_error_yaw*dt.t_period;

	extern throttle_t throttle;
	if(throttle.value == 0){
		res_force_roll=0;
		res_force_pitch =0;
		res_force_yaw =0;
	}

	balancing_force.roll += res_force_roll;
	balancing_force.pitch += res_force_pitch;
	balancing_force.yaw += res_force_yaw;
}
 // 목표각도, 회전각도, 주기를 이용하여 드론 조종 시 반응을 빠르게함
