#include "_00_drone.h"

void init(hm10_t& hm10) // HM10을 초기화 - 사용자로 부터 throttle입력을 받을 준비한다.
{
	hm10.serial_port = serialOpen("/dev/ttyS0" , 115200);

	if(hm10.serial_port<0){
		fprintf(stderr, "Unable to open serial device : %s\n", strerror(errno));
		exit(-1);
	}
}
 
void check(hm10_t& hm10, throttle_t& throttle, target_angle_t& target_angle) // HM10으로부터 throttle값과 목표 각도 값을 얻어온다.
{
	static int cntMsg;

	if(serialDataAvail (hm10.serial_port)){
		while(serialDataAvail(hm10.serial_port)){
			char msp_data = serialGetchar(hm10.serial_port);
			if(msp_data == '$') cntMsg=0;
			else cntMsg++;

			if(cntMsg == 8) throttle.value = msp_data;
			else if(cntMsg == 7)
				target_angle.yaw = -(msp_data-125);
			else if(cntMsg == 6)
				target_angle.pitch = -(msp_data-124.5);
			else if(cntMsg == 5)
				target_angle.roll = (msp_data-120);

			#define ANGLE_MAX 30
			if(target_angle.roll < -ANGLE_MAX)
				target_angle.roll = -ANGLE_MAX;
			else if(target_angle.roll > ANGLE_MAX)
				target_angle.roll = ANGLE_MAX;
			if(target_angle.pitch < -ANGLE_MAX)
				target_angle.pitch = -ANGLE_MAX;
			else if(target_angle.pitch > ANGLE_MAX)
				target_angle.pitch = ANGLE_MAX;
			if(target_angle.yaw < -ANGLE_MAX)
				target_angle.yaw = -ANGLE_MAX;
			else if(target_angle.yaw > ANGLE_MAX)
				target_angle.yaw = ANGLE_MAX;
		}
	}
} 
