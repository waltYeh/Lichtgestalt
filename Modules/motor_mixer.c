#include "motor_mixer.h"
#include "stabilizer_types.h"
#include "../Devices/battery.h"
#include "../Devices/motor_pwm.h"
#include "../Commons/utils.h"
#include "../Commons/platform.h"


void force2output(int force[4], unsigned short duty[4], unsigned int battery)
//mNewton to 0~2400
{
	int k,i;
	if(battery == 0){
		k = 110;
	} else{
		//full4.20V(4.05V) k=100, low3.80V(3.65V) k=120
		k = constrain_int32(303-(int)battery/20,95,125);
	}
	for(i=0;i<4;i++){
		duty[i] = force[i] * k / MOTOR_POWER + 2650;
		duty[i] = duty[i]* 5 / 12;//originally 2400~4800, now 1000~2000
	}
}

void powerDistribution(output_t* output, const battery_t * bat)
{
	int motorForce[4] = {0,0,0,0};
	unsigned short motorDuty[4] = {1000,1000,1000,1000};
	short i;
	if(0){
		motorForce[0] = 0;
		motorForce[1] = 0;
		motorForce[2] = 0;
		motorForce[3] = 0;
		for(i=0;i<4;i++){
			motorDuty[i]=1000;
		}
	}
	else{
	#if PLUS
/*			A								inv(A)
			[  1,  1, 1,  1]				[ 1/4, -1/(2*d),        0,  1/(4*c)]
			[ -d,  0, d,  0]				[ 1/4,        0, -1/(2*d), -1/(4*c)]
			[  0, -d, 0,  d]				[ 1/4,  1/(2*d),        0,  1/(4*c)]
			[  c, -c, c, -c]				[ 1/4,        0,  1/(2*d), -1/(4*c)]*/
		motorForce[0] = (output->thrust*0.25f) - output->moment.P * ONE_2_ROTOR_DIST + output->moment.Y * ONE_4_F_T_RATIO;
		motorForce[1] = (output->thrust*0.25f) - output->moment.R * ONE_2_ROTOR_DIST - output->moment.Y * ONE_4_F_T_RATIO;
		motorForce[2] = (output->thrust*0.25f) + output->moment.P * ONE_2_ROTOR_DIST + output->moment.Y * ONE_4_F_T_RATIO;
		motorForce[3] = (output->thrust*0.25f) + output->moment.R * ONE_2_ROTOR_DIST - output->moment.Y * ONE_4_F_T_RATIO;
	#elif CROSS
/*			A													inv(A)
			[        1,        1,        1,       1]			[ 1/4, -sqrt2/(4*d),  sqrt2/(4*d),  1/(4*c)]
			[ -d/sqrt2, -d/sqrt2,  d/sqrt2, d/sqrt2]			[ 1/4, -sqrt2/(4*d), -sqrt2/(4*d), -1/(4*c)]
			[  d/sqrt2, -d/sqrt2, -d/sqrt2, d/sqrt2]			[ 1/4,  sqrt2/(4*d), -sqrt2/(4*d),  1/(4*c)]
			[        c,       -c,        c,      -c]			[ 1/4,  sqrt2/(4*d),  sqrt2/(4*d), -1/(4*c)]
		d=D/2
		torq=c*f*/
	#endif		
		force2output(motorForce, motorDuty, bat->voltage);
	}
	motor_pwm_output(motorDuty);
}
