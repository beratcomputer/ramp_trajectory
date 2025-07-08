/*
 * ramp_trajectory.c
 *
 *  Created on: Jun 26, 2024
 *      Author: BeratComputer
 */
#include "ramp_trajectory.h"

tReturnTrajectory initRampTrajectory(tRampTrajectory* trajectory, float interval_time, float* max_velocity, float* max_acceleration){
	trajectory->trajectory_time = 0;

	trajectory->MAX_VELOCITY = max_velocity;
	trajectory->MAX_ACCELERATION = max_acceleration;
	trajectory->INTERVAL_TIME = interval_time;

	return TrajectoryTrue;
}


static inline float discriminant(float desired_accel, float desired_time, float total_error){
    float needed_t1;
	float discriminant = desired_time * desired_time - 4 * (total_error/desired_accel);
    if (discriminant > 0) {
        float root1 = (desired_time + sqrt(discriminant)) / (2);
        float root2 = (desired_time - sqrt(discriminant)) / (2);
        if (2*root1 > desired_time){
            needed_t1 = root2;
        }
        else {
            needed_t1 = root1;
        }
    }
    else if (discriminant == 0) {
        float root1 = (desired_time + sqrt(discriminant)) / (2);
        needed_t1 = root1;
    }
    else {
        return -1;
    }
    return needed_t1;
}

static tTrajectoryScenario scenarioCalculator(float* needed_t1, float* needed_t2, float* needed_Vp, float desired_acc, float desired_time, float Vmax, float total_error){
	uint8_t traj_case = (!(desired_acc == 0.0)) << 1 | (!(desired_time == 0.0));
	float Vp;
	float acc = desired_acc;
	switch(traj_case){
		case (0b01):	// only TIME is defined
			if (Vmax*desired_time > total_error){
				Vp = 2*total_error/desired_time;
				if(Vp <= Vmax){
					*needed_t2 = 0;
					*needed_t1 = desired_time/2;
					*needed_Vp = Vp;
					return SCENARIO_CALCULATED;
				}
				else{
					*needed_t1 = desired_time - total_error/Vmax;
					*needed_t2 = desired_time - 2* (*needed_t1);
					*needed_Vp = Vmax;
					return SCENARIO_CALCULATED;
				}
			}
			else{
				return NOT_POSSIBLE_TIME_DEFINED;
			}
			break;
		case (0b011):	// TIME and ACCELERATION are defined.
			*needed_t1 = discriminant(acc, desired_time, total_error);
			if(*needed_t1 == -1){
				return NOT_POSSIBLE_ACCEL_TIME_DEFINED;
			}
			else{
				if(Vmax > (*needed_t1) * acc){
					*needed_t2 = desired_time - 2*(*needed_t1);
					*needed_Vp = (*needed_t1) * acc;
					return SCENARIO_CALCULATED;
				}
				else{
					return NOT_POSSIBLE_ACCEL_TIME_DEFINED;
				}
			}
			break;
		case (0b00): 	// BOTH are undefined.
			acc = Vmax / 2;
		case (0b10):	// only ACCELERATION is defined.
			*needed_t1 = sqrtf(total_error/acc);
			Vp = acc* (*needed_t1);
			if(Vp > Vmax){
				*needed_t1 = Vmax/acc;
				*needed_t2 = (total_error - Vmax*(*needed_t1))/Vmax;
				*needed_Vp = Vmax;
				return SCENARIO_CALCULATED;
			}
			else{
				*needed_t2 = 0;
				*needed_t1 = Vp/acc;
				*needed_Vp = Vp;
				return SCENARIO_CALCULATED;
			}
			break;
		default:
			break;
	}
	return NOT_POSSIBLE_TIME_DEFINED;
}



tReturnTrajectory createRampTrajectory(tRampTrajectory* trajectory, float inital_pos, float goal_pos){
	float needed_t1;		// hizlanana kadar gecen sure.
	float needed_t2;		// sabit hizda kaldigi sure.
	float t1_pos;			// hizlanirken aldigi konum.
	float Vp;				// cikacagi max hiz.

	trajectory->initial_pos = inital_pos;
	trajectory->goal_pos = goal_pos;

    float desired_time = trajectory->desired_time;
	float acc  		 =   trajectory->desired_accel;
	float Vmax 		 = 	 trajectory->desired_maxVelocity;
	float total_error = trajectory->goal_pos - trajectory->initial_pos;

	if((Vmax > *trajectory->MAX_VELOCITY) || (Vmax == 0.0)){        // Vmax is not true.
		Vmax = *trajectory->MAX_VELOCITY;
	}
	if((acc > *trajectory->MAX_ACCELERATION)||(acc == 0.0)){
		acc = *trajectory->MAX_ACCELERATION;
	}

	if(total_error < 0){
		trajectory->direction = -1;
		total_error = fabs(total_error);
	}
	else trajectory->direction = 1;

	tTrajectoryScenario check = scenarioCalculator(&needed_t1, &needed_t2, &Vp, acc, desired_time, Vmax, total_error);
	if (check != SCENARIO_CALCULATED){
		switch (check){
			case NOT_POSSIBLE_ACCEL_TIME_DEFINED:
				scenarioCalculator(&needed_t1, &needed_t2, &Vp, acc, 0.0, Vmax, total_error);
				break;
			case NOT_POSSIBLE_TIME_DEFINED:
				scenarioCalculator(&needed_t1, &needed_t2, &Vp, 0.0 , 0.0 , Vmax, total_error);
				break;
			default:
				break;
		}
	}

	acc = Vp / needed_t1;
	t1_pos = (Vp*needed_t1)/2;

	trajectory->needed_time = (needed_t1*2 + needed_t2);
	trajectory->needed_t1 = needed_t1;
	trajectory->needed_t2 = needed_t2;
	trajectory->needed_accel = acc;
	trajectory->needed_Vp = Vp;
	trajectory->needed_t1_pos = t1_pos;
	trajectory->trajectory_time = 0.0;
	return TrajectoryTrue;
}

float goWithRampTrajectory(tRampTrajectory* trajectory){
	static float setpoint = 0;

	/////////////////// CONTROL OUTPUT set position //////////////////////

	if(trajectory->trajectory_time < trajectory->needed_t1){
		// needed_t1 / speed up
		setpoint = trajectory->initial_pos + (trajectory->trajectory_time * trajectory->trajectory_time * trajectory->needed_accel / 2) * trajectory->direction;
	}
	else if(trajectory->trajectory_time < trajectory->needed_t1 + trajectory->needed_t2){
		// needed_t2 / constant velocity
		setpoint = trajectory->initial_pos + (trajectory->needed_t1_pos + trajectory->needed_Vp * (trajectory->trajectory_time - trajectory->needed_t1)) * trajectory->direction;
	}
	else if(trajectory->trajectory_time < trajectory->needed_t1 + trajectory->needed_t2 + trajectory->needed_t1){
		// needed_t1 / slow down
		setpoint = trajectory->goal_pos - trajectory->needed_accel*(trajectory->needed_time - trajectory->trajectory_time)*(trajectory->needed_time - trajectory->trajectory_time)/2*trajectory->direction;
	}
	else{
		// end.
		setpoint = trajectory->goal_pos;
	}

	//Check Target position limit.
	if((trajectory->direction > 0) && (setpoint > trajectory->goal_pos)){
		setpoint = trajectory->goal_pos;
	}
	else if((trajectory->direction < 0) && (setpoint < trajectory->goal_pos)){
		setpoint = trajectory->goal_pos;
	}

	trajectory->trajectory_time += trajectory->INTERVAL_TIME; //keeps in ms.
	return setpoint;
}
