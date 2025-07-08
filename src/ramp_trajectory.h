
/*
 * ramp_trajectory.h
 *
 *  Created on: Jun 26, 2024
 *      Author: BeratComputer
 */

#ifndef INC_RAMP_TRAJECTORY_H_
#define INC_RAMP_TRAJECTORY_H_

#include "math.h"
#include "stdint.h"

typedef enum{
	TrajectoryFalse = -1,
	TrajectorySteady = 0,
	TrajectoryTrue = 1,
}tReturnTrajectory;

typedef enum{
	NOT_POSSIBLE_ACCEL_TIME_DEFINED = -1,
	NOT_POSSIBLE_TIME_DEFINED,
	SCENARIO_CALCULATED,
}tTrajectoryScenario;

typedef struct{
	// INPUTS
	float initial_pos;
	float goal_pos;

	float desired_time;
	float desired_accel;
	float desired_maxVelocity;

	// OUTPUTS
	float needed_time;
	float needed_t1;
	float needed_t2;
	float needed_Vp;
	float needed_t1_pos;
	float needed_accel;
	int8_t direction;

	float trajectory_time;
	float setpoints[4000];

	// CONSTANTS
	float* MAX_VELOCITY;			// pointer for update in main side.
	float* MAX_ACCELERATION;		// pointer for update in main side.
	float INTERVAL_TIME; 			// in seconds. default since init.
}tRampTrajectory;

tReturnTrajectory initRampTrajectory(tRampTrajectory* trajectory, float interval_time, float* max_velocity, float* max_acceleration);
tReturnTrajectory createRampTrajectory(tRampTrajectory* trajectory, float inital_pos, float goal_pos);
float goWithRampTrajectory(tRampTrajectory* trajectory);

#endif /* INC_RAMP_TRAJECTORY_H_ */
