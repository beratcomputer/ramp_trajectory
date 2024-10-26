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

	uint32_t trajectory_index;
	float setpoints[4000];

	// CONSTANTS
	float MAX_VELOCITY;		// step count in Interval time
	int32_t INTERVAL_TIME; 	// in ms
}tRampTrajectory;

tReturnTrajectory calculateTrajectory(tRampTrajectory* trajectory, float inital_pos, float goal_pos);
float goWithTrajectory(tRampTrajectory* trajectory , uint8_t* newTrajectoryHandle);

#endif /* INC_RAMP_TRAJECTORY_H_ */
