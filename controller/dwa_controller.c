#include <math.h>
#include <stdio.h>

#include "dwa_controller.h"

/**
 * QUICK AND DIRTY CONTROLLER
 *
 * Implements a simple controller that moves in the direction of the goal
 * at max speed.
 *
 * Stops if it is too close to an obstacle.
 *
 * Problems:
 * - not thread safe
 * - might start driving before being given target/position/obstacles
 */

#define SAFETY_DIST 1.4
#define MAX_SPEED 2.05
#define MAX_STEER 0.8


typedef struct Data
{
    Pose currentPose;
    Point target;
    float closest_obstacle;
	PointCloud collidePoints;
	Velocity currentVelocity;
	Config config;
} Data;


Data *init(void)
{
    Data* data = malloc(sizeof(Data));

	data->config.maxSpeed = MAX_SPEED;
	data->config.minSpeed = 0.1;
	data->config.maxTheta = MAX_STEER;
	data->config.maxSpeedAccel = 0.5;
	data->config.maxThetaAccel = 0.1;
	data->config.linVelocityResolution = 0.05;
	data->config.angVelocityResolution = 0.05;
	data->config.time = 0.3;
	data->config.predictTime = 3;
	data->config.headingObjective = 0.07;
	data->config.clearanceObjective = 0.6;
	data->config.velocityObjective = 0.4;
	Rectangle robot;
	robot.xmax = 1.5;
	robot.xmin = -1.5;
	robot.ymax = 1.5;
	robot.ymin = -1.5;
	data->config.robot = robot;

	data->currentVelocity.linearVelocity = 0.0;
	data->currentVelocity.angularVelocity = 0.0;
    
    return data;
}


void free_data(Data *data)
{
    free(data);
}


void on_new_target(Data *data, Point target)
{
	// update target
    data->target = target;
}


void on_new_pose(Data *data, Pose p)
{
    // update current pose
    data->currentPose = p;
}


void on_new_scan(Data *data, const Point *points, uint32_t len)
{
    // distance to closest obstacle
    float min_dist = 9999999.0;
	for (int i = 0; i < len; i++)
	{
		Point impact = points[i];
		float dx = data->currentPose.point.x - impact.x;
		float dy = data->currentPose.point.y - impact.y;
		float d = sqrtf(dx * dx + dy * dy);
		if (d < min_dist) {
			min_dist = d;
		}

		/*if (data->target.x == points[i].x && data->target.x == points[i].y)
		{
			points[i].x = -1;
			points[i].y = -1;
		}*/
    }
	// update in data
	data->closest_obstacle = min_dist;

	data->collidePoints.size = len;
	data->collidePoints.points = (Point*)points;
}


Command get_command(Data *data)
{
    Command cmd;
    if(data->closest_obstacle > SAFETY_DIST)
	{
		data->currentVelocity = planning(data->currentPose, data->currentVelocity, data->target, &data->collidePoints, data->config);
		cmd.forward_vel = data->currentVelocity.linearVelocity;
		cmd.steering = data->currentVelocity.angularVelocity;
    } else
	{
        // too close, stop
        cmd.forward_vel = 0.0;
        cmd.steering = 0.0;
    }
    return cmd;
}


void createDynamicWindow(Velocity velocity, Config config, DynamicWindow **dynamicWindow)
{
	float minV = fmax(config.minSpeed, velocity.linearVelocity - config.maxSpeedAccel * config.time);
	float maxV = fmin(config.maxSpeed, velocity.linearVelocity + config.maxSpeedAccel * config.time);
	float minW = fmax(-config.maxTheta, velocity.angularVelocity - config.maxThetaAccel * config.time);
	float maxW = fmax(config.maxTheta, velocity.angularVelocity + config.maxThetaAccel * config.time);

	int nPossibleV = (maxV - minV) / config.linVelocityResolution;
	int nPossibleW = (maxW - minW) / config.angVelocityResolution;
	*dynamicWindow = malloc(sizeof(DynamicWindow));

	(*dynamicWindow)->possibleV = malloc(nPossibleV * sizeof(float));
	(*dynamicWindow)->possibleW = malloc(nPossibleW * sizeof(float));
	(*dynamicWindow)->nPossibleV = nPossibleV;
	(*dynamicWindow)->nPossibleW = nPossibleW;

	for (int i = 0; i < nPossibleV; i++)
	{
		(*dynamicWindow)->possibleV[i] = minV + (float)i * config.linVelocityResolution;
	}

	for (int i = 0; i < nPossibleW; i++)
	{
		(*dynamicWindow)->possibleW[i] = minW + (float)i * config.angVelocityResolution;
	}
}


void freeDynamicWindow(DynamicWindow *dynamicWindow)
{
	free(dynamicWindow->possibleV);
	free(dynamicWindow->possibleW);
	free(dynamicWindow);
}


Pose motion(Pose pose, Velocity velocity, float time)
{
	Pose new_pose;
	new_pose.theta = pose.theta + velocity.angularVelocity * time;
	new_pose.point.x = pose.point.x + velocity.linearVelocity * cos(new_pose.theta) * time;
	new_pose.point.y = pose.point.y + velocity.linearVelocity * sin(new_pose.theta) * time;
	return new_pose;
}


float calculateVelocityCost(Velocity velocity, Config config)
{
	return config.maxSpeed - velocity.linearVelocity;
}


float calculateHeadingCost(Pose pose, Point target)
{
	float dx = target.x - pose.point.x;
	float dy = target.y - pose.point.y;
	float angleError = atan2(dy, dx);
	float angleCost = angleError - pose.theta;
	return fabs(atan2(sin(angleCost), cos(angleCost)));
}


float calculateClearanceCost(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config)
{
	Pose pPose = pose;
	float time = 0.0;
	float minr = FLT_MAX;
	while (time < config.predictTime)
	{
		pPose = motion(pPose, velocity, config.time);
		for (int i = 0; i < pointCloud->size; ++i)
		{
			float dx = pPose.point.x - pointCloud->points[i].x;
			float dy = pPose.point.y - pointCloud->points[i].y;
			float x = -dx * cos(pPose.theta) + -dy * sin(pPose.theta);
			float y = -dx * -sin(pPose.theta) + -dy * cos(pPose.theta);
			if (x <= config.robot.xmax &&
				x >= config.robot.xmin &&
				y <= config.robot.ymax &&
				y >= config.robot.ymin)
			{
				return FLT_MAX; // collision
			}
			float r = sqrtf(dx * dx + dy * dy);
			if (r < minr) minr = r;
		}
		time += config.time;
	}
	return 1.0 / minr;
}


Velocity planning(Pose pose, Velocity velocity, Point target, PointCloud *pointCloud, Config config)
{
	DynamicWindow *dw;
	createDynamicWindow(velocity, config, &dw);
	float total_cost = FLT_MAX;
	Velocity bestVelocity;
	for (int i = 0; i < dw->nPossibleV; ++i)
	{
		for (int j = 0; j < dw->nPossibleW; ++j)
		{
			Velocity pVelocity;
			pVelocity.linearVelocity = dw->possibleV[i];
			pVelocity.angularVelocity = dw->possibleW[j];
			Pose pPose = motion(pose, pVelocity, config.predictTime);
			float cost = config.velocityObjective * calculateVelocityCost(pVelocity, config) +
				config.headingObjective * calculateHeadingCost(pPose, target) +
				config.clearanceObjective * calculateClearanceCost(pose, pVelocity, pointCloud, config);
			if (cost < total_cost)
			{
				total_cost = cost;
				bestVelocity = pVelocity;
			}
		}
	}
	freeDynamicWindow(dw);
	return bestVelocity;
}