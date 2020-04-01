#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <float.h>

/** Internal data structure hold by the controller. */
typedef struct Data Data;


/**
 * 2D Point
 */
typedef struct {
  float x; // meters
  float y; // meters
} Point;


typedef struct {
	float forward_vel; // desired forward velocity
	float steering; // steering angle
} Command;


/**
 * Robot pose
 */
typedef struct {
	Point point; // x, y
	float theta; // orientation in radians
} Pose;


/**
 * Called once when initializing the plugin.
 * The returned Data object will be provided as the first argument of all subsequent call.
 */
Data *init(void);

/**
 * Called once when unloading the plugin. Should deallocate the memory poointed by data.
 */
void free_data(Data *data);

/**
 * Typically invoked once to provide the (x, y) coordinates of the point to reach.
 */
void on_new_target(Data *data, Point target);

/**
 * Notifies of a new pose estimation.
 *
 * Typical update rate : 60Hz
 */
void on_new_pose(Data *data, Pose p);

/**
 * Notifies of a new laser scan results.
 * "points" is a pointer to the first element of an array of "len" elements.
 *
 * The memory pointed by "points" will be freed immediately after exiting this callback.
 * If you want to keep access to this data, you should make your own copy.
 *
 * Typical update rate: 5Hz
 */
void on_new_scan(Data *data, const Point *points, uint32_t len);

/**
 * Called to get the command to send to the wheels actuators.
 *
 * Typical inovation rate : 30Hz
 * This function should return almost instantely. If non trivial computation
 * must be made, they should be made on a separate thread and this function should just return the latest command.
 */
Command get_command(Data *data);


/*
 * AJOUT POUR DWA
 */

typedef struct
{
	float linearVelocity;
	float angularVelocity;
} Velocity;


typedef struct
{
	int nPossibleV;
	float *possibleV;
	int nPossibleW;
	float *possibleW;
} DynamicWindow;


typedef struct
{
	float xmin;
	float ymin;
	float xmax;
	float ymax;
} Rectangle;


typedef struct
{
	float maxSpeed;
	float minSpeed;
	float maxTheta;
	float maxSpeedAccel;
	float maxThetaAccel;
	float linVelocityResolution;
	float angVelocityResolution;
	float time;
	float predictTime;
	float headingObjective;
	float clearanceObjective;
	float velocityObjective;
	Rectangle robot;
} Config;


typedef struct
{
	int size;
	Point *points;
} PointCloud;



void createDynamicWindow(Velocity velocity, Config config, DynamicWindow **dynamicWindow);
void freeDynamicWindow(DynamicWindow *dynamicWindow);

Pose motion(Pose pose, Velocity velocity, float time);

float calculateVelocityCost(Velocity velocity, Config config);
float calculateHeadingCost(Pose pose, Point target);
float calculateClearanceCost(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config);

Velocity planning(Pose pose, Velocity velocity, Point target, PointCloud *pointCloud, Config config);



