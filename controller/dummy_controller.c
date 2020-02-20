#include <math.h>
#include <stdio.h>

#include "controller.h"

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

#define SAFETY_DIST 2.0
#define MAX_SPEED 2.0
#define MAX_STEER 0.4

typedef struct Data {
    Pose current;
    Point target;
    float closest_obstacle;
} Data;

Data *init(void) {
    Data* data = malloc(sizeof(Data));
    // careful, not initialized...
    return data;
}


void free_data(Data *data) {
    free(data);
}


void on_new_target(Data *data, Point target) {
    data->target = target;
}



void on_new_pose(Data *data, Pose p) {
    // update current pose
    data->current = p;
}


void on_new_scan(Data *data, const Point *points, uint32_t len) {
    // distance to closest obstacle
    float min_dist = 9999999.0;
    Pose cur = data->current;
    for (int i=0; i< len; i++) {
        Point impact = points[i];
        float dx = cur.x - impact.x;
        float dy = cur.y - impact.y;
        float d = sqrtf(dx * dx + dy * dy);
        if(d < min_dist) {
            min_dist = d;
        }
    }
    // update in data
    data->closest_obstacle = min_dist;
}

int direction(Pose, Point);

Command get_command(Data *data) {
    Command cmd;
    Point target = data->target;
    target.x = 100;
    target.y = 30;
    if(data->closest_obstacle > SAFETY_DIST) {
        // got some margin
        cmd.forward_vel = MAX_SPEED;
        int dir = direction(data->current, target);
        if(dir == 0) {
            cmd.steering = 0;
        } else if(dir == -1) {
            // GO LEFT !!!!
            cmd.steering = -MAX_STEER;
        } else {
            // GO RIGHT !!!!!!
            cmd.steering = MAX_STEER;
        }
    } else {
        // too close, stop
        cmd.forward_vel = 0.0;
        cmd.steering = 0.0;
    }
    return cmd;
}

/** return -1 for left, 0 for center and 1 for right */
int direction(Pose self, Point target) {
    // self direction vector
    float self_dir_x = cosf(self.theta);
    float self_dir_y = sinf(self.theta);

    // right pointing vector
    float rightward_x = -self_dir_y;
    float righward_y = self_dir_x;

    // target direction
    float target_dir_x = target.x - self.x;
    float target_dir_y = target.y - self.y;

    float dot_prod = rightward_x * target_dir_x + righward_y * target_dir_y;

    if(abs(dot_prod) < 0.001) { // note the numer here is arbitrary
        return 0; // approximately center
    } else if(dot_prod < 0) {
        return -1; // left
    } else {
        return 1; // right
    }
}
