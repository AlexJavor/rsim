#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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

#define SAFETY_DIST 1.8
#define MAX_SPEED 2.0
#define MAX_STEER 0.4
//High state definition
#define HIGH_SAFETY 3.0
#define ROBOT_WIDTH 1.0

typedef struct PointCylindrique {
    float dist, angle;
} PointCyl;

typedef struct Valley {
    float Alpha1, Alpha2;       //Angle un radius, the valley is oriented according to the trigo convention
    Point edge1, edge2;         //Edges of the valley
} Valley;

typedef struct Data {
    Pose current;
    Point target;
    float closest_obstacle;
    Valley free_walking_area;
    int pathClear;
} Data;

Data *init(void) {
    Data* data = malloc(sizeof(Data));
    // careful, not initialized...
    return data;
}

/*
 * Couple of functions to convert absolute position to relative cylindric points
 *  -> Analisis of discontinuity and finding Valleys
 */
PointCyl absToCyl(const Point p, Data *data){
    PointCyl cyl;
    cyl.dist = sqrt(pow(p.x-data->current.x, 2.0) + pow(p.y-data->current.y,2.0));
    int tan = (p.y - data->current.y)/(p.x - data->current.x);
    int alpha = atan(tan);
    cyl.angle = alpha - data->current.theta ; //don't perfectly work

    return cyl;
}

PointCyl *tabAbsToTabCyl (Data *data, const Point *points, uint32_t len){
    PointCyl* tabCyl = malloc(len*sizeof(PointCyl));
    for (int i=0; i< len; i++) {
        tabCyl[i] = absToCyl(points[i], data);
    }
    return tabCyl;
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

/*
 * Function which gives
 * 0 if the direction from the robot to the target is free
 * 1 if it has at least an obstacle
 * Possible Upgrade : useless to check all the points : if the angle between the obstacle and the target is above 90° then it can't be on the way
 */
void pathIsClear(PointCyl *points, Data *data, uint32_t len){
    int pathClear = 0;
    float dist ;
    float angle;
    FILE* fpoints = fopen("fcoordonnees.txt", "a");
     if (fpoints != NULL)
    {
        for (int i=0; i< len; i++) {
           dist = points[i].dist * sin(abs(absToCyl(data->target, data).angle-points[i].angle));  //verify that the width  of the robot matches the path's one
           angle = points[i].angle ;
            //printf("dist = %f, angle = %f;\n", dist, angle);
            fprintf(fpoints, "%f\t%f \n", angle, dist);
            if (dist < (ROBOT_WIDTH/2)){
                //printf("obstacle detected\n");
                pathClear = 1;
            }
        }
        fprintf(fpoints, "\n;\n");
        fclose(fpoints);
    }
     else
         {
         printf("Impossible d'ouvrir le fichier fpoints\n");
     }
    data->pathClear = pathClear;
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
    PointCyl *tabCyl = tabAbsToTabCyl(data,points, len);
    // update in data
    data->closest_obstacle = min_dist;
    pathIsClear(tabCyl,data,len);   //Update of path state from robot to target
}

int direction(Pose, Point);



Command get_command(Data *data) {
    Command cmd;
    Point target = data->target;
    //Decision Tree of ND Navigation ---------
    if (data->closest_obstacle > HIGH_SAFETY) {
        if (data->pathClear == 0){
            //HSGR ----------------------------
            cmd.forward_vel = MAX_SPEED;
            int dir = direction(data->current, target);
            if (dir == 0) {
                cmd.steering = 0;
            } else if (dir == -1) {
                // GO LEFT !!!!
                cmd.steering = -MAX_STEER;
            } else {
                // GO RIGHT !!!!!!
                cmd.steering = MAX_STEER;
            }
        } else {
            //HSWR ou HS -----------------------------
            //TODO
            cmd.forward_vel = MAX_SPEED;
            int dir = direction(data->current, target);
            if (dir == 0) {
                cmd.steering = 0;
            } else if (dir == -1) {
                // GO LEFT !!!!
                cmd.steering = -MAX_STEER;
            } else {
                // GO RIGHT !!!!!!
                cmd.steering = MAX_STEER;
            }
        }
    } else {
        //LS1 ou LS2
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

    if(fabs(dot_prod) < 0.0001) { // note the number here is arbitrary
        return 0; // approximately center
    } else if(dot_prod < 0) {
        return -1; // left
    } else {
        return 1; // right
    }
}
