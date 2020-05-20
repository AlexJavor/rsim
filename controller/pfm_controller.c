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

#define SAFETY_DIST 1.8
#define MAX_SPEED 10.0
#define MAX_STEER 3.0

#define NB_OBSTACLES 200
#define K_ATT -1000.0
#define K_REF 22.5

typedef struct Data {
    Pose current;
    Point target;
    float closest_obstacle;
    // As in the mod.rs file, we have decided to take 200 points
    int nb_obstacles;
    Point obstacles[NB_OBSTACLES];
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
    // Number of obstacles seen by the lidar (obstacles array length)
    data->nb_obstacles = len;
    // distance to closest obstacle
    float min_dist = 9999999.0;
    Pose cur = data->current;

    // Updating Data with new points (obstacles) and getting the closest obstacle distance
    for (int i = 0; i < len; i++) {
        data->obstacles[i] = points[i];

        Point impact = points[i];
        float dx = cur.x - impact.x;
        float dy = cur.y - impact.y;
        float d = sqrtf(dx * dx + dy * dy);
        if(d < min_dist) {
            min_dist = d;
        }
    }
    Point infinit_point = {100000,100000};
    for(int i = len; i < NB_OBSTACLES; i++){
        data->obstacles[i] = infinit_point;
    }
    data->closest_obstacle = min_dist;
}

int direction(Data *data);

Command get_command(Data *data) {
    Command cmd;
    if(data->closest_obstacle > SAFETY_DIST) {
        // got some margin
        cmd.forward_vel = MAX_SPEED;
        int dir = direction(data);
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

/************ PFM Functions *************/

typedef struct Vector {
    float x; // meters
    float y; // meters
} Vector;

Vector add_vectors(Vector v1, Vector v2){
    Vector v_return;
    v_return.x = v1.x + v2.x;
    v_return.y = v1.y + v2.y;
    return v_return;
}

Vector mult_vector_by_const(Vector v1, float k){
    Vector v_return;
    v_return.x = v1.x * k;
    v_return.y = v1.y * k;
    return v_return;
}

float scalar_product(Vector v1, Vector v2){
    float res = v1.x*v2.x + v1.y*v2.y;
    return res;
}

Vector Attractive_Force(Data *data){
    Pose current = data->current;
    Point target = data->target;
    
    Vector current_target_vector = {(target.x-current.x), (target.y-current.y)};
    float current_target_dist = sqrt(powf((target.x-current.x),2) + powf((target.y-current.y),2));

    Vector u_vector = mult_vector_by_const(current_target_vector, (1.0/current_target_dist));
    Vector f_att = mult_vector_by_const(u_vector, K_ATT);
    Vector distance = {f_att.x*2.0/current_target_dist, f_att.y*2.0/current_target_dist};
    f_att = add_vectors(f_att,distance);
    //printf("f_att = (%f,%f)\n", f_att.x, f_att.y);
    return f_att;
}

Vector Repulsive_Force(Data *data, int cur_obst){
    Pose current = data->current;
    Point obstacle = data->obstacles[cur_obst];

    Vector current_obstacle_vector = {(obstacle.x-current.x), (obstacle.y-current.y)};
    float current_obstacle_dist = sqrt(powf((obstacle.x-current.x),2) + powf((obstacle.y-current.y),2));

    Vector u_vector = mult_vector_by_const(current_obstacle_vector, (1.0/current_obstacle_dist));
    Vector f_ret = mult_vector_by_const(u_vector, K_REF);
    //printf("f_ret = (%f,%f)\n", f_ret.x, f_ret.y);
    return f_ret;
}

Vector Total_Force(Data *data){
    Vector f_att = Attractive_Force(data);
    Vector f_ret = {0.0,0.0};
    int i;
    for(i = 0; i < data->nb_obstacles; i++){
        f_ret = add_vectors(f_ret, Repulsive_Force(data, i));
    }
    Vector f_total = add_vectors(f_att, f_ret);
    //printf("f_att   = (%f,%f)\n", f_att.x, f_att.y);
    //printf("f_total = (%f,%f)\n", f_total.x, f_total.y);
    return f_total;
}

float force_reference_angle(Vector f_total){
    Vector v_ref = {1.0,0.0};
    float scal_prod = scalar_product(v_ref, f_total);
    //printf("scal_prod = %f\n", scal_prod);
    float f_total_length = sqrt(powf(f_total.x,2) + powf(f_total.y,2));
    //printf("f_total_length = %f\n", f_total_length);
    float f_ref_angle = acosf(scal_prod/f_total_length*1.0);
    
    if(f_total.y < 0){
        f_ref_angle *= -1;
    }

    //printf("f_ref_angle = %f\n", f_ref_angle);
    return f_ref_angle;
}


/** return -1 for left, 0 for center and 1 for right */
int direction(Data *data) {

    Vector f_tot = Total_Force(data);
    // Optimal direction to reach the target
    float theta_opt = force_reference_angle(f_tot);
    // Current direction the robot is facing
    float theta_robot = data->current.theta;
    // Difference between the 2 angles
    float diff_angle = theta_opt - theta_robot;
    
    // Adjust the angle in order to have it between -Pi and +Pi
    if(diff_angle > M_PI){
        diff_angle -= 2*M_PI;
    } else if (diff_angle < -M_PI){
        diff_angle += 2*M_PI;
    }

    if(fabs(diff_angle) < 0.00001) { // note the number here is arbitrary
        return 0; // approximately center
    } else if(diff_angle > 0) {
        return -1; // left
    } else {
        return 1; // right
    }
}


