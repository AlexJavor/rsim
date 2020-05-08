#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

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
	PointCyl closest_gap;
    Valley free_walking_area;
    int pathClear;
	pthread_mutex_t* mutex_data;
} Data;

Data *init(void) {
    Data* data = malloc(sizeof(Data));
    // careful, not initialized...
    data->mutex_data = malloc(sizeof(*(data->mutex_data)));
    if (data->mutex_data == NULL) {
        free(data);
        printf("ERROR malloc data\n");
        return NULL;
    }
    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);
	printf("step 3\n");
    pthread_mutex_init(data->mutex_data, &mutexattr);
	printf("step 4\n");
	pthread_mutexattr_destroy(&mutexattr);
    return data;
}

PointCyl absToCyl(const Point p, Data *data){
	PointCyl cyl;
	cyl.dist = sqrt(pow(p.x - data->current.x , 2.0) + pow(p.y - data->current.y , 2.0));
	float sintheta = (p.y - data->current.y) / cyl.dist ;
	float theta = asinf(sintheta);
	cyl.angle = theta;
	return cyl;
}

Point cylToAbs(const PointCyl pC, Data *data){
	Point p;
	p.x= cosf(pC.angle)*pC.dist + data->current.x;
	p.y= sinf(pC.angle)*pC.dist + data->current.y;
	return p;
}

PointCyl *tabAbsToTabCyl (Data *data, const Point *points, uint32_t len){
	PointCyl* tabCyl = malloc(len*sizeof(PointCyl));
	Point obstacle;
	obstacle.x=0;
	obstacle.y=0;
	//printf("Mutex Debug : tabAbsToCyl : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
    for (int i=0; i< len; i++) { //on ajoute les points qui ne sont pas la target au tableau
    	if ((fabs(points[i].x-data->target.x) > 1 )|| (fabs(points[i].y-data->target.y) > 1)){
			tabCyl[i] = absToCyl(points[i], data);
		}else{
    		tabCyl[i] = absToCyl(obstacle, data);
    	}
    }
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : tabAbsToCyl : lacher\n");
    return tabCyl;
}

//TODO
int isLarge(Valley v){
	return 0;
}

void free_data(Data *data) {
	pthread_mutex_destroy(data->mutex_data);
	free(data->mutex_data);
    free(data);
}


void on_new_target(Data *data, Point target) {
    data->target = target;
}



void on_new_pose(Data *data, Pose p) {
    // update current pose
    data->current = p;
}

/**
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
     	fprintf(fpoints, "len = %d \n", len);
		//printf("Mutex Debug : pathisclear : Prise\n");
		/*************************************************/
     	pthread_mutex_lock(data->mutex_data);
     	PointCyl targetCyl = absToCyl(data->target, data);
     	pthread_mutex_unlock(data->mutex_data);
		/*************************************************/
		//printf("Mutex Debug : pathisclear : lacher\n");
        for (int i=0; i< len; i++) {
            dist = points[i].dist * sinf(fabs(targetCyl.angle-points[i].angle));  //verify that the width  of the robot matches the path's one
			//printf ("dist : %f, angle %f \n", points[i].dist, points[i].angle*180/M_PI);
            angle = points[i].angle ;
            //printf("dist = %f, angle = %f;\n", dist, angle);
            fprintf(fpoints, "%f\t%f \n", angle, dist);
            //dist est la distance entre un obstacle et la droite (Robot---Target)
            if ((fabs(dist) < (ROBOT_WIDTH + SAFETY_DIST)) && (pathClear == 0)&& (points[i].dist < targetCyl.dist)){
                //printf("obstacle detected dist : %f\n", fabs(dist));
                pathClear = 1;
            }
            fprintf(fpoints, "path is clear = %d \n", pathClear);
        }
		//printf ("target dist : %f, angle %f\n",targetCyl.dist,targetCyl.angle*180/M_PI);
        printf("-------------------------------------------------------------------------------------------------\n");
        fprintf(fpoints, "\n;\n");
        fclose(fpoints);
    }
     else
         {
         printf("Impossible d'ouvrir le fichier fpoints\n");
     }
	//printf("Mutex Debug : pathisclear : Prise\n");
	/*************************************************/
     pthread_mutex_lock(data->mutex_data);
     data->pathClear = pathClear;
     pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : pathisclear : lacher\n");
}

//---------------------------------------------------------------------------------------------------------------------------

/** Indique si des obstacles sont présents le long du vecteur
	//Pose self = data->current ;
	//Point target = data->target ;
	// là si on arrive à bien faire le tableau de correspondance
	// angle -> distance, il suffit de regarder à l'angle theta du vecteur
	// si il y a une correspondance. Si oui, ça veut dire que le lidar a repéré un obstacle
	// Si non, ça veut dire que pour l'instant le vecteur est considéré comme "libre"
 ** retourne 1 si c'est le cas **/
int free_vector(PointCyl vector, PointCyl *points, uint32_t len){
	int vector_free = 0;
	float dist;
	for (int i=0; i< len; i++) {
		dist = points[i].dist * sin(fabs(vector.angle-points[i].angle));  //verify that the width  of the robot matches the path's one
		//dist est la distance entre un obstacle et le vecteur (Robot---Point)
		if ((fabs(dist) < (ROBOT_WIDTH + HIGH_SAFETY)) && (vector_free == 0)&& (points[i].dist < vector.dist)){// && (points[i].dist < targetCyl.dist) (dist < (ROBOT_WIDTH/2))
			//printf("obstacle detected dist : %f\n", fabs(dist));
			vector_free= 1;
		}
	}
	return vector_free;
}

/**
 * @param points
 * @param data
 * @param len
 * Update data->closest_gap with the new *points data from new scan
 */
void closest_gap(PointCyl *points, Data *data, uint32_t len){
	int found = 0  ;
	PointCyl winner ;
	//printf("Mutex Debug : closest_gap : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
	PointCyl rvg = absToCyl(data->target, data); //vecteur de gauche
	PointCyl rvd = absToCyl(data->target, data); // vecteur de droite
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : closest_gap : lacher\n");
	// on regarde progressivement à gauche et à droite quel sera le gap le plus proche
	while (!found){
		rvg.angle -= 0.01 ;
		rvd.angle += 0.01 ;
		if (free_vector(rvg,points, len)==0){
			winner = rvg ;
			found = 1 ;
		} else if (free_vector(rvd,points, len)==0){
			winner = rvd ;
			found = 1 ;
		}
	}
	//printf("Mutex Debug : closest_gap : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
	data->closest_gap = winner;
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : closest_gap : lacher\n");
	printf("closest_Gap distance = %f, angle %f", winner.dist, winner.angle*180/M_PI);
}

void on_new_scan(Data *data, const Point *points, uint32_t len) {
    // distance to closest obstacle
    float min_dist = 9999999.0;
	//printf("Mutex Debug : onNewScan : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
    Pose cur = data->current;
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : onNewScan : lacher\n");
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
	//printf("Mutex Debug : onNewScan : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
    data->closest_obstacle = min_dist;
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : onNewScan : lacher\n");
	closest_gap(tabCyl, data, len);
    pathIsClear(tabCyl,data,len);   //Update of path state from robot to target
}

int direction(Data*, int);


Command get_command(Data *data) {
    Command cmd;
    //Point target = data->target;
    //Decision Tree of ND Navigation ---------
    //printf("Mutex Debug : onNewScan : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
    float closest_obstacle = data->closest_obstacle;
    int pathClear = data->pathClear;
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : onNewScan : lacher\n");
    if (closest_obstacle > HIGH_SAFETY) {
        if (pathClear == 0){
            //HSGR ----------------------------
            cmd.forward_vel = MAX_SPEED;
            int dir = direction(data, 1);//situation à gérer
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
            int dir = direction(data, 2);//situation à gérer
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

/** return -1 for left, 0 for center and 1 for right 69 = ERROR*/
/** Situation :
 * 1 = HSGR
 * 2 = HSWR
 *
 *
 *
 *
*/
int direction(Data* data, int Situation) {


	//printf("Mutex Debug : direction : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
	// self direction vector
	float self_dir_x = cosf(data->current.theta);
	float self_dir_y = sinf(data->current.theta);

	// right pointing vector
	float rightward_x = -self_dir_y;
	float righward_y = self_dir_x;

	// target direction
	float target_dir_x = data->target.x - data->current.x;
	float target_dir_y = data->target.y - data->current.y;
	float dot_prod = rightward_x * target_dir_x + righward_y * target_dir_y;
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : direction : lacher \n");
	PointCyl follow;
	//int diff;

	if (Situation == 1) {

		printf("HSGGGGGR\n");
		if (fabs(dot_prod) < 0.0001) { // note the number here is arbitrary
			return 0; // approximately center
		} else if (dot_prod < 0) {
			return -1; // left
		} else {
			return 1; // right
		}
	}else if (Situation == 2){
		printf("HSWWWWWR\n");
		//printf("Mutex Debug : direction : prise\n");
		/*************************************************/
		pthread_mutex_lock(data->mutex_data);
		follow = data->closest_gap;
		//Point follow_abs = cylToAbs(follow, data);
		/*float follow_dir_x = follow_abs.x - data->current.x;
		float follow_dir_y = follow_abs.y - data->current.y;
		float follow_prod = rightward_x * follow_dir_x + righward_y * follow_dir_y;*/

		float follow_prod = data->current.theta - follow.angle;
		pthread_mutex_unlock(data->mutex_data);
		/*************************************************/
		//printf("Mutex Debug : direction : lacher\n");
		// Adjust the angle in order to have it between -Pi and +Pi
		if (follow_prod > M_PI) {
			follow_prod -= 2 * M_PI;
		} else if (follow_prod < -M_PI) {
			follow_prod += 2 * M_PI;
		}
		printf("followprod = %f", follow_prod*180/M_PI);
		if (fabs(follow_prod) < 0.0001) { // note the number here is arbitrary
			return 0; // approximately center
		} else if (follow_prod > 0) {
			printf("Gauche\n");
			return -1; // left
		} else {
			printf("Droite\n");
			return 1; // right
		}
	} else {
		printf("erreur de situation    ") ;
		return 1 ;
	}
}

