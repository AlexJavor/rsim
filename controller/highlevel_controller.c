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
 *
 * ./rsim --controller controller/libhighlevel_controller.so --map maps/base.jpg -t 180
 */

#define SAFETY_DIST 1.8
#define MAX_SPEED 2.0
#define MAX_STEER 0.4
//High state definition
#define HIGH_SAFETY 6.0
#define ROBOT_WIDTH 1.0
#define LARGE_VALLEY 1.5 //the angle size of a valley in radius (1.5 rad ~ 90 deg)

typedef struct PointCylindrique {
    float dist, angle;
} PointCyl;

typedef struct Valley {
    PointCyl leftEdge, rightEdge, center;         //Edges of the valley
} Valley;

typedef struct Data {
    Pose current;
    Point target;
    float closest_obstacle;
	PointCyl closest_gap;
    Valley free_walking_area;
    int pathClear;
	pthread_mutex_t* mutex_data;
	int ndSituation;
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
    pthread_mutex_init(data->mutex_data, &mutexattr);
	pthread_mutexattr_destroy(&mutexattr);
	data->ndSituation = 0;
    return data;
}

//Sort of constructor for a valley
Valley newValley(float rightEdgeAngle, float rightEdgeDist,float leftEdgeAngle,float leftEdgeDist,float centerAngle,float centerDist){
	Valley v;
	v.rightEdge.angle = rightEdgeAngle;
	v.rightEdge.dist = rightEdgeDist;
	v.leftEdge.angle = leftEdgeAngle;
	v.leftEdge.dist = leftEdgeDist;
	v.center.angle = centerAngle;
	v.center.dist = centerDist;
	return v;
}

PointCyl absToCyl(const Point p, Pose current){
	PointCyl cyl;
	cyl.dist = sqrt(pow(p.x - current.x , 2.0) + pow(p.y - current.y , 2.0));
	float theta;
	if (p.x >= current.x){	//Il y a pb de symétrie, il faut décaler de PI/2
		float sintheta = (p.y - current.y) / cyl.dist ;
		theta = asinf(sintheta);
	} else {
		float costheta = (p.x - current.x) / cyl.dist;
		theta = acosf(costheta);
		if (p.y < current.y){
			theta = - theta;
		}
	}
	cyl.angle = theta;
	return cyl;
}

Point cylToAbs(const PointCyl pC, Pose current){
	Point p;
	p.x= cosf(pC.angle)*pC.dist + current.x;
	p.y= sinf(pC.angle)*pC.dist + current.y;
	return p;
}

int cmpfunc (const void * a, const void * b) {
	PointCyl pca = *(PointCyl*)a;
	PointCyl pcb = *(PointCyl*)b;
	if ( (pca.angle - pcb.angle) > 0 ){
		return 1;
	} else if (( pca.angle - pcb.angle ) < 0){
		return -1;
	}else {
		return 0;
	}
}

PointCyl *tabAbsToTabCyl (Pose current, Point target, const Point *points, uint32_t len){
	PointCyl* tabCyl = malloc(len*sizeof(PointCyl));
    for (int i=0; i< len; i++) {
    	tabCyl[i] = absToCyl(points[i], current);
    }
	qsort(tabCyl, len, sizeof(PointCyl), cmpfunc);
    return tabCyl;
}

/**
 * @param vallee
 * @return 0 if valley is large, 1 if valley is small
 */
int isLarge(Valley vallee){
	int retour;
	if (vallee.leftEdge.angle < vallee.rightEdge.angle){
		if (fabs(vallee.rightEdge.angle - vallee.leftEdge.angle) < LARGE_VALLEY){
			retour = 1;
		} else {
			retour = 0;
		}
	} else {
		vallee.leftEdge.angle = vallee.leftEdge.angle- (2*M_PI);
		if (fabs(vallee.rightEdge.angle - vallee.leftEdge.angle) < LARGE_VALLEY){
			retour = 1;
		} else {
			retour = 0;
		}
	}
	return retour;
}

void free_data(Data *data) {
	pthread_mutex_destroy(data->mutex_data);
	free(data->mutex_data);
    free(data);
}


void on_new_target(Data *data, Point target) {
	pthread_mutex_lock(data->mutex_data);
    data->target = target;
	pthread_mutex_unlock(data->mutex_data);
}

void on_new_pose(Data *data, Pose p) {
    // update current pose
	pthread_mutex_lock(data->mutex_data);
    data->current = p;
	pthread_mutex_unlock(data->mutex_data);
}

/**
 * Function which gives
 * 0 if the direction from the robot to the target is free
 * 1 if it has at least an obstacle
 * Possible Upgrade : useless to check all the points : if the angle between the obstacle and the target is above 90° then it can't be on the way
 */
int pathIsClear(PointCyl *points, Pose current, Point target, uint32_t len){
    int pathClear = 0;
    float dist ;
    float angle;
    FILE* fpoints = fopen("fcoordonnees.txt", "a");
     if (fpoints != NULL)
    {
     	fprintf(fpoints, "len = %d \n", len);
     	PointCyl targetCyl = absToCyl(target, current);
        for (int i=0; i< len; i++) {
            dist = points[i].dist * sinf(fabs(targetCyl.angle-points[i].angle));  //verify that the width  of the robot matches the path's one
			//printf ("dist : %f, angle %f \n", points[i].dist, points[i].angle*180/M_PI);
            angle = points[i].angle ;
            //printf("dist = %f, angle = %f;\n", dist, angle);
            fprintf(fpoints, "%f\t%f \n", angle, dist);
            //dist est la distance entre un obstacle et la droite (Robot---Target)
            if ((fabs(dist) < (2* ROBOT_WIDTH + HIGH_SAFETY)) && (pathClear == 0)&& (points[i].dist < targetCyl.dist)){
                //printf("obstacle detected dist : %f\n", fabs(dist));
                pathClear = 1;
            }
            fprintf(fpoints, "path is clear = %d \n", pathClear);
        }
		//printf ("target dist : %f, angle %f\n",targetCyl.dist,targetCyl.angle*180/M_PI);
        //printf("-------------------------------------------------------------------------------------------------\n");
        fprintf(fpoints, "\n;\n");
        fclose(fpoints);
    }else {
     	printf("Impossible d'ouvrir le fichier fpoints\n");
     }
	return pathClear;
}

//---------------------------------------------------------------------------------------------------------------------------

/** Indique si des obstacles sont présents le long du vecteur
 * Pose self = data->current ;
 * Point target = data->target ;
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
		if ((fabs(dist) < (ROBOT_WIDTH + SAFETY_DIST)) && (vector_free == 0)&& (points[i].dist < vector.dist)){// && (points[i].dist < targetCyl.dist) (dist < (ROBOT_WIDTH/2))
			//obstacle detected
			vector_free= 1;
		}
	}
	return vector_free;
}

/**
 *
 * @param freeVector : Free vector un the valley
 * @param points : Sorted by angle obstacle tab
 * @param len : number of obstacle
 * @return Valley : the two edges of the valley
 */
Valley findValley(PointCyl freeVector, PointCyl *points, uint32_t len){
	Valley vallee;
	PointCyl leftEdge = points[len-1];
	PointCyl rightEdge = points[0];
	PointCyl center = freeVector;
	//cas général:
	int i = 0;
	while (i<len && freeVector.angle > points[i].angle){
		leftEdge = points[i];
		i++;
	}
	if (i==0 || i>=len){
		rightEdge = points[0];
	}else {
		rightEdge = points[i + 1];
	}
	//printf("Vallee : leftEdge angle = %f, freeVector angle = %f, rightEdge angle = %f\n", leftEdge.angle*180/M_PI, freeVector.angle*180/M_PI, rightEdge.angle*180/M_PI);
	if (leftEdge.angle < rightEdge.angle){
		center.angle = leftEdge.angle + (rightEdge.angle - leftEdge.angle)/2;
	} else {
		leftEdge.angle = leftEdge.angle- (2*M_PI);
		center.angle = leftEdge.angle+ (rightEdge.angle - leftEdge.angle)/2;
		if (center.angle > M_PI) {
			center.angle -= 2 * M_PI;
		} else if (center.angle < -M_PI) {
			center.angle += 2 * M_PI;
		}
	}
	vallee.center = center;
	vallee.leftEdge = leftEdge;
	vallee.rightEdge = rightEdge;
	return vallee;
}

/*Doesn't Work (Impossible to implement, to know what is a valley (need discontinuities)
Valley* findTabValley( const Point *points, Pose current, Point target, uint32_t len){
	Valley *tabValley = malloc(len*sizeof(Valley));
	Valley v;
	int indiceTabValley = 0;
	int disto1o2;
	Point center;
	disto1o2 = sqrt(pow(points[len].x- points[0].x , 2.0) + pow(points[len].y- points[0].y , 2.0));
	if (disto1o2 > ROBOT_WIDTH + SAFETY_DIST){	//Vallée trouvée !
		center.x = (points[len].x + points[0].x)/2;
		center.y = (points[len].y + points[0].y)/2;
		v.leftEdge = absToCyl(points[len], current);
		v.rightEdge = absToCyl(points[0], current);
		v.center = absToCyl(center, current);
		if (free_vector(v.center, tabAbsToTabCyl(current, target, points, len),len) == 0){
			//on a le bon sens
			printf("valley founded !\n");
			tabValley[indiceTabValley] = v;
			indiceTabValley ++;
		} else{
			//mauvais sens de la vallée : rotation de 180 degres de center
			v.center.angle += M_PI;
			v.rightEdge = absToCyl(points[len], current);
			v.leftEdge = absToCyl(points[0], current);
			if (free_vector(v.center, tabAbsToTabCyl(current, target, points, len),len) == 0){
				//on a le bon sens
				printf("valley founded !\n");
				tabValley[indiceTabValley] = v;
				indiceTabValley ++;
			}
		}
	}
	for (int o = 1; o < len; o++){
		disto1o2 = sqrt(pow(points[o].x- points[o-1].x , 2.0) + pow(points[o].y- points[o-1].y , 2.0));
		if (disto1o2 > ROBOT_WIDTH + SAFETY_DIST){	//Vallée trouvée !
			center.x = (points[o-1].x + points[o].x)/2;
			center.y = (points[o-1].y + points[o].y)/2;
			v.leftEdge = absToCyl(points[o-1], current);
			v.rightEdge = absToCyl(points[o], current);
			v.center = absToCyl(center, current);
			if (free_vector(v.center, tabAbsToTabCyl(current, target, points, len),len) == 0){
				//on a le bon sens
				printf("valley founded !\n");
				tabValley[indiceTabValley] = v;
				indiceTabValley ++;
			} else{
				//mauvais sens de la vallée : rotation de 180 degres de center
				v.center.angle += M_PI;
				v.rightEdge = absToCyl(points[len], current);
				v.leftEdge = absToCyl(points[0], current);
				if (free_vector(v.center, tabAbsToTabCyl(current, target, points, len),len) == 0){
					//on a le bon sens
					printf("valley founded !\n");
					tabValley[indiceTabValley] = v;
					indiceTabValley ++;
				}else{
					printf("Incorrect valley WTFFFFFFF !\n");
				}
			}
		}
	}
	//fill the rest with -1 Valley
	v = newValley(-1, -1, -1, -1, -1, -1);
	while(indiceTabValley < len){
		tabValley[indiceTabValley] = v;
		indiceTabValley++;
	}
	return tabValley;
}
*/

/**
 * @param points
 * @param data
 * @param len
 * @return PointCyl target intermédiaire if ok, PointCyl.dist = -1 if trapped
 * Update data->closest_gap with the new *points data from new scan
 */
PointCyl closest_gap(PointCyl *points, Pose current, Point targetPoint, uint32_t len){
	int found = 0;
	PointCyl winner ;
	PointCyl target = absToCyl(targetPoint, current);
	// on regarde progressivement à gauche et à droite quel sera le gap le plus proche
	PointCyl rvg = target; //vecteur de gauche
	PointCyl rvd = target; // vecteur de droite
	/**Problème  = si le robot est entouré d'obstacle (à l'intérieur d'une pièce par exemple),
	 * il n'y à pas de free vector : il faut regarder des discontinuités ?
	 * Pour la condition du while : pas besoin de tester les deux composante : si l'une depasse, l'autre aussi
	 */
	int cote = 0; //1 = droite, 2 = gauche
	while (!found && (rvd.angle < (target.angle + M_PI))){
		rvg.angle -= 0.01 ;
		rvd.angle += 0.01 ;
		if (free_vector(rvg,points, len)==0){
			cote = 2;//gauche
			winner = rvg;
			found = 1 ;
		} else if (free_vector(rvd,points, len)==0){
			cote = 1;//droite
			winner = rvd;
			found = 1 ;
		}
	}
	if (!found){
		//rvd.angle = M_PI/2;
		//winner = rvd;
		//printf("It's A Trap !!!!!!!\n");
		winner.dist = -1;
	}
	if (cote == 1){
		winner.angle += 0.2;
	}else if (cote ==2){
		rvg.angle -= 0.2;
	}
	//printf("closest_Gap distance = %f, angle %f", winner.dist, winner.angle*180/M_PI);
	return winner;
}



void on_new_scan(Data *data, const Point *points, uint32_t len) {
	//printf("Mutex Debug : onNewScan : Prise \n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
	int size = 0;
	for (int i = 0; i<len; i++){
		if ((fabs(points[i].x-data->target.x) > 1 )|| (fabs(points[i].y-data->target.y) > 1)){
			size ++;
		}
	}
	Point pointsWithoutTarget[size];
	int j = 0;
	for (int i = 0; i<len; i++){
		if ((fabs(points[i].x-data->target.x) > 1 )|| (fabs(points[i].y-data->target.y) > 1)){
			pointsWithoutTarget[j] = points[i];
			j++;
		}
	}
    Pose cur = data->current;
    printf("New Scan ********************************************************************************\n");
    printf("target angle %f, dist %f\n", absToCyl(data->target, data->current).angle, absToCyl(data->target, data->current).dist);
    /*Valley* tabValley = findTabValley(pointsWithoutTarget, data->current, data->target, len);
    Valley v;
    for (int i=0;i<size;i++){
    	v = tabValley[i];
    	if (v.leftEdge.dist != -1){
    			printf("Vallée no %d : alpha1 = %f, dist1 = %f, alphacenter = %f, distcenter = %f alpha2 = %f, dist2 = %f\n",i, v.rightEdge.angle*180/M_PI, v.rightEdge.dist, v.center.angle*180/M_PI, v.center.dist, v.leftEdge.angle*180/M_PI, v.leftEdge.dist);
    		}
    }*/
	// distance to closest obstacle
	float min_dist = 9999999.0;
    for (int i=0; i< size; i++) {
        Point impact = pointsWithoutTarget[i];
        float dx = cur.x - impact.x;
        float dy = cur.y - impact.y;
        float d = sqrtf(dx * dx + dy * dy);
        if(d < min_dist) {
            min_dist = d;
        }
    }
	/*if (min_dist<HIGH_SAFETY){	//Calculs pour Low Safety
		//recuperer le tableau d'obstacle à l'intérieur de HIGH_SAFETY zone
	}*/
    PointCyl *tabCyl = tabAbsToTabCyl(data->current, data->target,pointsWithoutTarget, len);
    // update in data
	//Update of path state from robot to target
	int pathClear = pathIsClear(tabCyl,data->current, data->target,len);
	data->pathClear = pathClear;
    data->closest_obstacle = min_dist;
	PointCyl closestGap = absToCyl(data->target, data->current);
    if (len != 0){
		closestGap = closest_gap(tabCyl, data->current, data->target, len);
		data->closest_gap = closestGap;
		data->free_walking_area = findValley(closestGap, tabCyl, len);
    }
	if (closestGap.dist == -1){
		data->ndSituation = -1; //It's A Trap
	}else if (min_dist > HIGH_SAFETY){
    	if (pathClear == 0){ //càd aucun obstacle entre le robot et la target
			data->ndSituation = 0;	//HSGR
    	} else if (isLarge(data->free_walking_area) == 0){
    		//printf("Large Valley \n");
			data->ndSituation = 1;    //HSWR
    	} else{
			data->ndSituation = 2;    //HSNR
		}
    }else{
    	data->ndSituation = 3; //LS
    }
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
	//printf("Mutex Debug : onNewScan : lacher 2\n");
}

int direction(Data*);


Command get_command(Data *data) {
    Command cmd;
    //printf("Mutex Debug : onNewScan : Prise\n");
	/*************************************************/
	pthread_mutex_lock(data->mutex_data);
    int ndSituation = data->ndSituation;
	//Decision Tree of ND Navigation ---------
	int cantGo =0;
	if (ndSituation == -1){ //It's A trap
		printf("Admiral Ackbot : << It's A Trap ! >>\n");
		cmd.forward_vel = -MAX_SPEED/2; //I'm trying and heroic move bro !
	}else if (ndSituation < 3) { //HSGR
		//printf("High Safety\n");
		cmd.forward_vel = MAX_SPEED;
	} else if (ndSituation == 3) { //LS
		//LS1 ou LS2
		//printf("Low Safety \n");
		cmd.forward_vel = MAX_SPEED /2;
	} else { //DANGER
    	printf("WARNING : TOO CLOSE !!\n");
		cmd.forward_vel = 0;
		cmd.steering = 0;
    	cantGo =1;
    }
    if (!cantGo) {
    	int dir = direction(data);//situation à gérer
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
	pthread_mutex_unlock(data->mutex_data);
	/*************************************************/
    return cmd;
}

/** return -1 for left, 0 for center and 1 for right 69 = ERROR*/
/** Situation :
 * 0 = HSGR
 * 1 = HSWR
 * 2 = HSNR
 * 3 = LS
 *
 *
*/
int direction(Data* data) {

	int Situation = data->ndSituation;
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
	PointCyl follow;
	//int diff;
	//printf("Situation : %d\n", Situation);
	if (Situation == -1){ //man You still don't understand ? I'M TRAPPED ! What am-I supposed to do ?
		return 0;
	}else if (Situation == 0) {
		if (fabs(dot_prod) < 0.0001) { // note the number here is arbitrary
			return 0; // approximately center
		} else if (dot_prod < 0) {
			return -1; // left
		} else {
			return 1; // right
		}
	}else if (Situation <3){
		if (Situation == 1) {
			follow = data->closest_gap;
		}else{ //Situation = 2
			follow = data->free_walking_area.center;
		}
		float follow_prod = data->current.theta - follow.angle;
		// Adjust the angle in order to have it between -Pi and +Pi
		if (follow_prod > M_PI) {
			follow_prod -= 2 * M_PI;
		} else if (follow_prod < -M_PI) {
			follow_prod += 2 * M_PI;
		}
		//printf("followprod = %f", follow_prod*180/M_PI);
		if (fabs(follow_prod) < 0.0001) { // note the number here is arbitrary
			return 0; // approximately center
		} else if (follow_prod > 0) {
			//printf("Gauche\n");
			return -1; // left
		} else {
			//printf("Droite\n");
			return 1; // right
		}
	} else {
		//printf("erreur de situation    \n") ;
		return 1 ;
	}
}

