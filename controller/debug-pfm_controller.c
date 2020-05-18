#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/************************************* Basic tests ****************************************/

#define SAFETY_DIST 1.8
#define MAX_SPEED 2.0
#define MAX_STEER 0.4

#define D0 20.0
#define NB_OBSTACLES 200
#define K_ATT 1000.0
#define K_REF 50.0

/**
 * Definition of the vector structure : it has an (x,y) coordinate
 */
typedef struct Vector {
    float x; // meters
    float y; // meters
} Vector;


/** Some vectors used for the tests */
Vector v1 = {7.0, 3.21};
Vector v2 = {0.0, 0.0};
Vector v3 = {M_PI, M_PI_2};
Vector v4 = {-1.36, 5.745741};
Vector v5 = {-1.47, -0.745746};

Vector v6 = {1.0, 1.0};
Vector v7 = {3.0, 2.0};
Vector v8 = {6.0, 7.0};
Vector v9 = {-8.0, 2.0};
Vector v10 = {4.0, 17.0};

/**
 * * Function that prints a vector
 * @param v The vector we want to print
 * @param name The name of the vector
 */
void print_vector(Vector v, char* name){
	printf("Vector %s : (x = %.2f; y = %.2f)\n", name, v.x, v.y);
}

/**
 * * Test function for the print_vector function
 */
void test_print_vector(){
	printf("**************************** PRINTING VECTORS TESTS ****************************\n");
	print_vector(v1, "v1");
	print_vector(v2, "v2");
	print_vector(v3, "v3");
	print_vector(v4, "v4");
	print_vector(v5, "v5");
	print_vector(v6, "v6");
	print_vector(v7, "v7");
	print_vector(v8, "v8");
	print_vector(v9, "v9");
	print_vector(v10, "v10");
	printf("********************************************************************************\n\n");
}

/************************************ Adding tests ************************************/

/**
 * * Function that adds two vectors
 * @param v1 First vector to add
 * @param v2 Second vector to add
 * @return The sum of the 2 vectors
 */
Vector add_vectors(Vector v1, Vector v2){
    Vector v_return;test_print_vector();

	test_add_vectors();

	test_mult_vector_by_constant();

	test_scalar_product();
    v_return.x = v1.x + v2.x;
    v_return.y = v1.y + v2.y;
    return v_return;
}

/**
 * * Test function for the add_vectors function
 */
void test_add_vectors(){
    printf("***************************** ADDING VECTORS TESTS *****************************\n");
	print_vector(add_vectors(v1,v2), "v1 + v2");
	print_vector(add_vectors(v6,v7), "v6 + v7");
	print_vector(add_vectors(v8,v9), "v8 + v9");
	print_vector(add_vectors(v10,v9), "v10 + v9");
	print_vector(add_vectors(v4,v5), "v4 + v5");
	printf("********************************************************************************\n\n");
}

/*************************** Multiplying a vector by a constant tests ***************************/

/**
 * * Function that multiplies a vector by a constants
 * @param v1 The vector we want to multiply
 * @param k The constant that multiplies the vector
 * @return The product of the vector times the constant
 */
Vector mult_vector_by_const(Vector v1, float k){
    Vector v_return;
    v_return.x = v1.x * k;
    v_return.y = v1.y * k;
    return v_return;
}

/**
 * * Test function for the mult_vector_by_const function
 */
void test_mult_vector_by_constant(){
	printf("******************** MULTIPLYING VECTORS BY CONSTANTS TESTS ********************\n");
	print_vector(mult_vector_by_const(v1,2.0), "v1 * 2.0");
	print_vector(mult_vector_by_const(v2,4.0), "v2 * 4.0");
	print_vector(mult_vector_by_const(v6,3.0), "v6 * 3.0");
	print_vector(mult_vector_by_const(v9,10.0), "v9 * 10.0");
	print_vector(mult_vector_by_const(v3,2.0), "v3 * 2.0");
	printf("********************************************************************************\n\n");
}

/****************************** Scalar products tests ******************************/

/**
 * * Function that calculates the scalar product of two vectors
 * @param v1 The first vector
 * @param v2 The second vector
 * @return The scalar product of these two vectors
 */
float scalar_product(Vector v1, Vector v2){
    float res = v1.x*v2.x + v1.y*v2.y;
    return res;
}

/**
 * * Test function for the scalar_product function
 */
void test_scalar_product(){
	printf("***************************** SCALAR PRODUCT TESTS *****************************\n");
	printf("Scalar product v6 * v7 : %.2f\n", scalar_product(v6, v7));
	printf("Scalar product v8 * v7 : %.2f\n", scalar_product(v8, v7));
	printf("Scalar product v8 * v9 : %.2f\n", scalar_product(v8, v9));
	printf("Scalar product v2 * v10 : %.2f\n", scalar_product(v2, v10));
	printf("Scalar product v3 * v1 : %.2f\n", scalar_product(v3, v1));
	printf("********************************************************************************\n\n");
}

/**************************** Force_reference_angle ****************************/
float force_reference_angle(Vector f_total){
    Vector v_ref = {-1.0,0.0};
    float scal_prod = scalar_product(v_ref, f_total);
    //float f_total_length = sqrt(powf(f_total.x,2) + powf(f_total.y,2));
	float f_total_length = 1000.0;
    float f_ref_angle = acosf(scal_prod/f_total_length*1.0);
    
	
    // Adjust the angle in order to have it between -Pi and +Pi
    if(f_ref_angle > M_PI){
        f_ref_angle -= 2*M_PI;
    } else if (f_ref_angle < -M_PI){
        f_ref_angle += 2*M_PI;
    }
	
    //printf("f_ref_angle = %f\n", f_ref_angle);
    return f_ref_angle;
}

void test_force_reference_angle(){
	Vector v1 = {-1000.0,0.0};
	Vector v2 = {-800.0,0.0};
	Vector v3 = {-600.0,0.0};
	Vector v4 = {-500.0,0.0};
	Vector v5 = {-400.0,0.0};
	Vector v6 = {-200.0,0.0};
	Vector v7 = {0.0,0.0};
	Vector v8 = {200.0,0.0};
	Vector v9 = {400.0,0.0};
	Vector v10 = {500.0,0.0};
	Vector v11 = {600.0,0.0};
	Vector v12 = {800.0,0.0};
	Vector v13 = {1000.0,0.0};

	printf("******************************* FORCE ANGLE TESTS ******************************\n");
	printf("Assuming there is no obstacles and f_total_length = 1000.0\n");
	printf("ANGLE: t_total = (-1000,y) : %.2f\n", force_reference_angle(v1));
	printf("ANGLE: t_total = (-800,y)  : %.2f\n", force_reference_angle(v2));
	printf("ANGLE: t_total = (-600,y)  : %.2f\n", force_reference_angle(v3));
	printf("ANGLE: t_total = (-500,y)  : %.2f\n", force_reference_angle(v4));
	printf("ANGLE: t_total = (-400,y)  : %.2f\n", force_reference_angle(v5));
	printf("ANGLE: t_total = (-200,y)  : %.2f\n", force_reference_angle(v6));
	printf("ANGLE: t_total = (0,y)     : %.2f\n", force_reference_angle(v7));
	printf("ANGLE: t_total = (200,y)   : %.2f\n", force_reference_angle(v8));
	printf("ANGLE: t_total = (400,y)   : %.2f\n", force_reference_angle(v9));
	printf("ANGLE: t_total = (500,y)   : %.2f\n", force_reference_angle(v10));
	printf("ANGLE: t_total = (600,y)   : %.2f\n", force_reference_angle(v11));
	printf("ANGLE: t_total = (800,y)   : %.2f\n", force_reference_angle(v12));
	printf("ANGLE: t_total = (1000,y)  : %.2f\n", force_reference_angle(v13));
	printf("********************************************************************************\n\n");
}


int main(int argc, char const *argv[])
{
	test_force_reference_angle();

    printf("END OF THE TESTS\n");
    return 0;
}
