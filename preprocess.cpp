#include "preprocess.h"

#include <iostream> // REMOVE THIS
#include <math.h>

#define HEIGHT 480
#define WIDTH 640

// The maximum observable depth, in meters.
#define MAX_DEPTH 10

// Here are all the camera parameters
#define FX_RGB 5.1930334103339817e+02
#define FY_RGB 5.1816401430246583e+02
#define CX_RGB 3.2850951551345941e+02
#define CY_RGB 2.5282555217253503e+02

// Distortion coefficients
#define K1_RGB 2.5785516449232132e-01
#define K2_RGB -9.1141470196267182e-01
#define P1_RGB 3.0173013316440469e-04
#define P2_RGB 2.5422024034001231e-03
#define K3_RGB 1.1823504884394158e+00

// Depth camera parameters
#define FX_D 5.7616540758591043e+02
#define FY_D 5.7375619782082447e+02
#define CX_D 3.2442516903961865e+02
#define CY_D 2.3584766381177013e+02

#define K1_D  -1.3708537316819339e-01
#define K2_D 7.2482751812234414e-01
#define P1_D 8.0826809257389550e-04
#define P2_D 3.4151576458975323e-03
#define K3_D -1.4621396186358457e+00

// Inverse Rotation matrix in column major order.
#define R1 0.999985794494467
#define R2 -0.003429138557773
#define R3 0.00408066391266
#define R4 0.003420377768765
#define R5 0.999991835033557
#define R6 0.002151948451469
#define R7 -0.004088009930192
#define R8 -0.002137960469802
#define R9 0.999989358593300

// Translation vector
#define T1 -2.2142187053089738e-02
#define T2 1.4391632009665779e-04
#define T3 7.9356552371601212e-03

// Constants for undoing the depth nonlinearity.
#define DN_W 0.3513e3
#define DN_B 1.0925e3

void kinect::preprocess_depth(uint16_t* depth, uint8_t* rgb, uint8_t* depth_out) {
  // First, flip the bits!
	//kinect::swap_bytes(depth, HEIGHT * WIDTH);
  
	// Next, remove the depth nonlinearity.
	double* depth_abs = new double[HEIGHT * WIDTH];
	remove_depth_nonlinearity(depth, depth_abs);
	
	// Next, project the depth image onto the RGB image using the calibrated
	// internal parameters.
	double* depth_proj = new double[HEIGHT * WIDTH];
  
  // Crucial for the background.
  for (int i = 0; i < HEIGHT * WIDTH; ++i) {
    depth_proj[i] = 0;
  }
  
	project_depth(depth_abs, depth_proj);
  
  // Finally, scale the output.
	scale_depth(depth_proj, depth_out);
	
	// Cleanup.
	delete depth_abs;
	delete depth_proj;
}

void kinect::project_depth(double* depth_abs, double* depth_proj) {
	// ****************************************
	//   PROJECT THE DEPTH TO 3D WORLD POINTS
	// ****************************************
	double* x_world = new double[HEIGHT * WIDTH];
	double* y_world = new double[HEIGHT * WIDTH];
	double* z_world = new double[HEIGHT * WIDTH];
	
	double* x_world_p = x_world;
	double* y_world_p = y_world;
	double* z_world_p = z_world;
	
	double* depth_abs_p = depth_abs;
	
  // We start with x=1 and NOT x=0 on purpose.
	for (int y = 1; y < HEIGHT + 1; ++y) {
  	for (int x = 1; x < WIDTH + 1; ++x, ++x_world_p, ++y_world_p, ++z_world_p, ++depth_abs_p) {
			*x_world_p = (x - CX_D) * *depth_abs_p / FX_D;
			*y_world_p = (y - CY_D) * *depth_abs_p / FY_D;
			*z_world_p = *depth_abs_p;
		}	
	}
	
	// *******************************************
	//   Next, Rotate and translate the 3D points
	// *******************************************
	// R * [X; Y; Z] + T
	
	x_world_p = x_world;
	y_world_p = y_world;
	z_world_p = z_world;
	
	double x_tmp, y_tmp, z_tmp;
	for (int i = 0; i < HEIGHT * WIDTH; ++i, ++x_world_p, ++y_world_p, ++z_world_p) {
		x_tmp = *x_world_p;
		y_tmp = *y_world_p;
		z_tmp = *z_world_p;
		*x_world_p = (R1 * x_tmp) + (R2 * y_tmp) + (R3 * z_tmp) + T1;
		*y_world_p = (R4 * x_tmp) + (R5 * y_tmp) + (R6 * z_tmp) + T2;
		*z_world_p = (R7 * x_tmp) + (R8 * y_tmp) + (R9 * z_tmp) + T3;
	}
	
	// *******************************************
	//   Project into the RGB coordinate frame.
	// *******************************************

	x_world_p = x_world;
	y_world_p = y_world;
	z_world_p = z_world;
	
	double* x_proj = new double[HEIGHT * WIDTH];
	double* y_proj = new double[HEIGHT * WIDTH];
	double* x_proj_p = x_proj;
	double* y_proj_p = y_proj;
	
	for (int i = 0; i < HEIGHT * WIDTH; ++i, ++x_proj_p, ++y_proj_p, ++x_world_p, ++y_world_p, ++z_world_p) {
	  *x_proj_p = *x_world_p * FX_RGB / *z_world_p + CX_RGB;
		*y_proj_p = *y_world_p * FY_RGB / *z_world_p + CY_RGB;
	}
	
	delete x_world;
	delete y_world;
	delete z_world;
	
	// ************************************************
	// Finally, reassign the values in ROW MAJOR order.
	// ************************************************
	depth_abs_p = depth_abs;
	
	x_proj_p = x_proj;
	y_proj_p = y_proj;

	double* depth_assgn = new double[HEIGHT * WIDTH];
	double* depth_assgn_p = depth_assgn;
	for (int i = 0; i < HEIGHT * WIDTH; ++i, ++depth_assgn_p) {
		*depth_assgn_p = 10;
	}
	
	int absOffset;
	int x, y;
	for (int i = 0; i < HEIGHT * WIDTH; ++i, ++x_proj_p, ++y_proj_p, ++depth_abs_p) {
		x = static_cast<int> (round(*x_proj_p));
		y = static_cast<int> (round(*y_proj_p));

    --x;
    --y;
    
		if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
			continue;
		}
		
		absOffset = y * WIDTH + x;
		
		if (*depth_abs_p > depth_assgn[absOffset]) {
			continue;
		}
		
		depth_assgn[absOffset] = *depth_abs_p;
		depth_proj[absOffset] = *depth_abs_p;
	}
		
	delete depth_assgn;
	
	delete x_proj;
	delete y_proj;
}

void kinect::swap_bytes(uint16_t* vec, int length) {
	uint16_t low, high;
  for (int i = 0; i < length; ++i, ++vec) {
		low = *vec & 255;
		high = *vec >> 8;
		*vec = (low << 8) + high;
	}
}

void kinect::remove_depth_nonlinearity(uint16_t* src, double* dst) {
	for (int i = 0; i < HEIGHT * WIDTH; ++i, ++src, ++dst) {
		*dst = DN_W / (DN_B - static_cast<double> (*src));
		
		if (*dst > MAX_DEPTH) {
			*dst = MAX_DEPTH;
		} else if (*dst < 0) {
			*dst = 0;
		}
	}
}

// TODO(silberman): the matlab code throws out the bottom and top 2%. 
// We may want to do the same.
void kinect::scale_depth(double* src, uint8_t* dst) {
	
	double max = 0;
	double min = 10e10;
	double* src_p = src;
	for (int i = 0; i < HEIGHT * WIDTH; ++i, ++src_p) {
		if (*src_p > max) {
			max = *src_p;
		}
		if (*src_p < min) {
			min = *src_p;
		}
	}
	
	src_p = src;
	char val;
	for (int i = 0; i < HEIGHT * WIDTH; ++i, ++src, ++dst) {
		*dst = static_cast<char>(round(255.0 * (*src-min) / (max-min)));
	}
}
