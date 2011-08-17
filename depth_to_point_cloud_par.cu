#include "cuda.h"

#include "preprocess.h"

#include <iostream>

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

// Args:
//   depth_abs - the absolute depth from the kinect.
//   depth_proj - the projected depth.
__global__ void depth_to_point_cloud(const float* depth_abs, int* res_x, int* res_y,
                                     const int N) {

  int idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (idx >= N) {
	  return;
	}
	
  // Figure out the current XY coordinates. These coordinates are actually
	// 1-indexed, not zero-indexed.
  int x = idx % WIDTH + 1;
	int y = static_cast<int>(floorf(idx / WIDTH) + 1);
	
  // ****************************************
	//   PROJECT THE DEPTH TO 3D WORLD POINTS
	// ****************************************
  float x_world = (x - CX_D) * depth_abs[idx] / FX_D;
	float y_world = (y - CY_D) * depth_abs[idx] / FY_D;
	float z_world = depth_abs[idx];
		
	// *******************************************
	//   Next, Rotate and translate the 3D points
	// *******************************************
	// R * [X; Y; Z] + T
	
	float x_tmp = x_world;
	float y_tmp = y_world;
	float z_tmp = z_world;
  x_world = (R1 * x_tmp) + (R2 * y_tmp) + (R3 * z_tmp) + T1;
	y_world = (R4 * x_tmp) + (R5 * y_tmp) + (R6 * z_tmp) + T2;
	z_world = (R7 * x_tmp) + (R8 * y_tmp) + (R9 * z_tmp) + T3;
	
	// *******************************************
	//   Project into the RGB coordinate frame.
	// *******************************************	
	float x_proj = x_world * FX_RGB / z_world + CX_RGB;
	float y_proj = y_world * FY_RGB / z_world + CY_RGB;
	
	// ************************************************
	// Finally, reassign the values in ROW MAJOR order.
	// ************************************************
	x = static_cast<int> (roundf(x_proj));
	y = static_cast<int> (roundf(y_proj));
	
	--x;
	--y;
	
	res_x[idx] = x;
	res_y[idx] = y;
}



namespace kinect {
  
// Projects the depth image onto the RGB image.
// Args:
//   depth_abs_h - depth matrix in row major order. This depth already should have
//               the nonlinearity removed as is measured in meters.
//   depth_proj_h - the projected depth matrix. The result is measured in meters.
//   
void project_depth_par(float* depth_abs_h, float* depth_proj_h) {
	
  int N = 480 * 640;
	
  // Copy the absolute depth values to the device:
	// 
	// depth_abs_h ==> depth_abs_d
	//
	float* depth_abs_d;
  cudaMalloc((void**) &depth_abs_d, N * sizeof(float));
  cudaMemcpy(depth_abs_d, depth_abs_h, N * sizeof(float), cudaMemcpyHostToDevice);
	
	// Allocate the output of the projection.
	int *proj_x_d, *proj_y_d;
	cudaMalloc((void**) &proj_x_d, N * sizeof(int));
	cudaMalloc((void**) &proj_y_d, N * sizeof(int));
	
  // Execute the parallel projection.
  int block_size = 512;
	int num_blocks = N / block_size + (N % block_size == 0 ? 0 : 1);

  depth_to_point_cloud<<<num_blocks, block_size>>>(depth_abs_d, proj_x_d, proj_y_d, N);
	
  // Copy the result back from the device.
	int *proj_x_h = (int*) malloc(N * sizeof(int));
	int *proj_y_h = (int*) malloc(N * sizeof(int));
	cudaMemcpy(proj_x_h, proj_x_d, N * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(proj_y_h, proj_y_d, N * sizeof(int), cudaMemcpyDeviceToHost);

  // Create the assignment matrix.
	float* depth_assgn = (float*) malloc(N * sizeof(float));
	for (int nn = 0; nn < N; ++nn) {
	  depth_assgn[nn] = 10;
	}

  // Now, go through the projection results:
	for (int i = 0; i < N; ++i) {
	  int x = proj_x_h[i];
		int y = proj_y_h[i];
		
	  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
	    continue;
	  }

    int abs_offset = y * WIDTH + x;
		
    if (depth_abs_h[i] > depth_assgn[abs_offset]) {
		  continue;
		}

    depth_proj_h[abs_offset] = depth_abs_h[i];
		depth_assgn[abs_offset] = depth_abs_h[i];
	}

	free(proj_x_h);
	free(proj_y_h);
	free(depth_assgn);
	
  // Cleanup.
  cudaFree(depth_abs_d);
	cudaFree(proj_x_d);
	cudaFree(proj_y_d);
}

}  // namespace kinect

