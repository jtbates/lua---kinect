/* Provides functions for preprocessing the  images from the KINECT.
 */

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include <stdint.h>

namespace kinect {
	
// Preprocesses the depth images by projecting the depth image onto the
// RGB image and then filtering the result.
//
// Args:
//   depth - Depth matrix in row major order. This depth is raw from the kinect and must
//           be flipped before being used.
//   rgb - the RGB image, HxWx3
//   depth_out - the output matrix. The result is projected, smoothed and scaled
//               by the maximum depth.
void preprocess_depth(uint16_t* depth, uint8_t* rgb, uint8_t* depth_out);

// Preprocesses the depth images by projecting the depth image onto the
// RGB image and then filtering the result.
//
// Args:
//   depth - Depth matrix in row major order. This depth is raw from the kinect and must
//           be flipped before being used.
//   rgb - the RGB image, HxWx3
//   depth_out - the output matrix. The result is projected, smoothed and scaled
//               by the maximum depth.
void preprocess_depth_par(uint16_t* depth, uint8_t* rgb, uint8_t* depth_out);
	
// Projects the depth image onto the RGB image.
// Args:
//   depth_abs - depth matrix in row major order. This depth already should have
//               the nonlinearity removed as is measured in meters.
//   depth_proj - the projected depth matrix. The result is measured in meters.
void project_depth(double* depth_abs, double* depth_proj);

// Projects the depth image onto the RGB image.
// Args:
//   depth_abs - depth matrix in row major order. This depth already should have
//               the nonlinearity removed as is measured in meters.
//   depth_proj - the projected depth matrix. The result is measured in meters.
void project_depth_par(float* depth_abs, float* depth_proj);
	
// Swaps the endianness of the image
void swap_bytes(uint16_t* vec, int length);

// Removes the depth nonlinearity using the constants learned from fitting
// the depth values.
//
// Args:
//   src - the depth image, in row major order, containing a non-linearity
//         in its depth values.
//   dst - the depth image, in row major order, in absolute depth, measured
//         in meters.
void remove_depth_nonlinearity(uint16_t* src, double* dst);

// Removes the depth nonlinearity using the constants learned from fitting
// the depth values.
//
// Args:
//   src - the depth image, in row major order, containing a non-linearity
//         in its depth values.
//   dst - the depth image, in row major order, in absolute depth, measured
//         in meters.
void remove_depth_nonlinearity(uint16_t* src, float* dst);
	
// Scales the depth image
//
// Args:
//   depth - the depth image, in row major order.
//   depth_out - the depth image, after its been scaled.
void scale_depth(double* src, uint8_t* dst);
	
// Scales the depth image
//
// Args:
//   depth - the depth image, in row major order.
//   depth_out - the depth image, after its been scaled.
void scale_depth(float* src, uint8_t* dst);
}  // namespace

#endif  // PREPROCESS_H_
