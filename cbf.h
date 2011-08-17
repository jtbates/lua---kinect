/* Alternative entry point for the Cross Bilateral Filter.
 */

#ifndef CBF_H_
#define CBF_H_

#include <stdint.h>

namespace kinect {

// Filters the given depth image using the Cross Bilateral Filter. The results
//
// Args:
//   depth - HxW row-major ordered matrix.
//   intensity - HxW row-major ordered matrix.
//   mask - HxW row-major ordered matrix.
//   result - HxW row-major ordered matrix.
//   num_scales - the number of scales at which to perform the filtering.
//   sigma_s - the space sigma (in pixels)
//   sigma_r - the range sigma (in intensity values, 0-1)
void cbf(uint8_t* depth, uint8_t* intensity, bool* mask,
         uint8_t* result, unsigned num_scales, double* sigma_s,
         double* sigma_r);

}	 // namespace
#endif  // CBF_H_
