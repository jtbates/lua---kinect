#include "cbf.h"

#include <iostream>
#include <math.h>

#define NO_XML // This needs to go before fast_lbf.h
#include "fast_lbf.h"

#define HEIGHT 480
#define WIDTH 640

typedef Array_2D<double> image_type;

// Src is coming in at 0-255. 
// Dst should leave at 0-1
void downsample(uint8_t* src, int src_h, int src_w, image_type* dst) {
  // First, figure out the scale factor, assumed to be integer.
  int scale = src_w / dst->width();
	
  for (int y = 0; y < dst->height(); ++y) {
    for (int x = 0; x < dst->width(); ++x) {
      dst->at(x, y) = *(src+y*scale * src_w + x*scale) / 255.0;
    }
  }
}

void upsample(image_type* src, image_type* dst) {
  // First, figure out the scale factor, assumed to be integer.
  int scale = dst->width() / src->width();
  
  for (int x = 0; x < dst->width(); ++x) {
    for (int y = 0; y < dst->height(); ++y) {
      int ii = x / scale;
      int jj = y / scale;
			
			if (src->at(ii, jj) != src->at(ii, jj)) {
				dst->at(x, y) = 0;
			} else {
				dst->at(x, y) = src->at(ii, jj);
			}
    }
  }
}

// 
void FillInNoise(image_type* filt, uint8_t* orig, bool* mask) {
	int missing = 0;
	for (int y = 0; y < filt->height(); ++y) {
    for (int x = 0; x < filt->width(); ++x, ++orig, ++mask) {
			if (!*mask == 0) {
				*orig = static_cast<char>(round(255 * filt->at(x, y)));
			}
    }
  }
}

void kinect::cbf(uint8_t* depth, uint8_t* intensity, bool* mask,
              uint8_t* result, unsigned num_scales, double* sigma_s,
              double* sigma_r) {
	
	int height = HEIGHT;
	int width = WIDTH;
  
  // First, copy the input into results and we'll work from results.
  uint8_t* depth_p = depth;
  uint8_t* result_p = result;
  for (int i = 0; i < height * width; ++i) {
    *result_p++ = *depth_p++;
  }
  
  --num_scales;
	for (int ss = num_scales; ss >= 0; --ss) {
		
    // Calculate the size of the sampled images.
    int downHeight = static_cast<int>(height / pow((float)2, ss));
    int downWidth = static_cast<int>(width / pow((float)2, ss));
		
    // Downsample the depth.
    image_type dwn_bf_depth(downWidth, downHeight);
    downsample(result, height, width, &dwn_bf_depth);
    
    // Downsample the intensity image.
    image_type dwn_bf_intensity(downWidth, downHeight);
    downsample(intensity, height, width, &dwn_bf_intensity);
    
    // Process the Bilateral Filter.
    image_type dwn_img_filt(downWidth, downHeight);
    
    double space_sigma = sigma_s[num_scales - ss];
    double range_sigma = sigma_r[num_scales - ss];
	
    Image_filter::fast_LBF(dwn_bf_depth, dwn_bf_intensity, space_sigma, range_sigma, false,
													 &dwn_img_filt, &dwn_img_filt);
  
    // Now upsample the resulting image.
    image_type up_filt(width, height);
	
    upsample(&dwn_img_filt, &up_filt);
  	//upsample(&dwn_bf_intensity, &up_filt);  
	
    // Now that we've filtered the image, replace the original non-noisy
    // pixels with the good pixels.
    FillInNoise(&up_filt, result, mask);
  }
}
