#include "mex.h"

#include <math.h>

#define NO_XML // This needs to go before fast_lbf.h
#include "fast_lbf.h"
#include "cbf_util.h"

#define ARG_IMG_DEPTH 0
#define ARG_IMG_EDGE 1
#define ARG_IMG_MASK 2
#define ARG_SPACE_SIGMA 3
#define ARG_RANGE_SIGMA 4

using namespace std;

typedef Array_2D<double> image_type;

// Args:
//   imgDepth - 2 dimensional depth image.
//   imgEdges - 2 dimensional edge image.
//   imgMask - 2D mask image (binary)
//   spaceSigma - Sx1
//   rangeSigma - Sx1
void mexFunction(int nlhs, mxArray* plhs[], const int nrhs, const mxArray* prhs[]) {
  if (nrhs != 5) {
    mexErrMsgTxt("Wrong number of arguments! 5 Required.");
  }
  
  // *****************************************************
  // Convert the Depth image to the required 'image_type'.
  // *****************************************************
  mxClassID img_depth_class_id = mxGetClassID(prhs[ARG_IMG_DEPTH]);
  if (img_depth_class_id != mxUINT8_CLASS) {
    mexErrMsgTxt("imgDepth must be UINT8");
  }
  
  const mwSize img_depth_num_dims = mxGetNumberOfDimensions(prhs[ARG_IMG_DEPTH]);
  if (img_depth_num_dims != 2) {
    mexErrMsgTxt("imgDepth must be 2-dimensional.");
  }
  
  const mwSize* img_depth_dims = mxGetDimensions(prhs[ARG_IMG_DEPTH]);
  
  int height = mxGetM(prhs[ARG_IMG_DEPTH]);
  int width = mxGetN(prhs[ARG_IMG_DEPTH]);

  unsigned char* p_img_depth = (unsigned char*) mxGetData(prhs[ARG_IMG_DEPTH]);
  image_type img_depth(width, height);
  CopyInto(p_img_depth, &img_depth);
  
  // ****************************************************
  // Convert the Edge image to the required 'image_type'.
  // ****************************************************
  mxClassID img_edge_class_id = mxGetClassID(prhs[ARG_IMG_EDGE]);
  if (img_edge_class_id != mxUINT8_CLASS) {
    mexErrMsgTxt("imgEdge must be UINT8");
  }
  
  const mwSize img_edge_num_dims = mxGetNumberOfDimensions(prhs[ARG_IMG_EDGE]);
  if (img_edge_num_dims != 2) {
    mexErrMsgTxt("imgEdge must be 2-dimensional.");
  }
  
  const mwSize* img_edge_dims = mxGetDimensions(prhs[ARG_IMG_EDGE]);
  
  if (mxGetM(prhs[ARG_IMG_EDGE]) != height) {
    mexErrMsgTxt("Edge image is of the wrong height");
  }
  
  if (mxGetN(prhs[ARG_IMG_EDGE]) != width) {
    mexErrMsgTxt("Edge image is of the wrong width");
  }
  
  unsigned char* p_img_edge = (unsigned char*) mxGetData(prhs[ARG_IMG_EDGE]);
  image_type img_edge(width, height);
  CopyInto(p_img_edge, &img_edge);
  
  // ****************************************************
  // Convert the Mask image to the required 'image_type'.
  // ****************************************************
  mxClassID img_mask_class_id = mxGetClassID(prhs[ARG_IMG_MASK]);
  if (img_mask_class_id != mxLOGICAL_CLASS) {
    mexErrMsgTxt("imgMask must be mxLOGICAL_CLASS");
  }
  
  const mwSize img_mask_num_dims = mxGetNumberOfDimensions(prhs[ARG_IMG_MASK]);
  if (img_mask_num_dims != 2) {
    mexErrMsgTxt("imgMask must be 2-dimensional.");
  }
  
  const mwSize* img_mask_dims = mxGetDimensions(prhs[ARG_IMG_MASK]);
  
  if (mxGetM(prhs[ARG_IMG_MASK]) != height) {
    mexErrMsgTxt("Mask image is of the wrong height");
  }
  
  if (mxGetN(prhs[ARG_IMG_MASK]) != width) {
    mexErrMsgTxt("Mask image is of the wrong width");
  }
  
  bool* img_mask = (bool*) mxGetData(prhs[ARG_IMG_MASK]);
  
  // **************************
  // Grab the range parameters.
  // **************************
  int num_scales = mxGetNumberOfElements(prhs[ARG_SPACE_SIGMA]);
  if (mxGetNumberOfElements(prhs[ARG_SPACE_SIGMA]) != num_scales) {
    mexErrMsgTxt("Mismatch in the number of space and range sigma");
  }
  
  double* sigma_s = (double*) mxGetData(prhs[ARG_SPACE_SIGMA]);
  double* sigma_r = (double*) mxGetData(prhs[ARG_RANGE_SIGMA]);

  // ******************
  // Create the Pyramid
  // ******************
  --num_scales;
  for (int ss = num_scales; ss >= 0; --ss) { 
  
    // Calculate the size of the sampled images.
    int downHeight = static_cast<int>(height / pow((float)2, ss));
    int downWidth = static_cast<int>(width / pow((float)2, ss));
  
    // Downsample the depth.
    image_type dwn_img_depth(downWidth, downHeight);  
    downsample(&img_depth, &dwn_img_depth);
    
    // Downsample the edge image.
    image_type dwn_img_edge(downWidth, downHeight);
    downsample(&img_edge, &dwn_img_edge);
    
    // Process the Bilateral Filter.
    image_type dwn_img_filt(downWidth, downHeight);
    
    double space_sigma = sigma_s[num_scales - ss];
    double range_sigma = sigma_r[num_scales - ss];

    Image_filter::fast_LBF(dwn_img_depth, dwn_img_edge, space_sigma, range_sigma, false,
            &dwn_img_filt, &dwn_img_filt);
    
    // Now upsample the resulting image.
    image_type img_filt(width, height);
    upsample(&dwn_img_filt, &img_filt);
    
    // Now that we've filtered the image, replace the original non-noisy
    // pixels with the good pixels.
    ReplaceGoodPixels(&img_depth, &img_filt, img_mask);
    
    img_depth = img_filt;
  }
  
  plhs[0] = mxCreateNumericArray(img_depth_num_dims, &img_depth_dims[0], mxUINT8_CLASS, mxREAL);
  unsigned char* output = (unsigned char*) mxGetData(plhs[0]);
  CopyInto(&img_depth, output);
}
