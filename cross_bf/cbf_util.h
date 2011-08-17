
#ifndef CBF_UTIL_H_
#define CBF_UTIL_H_

#define NO_XML // This needs to go before fast_lbf.h
#include "array.h"

typedef Array_2D<double> image_type;

void downsample(image_type* src, image_type* dst);

void upsample(image_type* src, image_type* dst);

void CopyInto(image_type* src, unsigned char* dst);

void CopyInto(unsigned char* src, image_type* dst);

void ReplaceGoodPixels(image_type* src, image_type* dst, bool* mask);

#endif  // CBF_UTIL_H_
