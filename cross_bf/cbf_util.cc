#include "cbf_util.h"
#include <iostream>

void downsample(image_type* src, image_type* dst) {
  // First, figure out the scale factor, assumed to be integer.
  int scale = src->width() / dst->width();

  for (int x = 0; x < dst->width(); ++x) {
    for (int y = 0; y < dst->height(); ++y) {
      dst->at(x, y) = src->at(x*scale, y*scale);
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
      dst->at(x, y) = src->at(ii, jj);
    }
  }
}

// 
void ReplaceGoodPixels(image_type* src, image_type* dst, bool* mask) {
  for (int x = 0; x < dst->width(); ++x) {
    for (int y = 0; y < dst->height(); ++y, ++mask) {
      if (!*mask) {
        dst->at(x, y) = src->at(x, y);
      }
    }
  }
}

// Copies the contents of the 'src' into the 'dst'.
void CopyInto(image_type* src, unsigned char* dst) {
  for (int x = 0; x < src->width(); ++x) {
    for (int y = 0; y < src->height(); ++y, ++dst) {
      *dst = static_cast<unsigned char>(255 * src->at(x, y));
    }
  }
}

void CopyInto(unsigned char* src, image_type* dst) {
  for (int x = 0; x < dst->width(); ++x) {
    for (int y = 0; y < dst->height(); ++y, ++src) {
      dst->at(x, y) = *src / 255.0;
    }
  }
}
