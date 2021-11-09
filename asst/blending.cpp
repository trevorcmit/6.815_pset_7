#include "blending.h"
#include "matrix.h"
#include <ctime>

using namespace std;

Image blendingweight(int imwidth, int imheight) {
  // // --------- HANDOUT  PS07 ------------------------------
  Image output(imwidth, imheight, 1);

  for (int h = 0; h < imheight; h++) {
    for (int w = 0; w < imwidth; w++) {

      float x_weight, y_weight;

      if (h % 2 == 0) {
        int new_h;
        int middle = imheight / 2;
        float offset = middle + 1;
        float half_point = h + 0.5f;
        if (half_point < middle) {int new_h = floor(half_point);}
        else                     {int new_h = ceil(half_point);}
        int distance = abs(middle - new_h);
        y_weight = ((float)offset - (float)distance) / (float)offset;
      }
      else {
        int middle = floor(imheight / 2);
        int distance = abs(middle - h);
        y_weight = ((float)middle - (float)distance) / (float)middle;
      }

      if (w % 2 == 0) {
        int new_w;
        int middle = imwidth / 2;
        float offset = middle + 1;
        float half_point = w + 0.5f;
        if (half_point < middle) {int new_w = floor(half_point);}
        else                     {int new_w = ceil(half_point);}
        int distance = abs(middle - new_w);
        x_weight = ((float)offset - (float)distance) / (float)offset;
      }
      else {
        int middle = floor(imwidth / 2);
        int distance = abs(middle - w);
        x_weight = ((float)middle - (float)distance) / (float)middle;
      }

      output(w, h, 0) = x_weight * y_weight;
    }
  }
  return output;
}

//  ****************************************************************************
//  * blending related functions re-written from previous assignments
//  ****************************************************************************

// instead of writing source in out, *add* the source to out based on the weight, so out(x,y) = out(x, y) + weight * image
void applyhomographyBlend(const Image &source, const Image &weight, Image &out, const Matrix &H, bool bilinear) {
  // --------- HANDOUT  PS07 ------------------------------
  Matrix H_inv = H.inverse(); // Find inverse of H to use for calculations

  for (int h = 0; h < out.height(); h++) {
    for (int w = 0; w < out.width(); w++) {

      Matrix coords = Matrix::Zero(3, 1); // create column matrix [x y 1]
      coords << w, h, 1;

      Matrix xyw_prime = H_inv * coords; // Multiply H^-1 times output column matrix

      float x_prime = xyw_prime(0, 0) / xyw_prime(2, 0); // x'/w'
      float y_prime = xyw_prime(1, 0) / xyw_prime(2, 0); // y'/w'

      if ((x_prime < source.width() && x_prime > 0) &&   // Check bounds, only utilize if valid in source image
          (y_prime < source.height() && y_prime > 0)) {
        
        for (int c = 0; c < out.channels(); c++) { // Iterate for all channels, coords are the same
          if (bilinear) {
            out(w, h, c) = out(w, h, c) 
            + interpolateLin(weight, x_prime, y_prime, 0, true) * interpolateLin(source, x_prime, y_prime, c, true); 
          }
          else {
            out(w, h, c) = out(w, h, c) 
            + weight.smartAccessor(round(x_prime), round(y_prime), 0, true) *
              source.smartAccessor(round(x_prime), round(y_prime), c, true);
          }
        }
      }
    }
  }
}

Image stitchLinearBlending(const Image &im1, const Image &im2, const Image &we1, const Image &we2, const Matrix &H) {
  // --------- HANDOUT  PS07 ------------------------------
  // stitch using image weights, note there is no weight normalization.
  return Image(1, 1, 1);
}

/*****************************************************************************
 * blending functions Pset 08
 *****************************************************************************/

// low freq and high freq (2-scale decomposition)
vector<Image> scaledecomp(const Image &im, float sigma) {
  vector<Image> ims;
  ims.push_back(gaussianBlur_separable(im, sigma));
  ims.push_back(im - ims[0]);
  return ims;
}

// stitch using different blending models
// blend can be 0 (none), 1 (linear) or 2 (2-layer)
Image stitchBlending(const Image &im1, const Image &im2, const Matrix &H,
                     BlendType blend) {
  // --------- HANDOUT  PS07 ------------------------------
  return Image(1, 1, 1);
}

// auto stitch
Image autostitch(const Image &im1, const Image &im2, BlendType blend,
                 float blurDescriptor, float radiusDescriptor) {
  // --------- HANDOUT  PS07 ------------------------------
  return Image(1, 1, 1);
}

/************************************************************************
 * Tricks: mini planets.
 ************************************************************************/

Image pano2planet(const Image &pano, int newImSize, bool clamp) {
  // // --------- HANDOUT  PS07 ------------------------------
  return Image(newImSize, newImSize, 3);
}

/************************************************************************
 * 6.865: Stitch N images into a panorama
 ************************************************************************/

// Pset07-865. Compute sequence of N-1 homographies going from Im_i to Im_{i+1}
// Implement me!
vector<Matrix> sequenceHs(const vector<Image> &ims, float blurDescriptor,
                          float radiusDescriptor) {
  // // --------- HANDOUT  PS07 ------------------------------
  return vector<Matrix>();
}

// stack homographies:
//   transform a list of (N-1) homographies that go from I_i to I_i+1
//   to a list of N homographies going from I_i to I_refIndex.
vector<Matrix> stackHomographies(const vector<Matrix> &Hs, int refIndex) {
  // // --------- HANDOUT  PS07 ------------------------------
  return vector<Matrix>();
}

// Pset07-865: compute bbox around N images given one main reference.
BoundingBox bboxN(const vector<Matrix> &Hs, const vector<Image> &ims) {
  // // --------- HANDOUT  PS07 ------------------------------
  return BoundingBox(0, 0, 0, 0);
}

// Pset07-865.
// Implement me!
Image autostitchN(const vector<Image> &ims, int refIndex, float blurDescriptor,
                  float radiusDescriptor) {
  // // --------- HANDOUT  PS07 ------------------------------
  return Image(1, 1, 1);
}

/******************************************************************************
 * Helper functions
 *****************************************************************************/

// copy a single-channeled image to several channels
Image copychannels(const Image &im, int nChannels) {
  // image must have one channel
  assert(im.channels() == 1);
  Image oim(im.width(), im.height(), nChannels);

  for (int i = 0; i < im.width(); i++) {
    for (int j = 0; j < im.height(); j++) {
      for (int c = 0; c < nChannels; c++) {
        oim(i, j, c) = im(i, j);
      }
    }
  }
  return oim;
}
