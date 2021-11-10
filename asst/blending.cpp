#include "blending.h"
#include "matrix.h"
#include <ctime>

using namespace std;

Image blendingweight(int imwidth, int imheight) {
  // // --------- HANDOUT  PS07 ------------------------------
  Image output(imwidth, imheight, 1);      // Initialize output image
  float middle_w = imwidth / 2.0f;         // Get centers in both X and Y directions
  float middle_h = imheight / 2.0f;
  for (int h = 0; h < imheight; h++) {
    for (int w = 0; w < imwidth; w++) {
      float weight_w = (1 - fabs(middle_w - w) / middle_w); // Get instantaneous weight in each direction
      float weight_h = (1 - fabs(middle_h - h) / middle_h);
      output(w, h, 0) = weight_w * weight_h;                // Set output weight to product of directional weights
    }
  }
  return output; // Return output weight image
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
            out(w, h, c) = out(w, h, c) // Use interpolation for multiplying by weight
            + interpolateLin(weight, x_prime, y_prime, 0, true) * interpolateLin(source, x_prime, y_prime, c, true); 
          }
          else {
            out(w, h, c) = out(w, h, c)  // Multiply source value by weight value without interpolation
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
  // Stitch using image weights, note there is no weight normalization.
  BoundingBox bbox = bboxUnion(
    computeTransformedBBox(im1.width(), im1.height(), H), // Bounding box for applying homography to im1
    computeTransformedBBox(im2.width(), im2.height(), Matrix::Identity(3,3)) // Add image 2 size
  );
  Matrix translate = makeTranslation(bbox); // Get translate based on combination of both boxes

  Image output(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, im2.channels()); // Create output
  output.set_color(); // Set to black so pixels can be adjusted by applyHomography

  applyhomographyBlend(im1, we1, output, translate * H, true); // translation and homography on image 1
  applyhomographyBlend(im2, we2, output, translate, true);     // solely translation on image 2
  return output;
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
Image stitchBlending(const Image &im1, const Image &im2, const Matrix &H, BlendType blend) {
  // --------- HANDOUT  PS07 ------------------------------

  BoundingBox bbox = bboxUnion(
      computeTransformedBBox(im1.width(), im1.height(), H), // Bounding box for applying homography to im1
      computeTransformedBBox(im2.width(), im2.height(), Matrix::Identity(3,3)) // Add image 2 size
  );
  Matrix translate = makeTranslation(bbox); // Get translate based on combination of both boxes
  Image output(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, im2.channels()); // Create output
  output.set_color(); // Set to black so pixels can be adjusted by applyHomography

  if (blend == BlendType::BLEND_NONE) {
    applyHomography(im1, translate * H, output, true); // translation and homography on image 1
    applyHomography(im2, translate, output, true);     // solely translation on image 2
    return output;
  }

  else if (blend == BlendType::BLEND_LINEAR) {
    // Image we1 = blendingweight(im1.width(), im1.height());
    // Image we2 = blendingweight(im2.width(), im2.height());
    Image smooth1 = blendingweight(im1.width(), im1.height());
    Image smooth2 = blendingweight(im2.width(), im2.height());
    Image ones1(im1.width(), im1.height());
    Image ones2(im2.width(), im2.height());
    ones1.set_color(1.0f);
    ones2.set_color(1.0f);
    Image low_weight(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1);
    applyhomographyBlend(smooth1, ones1, low_weight, H);
    applyhomographyBlend(smooth2, ones2, low_weight, H);
    Image image1 = im1 / low_weight.max();
    Image image2 = im2 / low_weight.max();

    applyhomographyBlend(image1, smooth1, output, translate * H, true); // translation and homography on image 1
    applyhomographyBlend(image2, smooth2, output, translate, true);     // solely translation on image 2
    return output;
  }

  else { // BLEND_2LAYER case
    Image low1 = gaussianBlur_separable(im1, 2.0), low2 = gaussianBlur_separable(im2, 2.0);
    Image high1 = im1 - low1, high2 = im2 - low2;


    // *********************************
    // ********* LOW FREQUENCY *********
    // *********************************
    Image smooth1 = blendingweight(im1.width(), im1.height());
    Image smooth2 = blendingweight(im2.width(), im2.height());
    Image ones1(im1.width(), im1.height());
    Image ones2(im2.width(), im2.height());
    ones1.set_color(1.0f);
    ones2.set_color(1.0f);
    Image low_weight(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1);
    applyhomographyBlend(smooth1, ones1, low_weight, H);
    applyhomographyBlend(smooth2, ones2, low_weight, H);


    // **********************************
    // ********* HIGH FREQUENCY *********
    // **********************************
    Image high_freq(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1);   // Initialize high frequency output image
    Image high_weight(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1); // Initialize high frequency weights
    high_freq.set_color(0.0f);
    high_weight.set_color(0.0f);

    Image im_high1(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1); // Create two stitch-sized images with only original images
    Image im_high2(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1);
    applyHomography(high1, translate * H, im_high1, true); 
    applyHomography(high2, translate, im_high2, true);     

    Image sw1(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1); // Create two stitch-sized images with only weights
    Image sw2(bbox.x2 - bbox.x1, bbox.y2 - bbox.y1, 1);
    sw1.set_color(-1.0f);
    sw2.set_color(-1.0f);
    applyHomography(smooth1, H, sw1);
    applyHomography(smooth2, H, sw2);

    for (int h = 0; h < high_freq.height(); h++) {   // Iterate over all pixels to generate high frequency
      for (int w = 0; w < high_freq.width(); w++) {  // stitch utilizing the abrupt weighting system
        if (sw1(w, h) > 0 && sw2(w, h) > 0) {
          if (sw1(w, h) > sw2(w, h)) {
            high_weight(w, h) = sw1(w, h);
            high_freq(w, h) = sw1(w, h) * im_high1(w, h);
          }
          else {
            high_weight(w, h) = sw2(w, h);
            high_freq(w, h) = sw2(w, h) * im_high2(w, h);
          }
        }
        else if (sw1(w, h) > 0) {
          high_weight(w, h) = sw1(w, h);
          high_freq(w, h) = sw1(w, h) * im_high1(w, h);
        }
        else if (sw2(w, h) > 0) {
          high_weight(w, h) = sw2(w, h);
          high_freq(w, h) = sw2(w, h) * im_high2(w, h);
        }
      }
    }

    // Image weight_sum = low_weight + high_weight;

    

  }

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
Image autostitchN(const vector<Image> &ims, int refIndex, float blurDescriptor, float radiusDescriptor) {
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
