// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image stuff
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

// Logging stuff
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = boost::filesystem;

typedef struct {
  size_t offset_x;
  size_t offset_y;
  std::string img_path;
  std::string mask_path;
  std::string weights_path;
} ConfigView;

bool feathering(aliceVision::image::Image<image::RGBfColor> & output, const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask) {

  std::vector<image::Image<image::RGBfColor>> feathering;
  std::vector<image::Image<unsigned char>> feathering_mask;
  feathering.push_back(color);
  feathering_mask.push_back(inputMask);

  int lvl = 0;
  int width = color.Width();
  int height = color.Height();
  
  while (1) {
    const image::Image<image::RGBfColor> & src = feathering[lvl];
    const image::Image<unsigned char> & src_mask = feathering_mask[lvl];
  
    image::Image<image::RGBfColor> half(width / 2, height / 2);
    image::Image<unsigned char> half_mask(width / 2, height / 2);

    for (int i = 0; i < half.Height(); i++) {

      int di = i * 2;
      for (int j = 0; j < half.Width(); j++) {
        int dj = j * 2;

        int count = 0;
        half(i, j) = image::RGBfColor(0.0,0.0,0.0);
        
        if (src_mask(di, dj)) {
          half(i, j) += src(di, dj);
          count++;
        }

        if (src_mask(di, dj + 1)) {
          half(i, j) += src(di, dj + 1);
          count++;
        }

        if (src_mask(di + 1, dj)) {
          half(i, j) += src(di + 1, dj);
          count++;
        }

        if (src_mask(di + 1, dj + 1)) {
          half(i, j) += src(di + 1, dj + 1);
          count++;
        }

        if (count > 0) {
          half(i, j) /= float(count);
          half_mask(i, j) = 1;
        } 
        else {
          half_mask(i, j) = 0;
        }
      }

      
    }

    feathering.push_back(half);
    feathering_mask.push_back(half_mask);

    
    width = half.Width();
    height = half.Height();

    if (width < 2 || height < 2) break;

    lvl++;  
  }


  for (int lvl = feathering.size() - 2; lvl >= 0; lvl--) {
    
    image::Image<image::RGBfColor> & src = feathering[lvl];
    image::Image<unsigned char> & src_mask = feathering_mask[lvl];
    image::Image<image::RGBfColor> & ref = feathering[lvl + 1];
    image::Image<unsigned char> & ref_mask = feathering_mask[lvl + 1];

    for (int i = 0; i < src_mask.Height(); i++) {
      for (int j = 0; j < src_mask.Width(); j++) {
        if (!src_mask(i, j)) {
          int mi = i / 2;
          int mj = j / 2;

          if (mi >= ref_mask.Height()) {
            mi = ref_mask.Height() - 1;
          }

          if (mj >= ref_mask.Width()) {
            mj = ref_mask.Width() - 1;
          }

          src_mask(i, j) = ref_mask(mi, mj);
          src(i, j) = ref(mi, mj);
        }
      }
    }
  }

  output = feathering[0];

  return true;
}

void drawBorders(aliceVision::image::Image<image::RGBAfColor> & inout, aliceVision::image::Image<unsigned char> & mask, size_t offset_x, size_t offset_y) {

  
  for (int i = 0; i < mask.Height(); i++) {
    int j = 0;
    int di = i + offset_y;
    int dj = j + offset_x;
    if (dj >= inout.Width()) {
      dj = dj - inout.Width();
    }

    if (mask(i, j)) {
      inout(di, dj) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
    }
  }

  for (int i = 0; i < mask.Height(); i++) {
    int j = mask.Width() - 1;
    int di = i + offset_y;
    int dj = j + offset_x;
    if (dj >= inout.Width()) {
      dj = dj - inout.Width();
    }

    if (mask(i, j)) {
      inout(di, dj) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
    }
  }

  for (int j = 0; j < mask.Width(); j++) {
    int i = 0;
    int di = i + offset_y;
    int dj = j + offset_x;
    if (dj >= inout.Width()) {
      dj = dj - inout.Width();
    }

    if (mask(i, j)) {
      inout(di, dj) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
    }
  }

  for (int j = 0; j < mask.Width(); j++) {
    int i = mask.Height() - 1;
    int di = i + offset_y;
    int dj = j + offset_x;
    if (dj >= inout.Width()) {
      dj = dj - inout.Width();
    }

    if (mask(i, j)) {
      inout(di, dj) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
    }
  }
  
  for (int i = 1; i < mask.Height() - 1; i++) {

    int di = i + offset_y;

    for (int j = 1; j < mask.Width() - 1; j++) {

      int dj = j + offset_x;
      if (dj >= inout.Width()) {
        dj = dj - inout.Width();
      }

      if (!mask(i, j)) continue;

      unsigned char others = true;
      others &= mask(i - 1, j - 1);
      others &= mask(i - 1, j + 1);
      others &= mask(i, j - 1);
      others &= mask(i, j + 1);
      others &= mask(i + 1, j - 1);
      others &= mask(i + 1, j + 1);
      if (others) continue;

      inout(di, dj) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
    }
  }
}

void drawSeams(aliceVision::image::Image<image::RGBAfColor> & inout, aliceVision::image::Image<IndexT> & labels) {

  for (int i = 1; i < labels.Height() - 1; i++) {

    for (int j = 1; j < labels.Width() - 1; j++) {

      IndexT label = labels(i, j);
      IndexT same = true;

      same &= (labels(i - 1, j - 1) == label);
      same &= (labels(i - 1, j + 1) == label);
      same &= (labels(i, j - 1) == label);
      same &= (labels(i, j + 1) == label);
      same &= (labels(i + 1, j - 1) == label);
      same &= (labels(i + 1, j + 1) == label);

      if (same) {
        continue;
      }

      inout(i, j) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
    }
  }
}

void getMaskFromLabels(aliceVision::image::Image<float> & mask, aliceVision::image::Image<IndexT> & labels, IndexT index, size_t offset_x, size_t offset_y) {

  for (int i = 0; i < mask.Height(); i++) {

    int di = i + offset_y;

    for (int j = 0; j < mask.Width(); j++) {

      int dj = j + offset_x;
      if (dj >= labels.Width()) {
        dj = dj - labels.Width();
      }


      if (labels(di, dj) == index) {
        mask(i, j) = 1.0f;
      }
      else {
        mask(i, j) = 0.0f;
      }
    }
  }
}

template <class T>
bool makeImagePyramidCompatible(image::Image<T> & output, size_t & out_offset_x, size_t & out_offset_y, const image::Image<T> & input, size_t offset_x, size_t offset_y, size_t num_levels) {

  if (num_levels == 0) {
    return false;
  }

  double max_scale = 1.0 / pow(2.0, num_levels - 1);

  double low_offset_x = double(offset_x) * max_scale;
  double low_offset_y = double(offset_y) * max_scale;

  /*Make sure offset is integer even at the lowest level*/
  double corrected_low_offset_x = floor(low_offset_x);
  double corrected_low_offset_y = floor(low_offset_y);

  /*Add some borders on the top and left to make sure mask can be smoothed*/
  corrected_low_offset_x = std::max(0.0, corrected_low_offset_x - 3.0);
  corrected_low_offset_y = std::max(0.0, corrected_low_offset_y - 3.0);

  /*Compute offset at largest level*/
  out_offset_x = size_t(corrected_low_offset_x / max_scale);
  out_offset_y = size_t(corrected_low_offset_y / max_scale);

  /*Compute difference*/
  double doffset_x = double(offset_x) - double(out_offset_x);
  double doffset_y = double(offset_y) - double(out_offset_y);
  
  /* update size with border update */
  double large_width = double(input.Width()) + doffset_x;
  double large_height = double(input.Height()) + doffset_y;
  
  /* compute size at largest scale */
  double low_width = large_width * max_scale;
  double low_height = large_height * max_scale;

  /*Make sure width is integer event at the lowest level*/
  double corrected_low_width = ceil(low_width);
  double corrected_low_height = ceil(low_height);  

  /*Add some borders on the right and bottom to make sure mask can be smoothed*/
  corrected_low_width = corrected_low_width + 3;
  corrected_low_height = corrected_low_height + 3;

  /*Compute size at largest level*/
  size_t width = size_t(corrected_low_width / max_scale);
  size_t height = size_t(corrected_low_height / max_scale);

  output = image::Image<T>(width, height, true, T(0.0f));
  output.block(doffset_y, doffset_x, input.Height(), input.Width()) = input;

  return true;
}


class Compositer {
public:
  Compositer(size_t outputWidth, size_t outputHeight) :
  _panorama(outputWidth, outputHeight, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f))
  {
  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    for (size_t i = 0; i < color.Height(); i++) {

      size_t pano_i = offset_y + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < color.Width(); j++) {

        if (!inputMask(i, j)) {
          continue;
        }

        size_t pano_j = offset_x + j;
        if (pano_j >= _panorama.Width()) {
          pano_j = pano_j - _panorama.Width();
        }

        _panorama(pano_i, pano_j).r() = color(i, j).r();
        _panorama(pano_i, pano_j).g() = color(i, j).g();
        _panorama(pano_i, pano_j).b() = color(i, j).b();
        _panorama(pano_i, pano_j).a() = 1.0f;
      }
    }

    return true;
  }

  virtual bool terminate() {
    return true;
  }

  aliceVision::image::Image<image::RGBAfColor> & getPanorama() {
    return _panorama;
  }

protected:
  aliceVision::image::Image<image::RGBAfColor> _panorama;
};

class AlphaCompositer : public Compositer {
public:

  AlphaCompositer(size_t outputWidth, size_t outputHeight) :
  Compositer(outputWidth, outputHeight) {

  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    for (size_t i = 0; i < color.Height(); i++) {

      size_t pano_i = offset_y + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < color.Width(); j++) {

        if (!inputMask(i, j)) {
          continue;
        }

        size_t pano_j = offset_x + j;
        if (pano_j >= _panorama.Width()) {
          pano_j = pano_j - _panorama.Width();
        }

        float wc = inputWeights(i, j);
          
        _panorama(pano_i, pano_j).r() += wc * color(i, j).r();
        _panorama(pano_i, pano_j).g() += wc * color(i, j).g();
        _panorama(pano_i, pano_j).b() += wc * color(i, j).b();
        _panorama(pano_i, pano_j).a() += wc;
      }
    }

    return true;
  }

  virtual bool terminate() {

    for (int i = 0; i  < _panorama.Height(); i++) {
      for (int j = 0; j < _panorama.Width(); j++) {
        
        if (_panorama(i, j).a() < 1e-6) {
          _panorama(i, j).r() = 1.0f;
          _panorama(i, j).g() = 0.0f;
          _panorama(i, j).b() = 0.0f;
          _panorama(i, j).a() = 0.0f;
        }
        else {
          _panorama(i, j).r() = _panorama(i, j).r() / _panorama(i, j).a();
          _panorama(i, j).g() = _panorama(i, j).g() / _panorama(i, j).a();
          _panorama(i, j).b() = _panorama(i, j).b() / _panorama(i, j).a();
          _panorama(i, j).a() = 1.0f;
        }
      }
    }

    return true;
  }
};

template<class T>
inline void convolveRow(typename image::Image<T>::RowXpr output_row, typename image::Image<T>::ConstRowXpr input_row, const Eigen::Matrix<float, 5, 1> & kernel, bool loop) {

  const int radius = 2;

  for (int j = 0; j < input_row.cols(); j++) {

    T sum = T();
    float sumw = 0.0f;

    for (int k = 0; k < kernel.size(); k++) {

      float w = kernel(k);
      int col = j + k - radius;

      /* mirror 5432 | 123456 | 5432 */

      if (!loop) {
        if (col < 0) {
          col = - col;
        }

        if (col >= input_row.cols()) {
          col = input_row.cols() - 1 - (col + 1 - input_row.cols());
        }
      }
      else {
        if (col < 0) {
          col = input_row.cols() + col;
        }

        if (col >= input_row.cols()) {
          col = col - input_row.cols();
        }
      }

      sum += w * input_row(col);
      sumw += w;
    }

    output_row(j) = sum / sumw;
  }
}

template<class T>
inline void convolveColumns(typename image::Image<T>::RowXpr output_row, const image::Image<T> & input_rows, const Eigen::Matrix<float, 5, 1> & kernel) {
  
  for (int j = 0; j < output_row.cols(); j++) {

    T sum = T();
    float sumw = 0.0f;

    for (int k = 0; k < kernel.size(); k++) {

      float w = kernel(k);
      sum += w * input_rows(k, j);
      sumw += w;
    }

    output_row(j) = sum / sumw;
  }
}

template<class T>
bool convolveGaussian5x5(image::Image<T> & output, const image::Image<T> & input, bool loop = false) {

  if (output.size() != input.size()) {
    return false;
  }

  Eigen::Matrix<float, 5, 1> kernel;
  kernel[0] = 1.0f;
  kernel[1] = 4.0f;
  kernel[2] = 6.0f;
  kernel[3] = 4.0f;
  kernel[4] = 1.0f;
  kernel = kernel / kernel.sum();

  image::Image<T> buf(output.Width(), 5);

  int radius = 2;

  convolveRow<T>(buf.row(0), input.row(2), kernel, loop);
  convolveRow<T>(buf.row(1), input.row(1), kernel, loop);
  convolveRow<T>(buf.row(2), input.row(0), kernel, loop);
  convolveRow<T>(buf.row(3), input.row(1), kernel, loop);
  convolveRow<T>(buf.row(4), input.row(2), kernel, loop);

  for (int i = 0; i < output.Height() - 3; i++) {

    convolveColumns<T>(output.row(i), buf, kernel);


    buf.row(0) = buf.row(1);
    buf.row(1) = buf.row(2);
    buf.row(2) = buf.row(3);
    buf.row(3) = buf.row(4);
    convolveRow<T>(buf.row(4), input.row(i + 3), kernel, loop);
  }

  /**
  current row : -5 -4 -3 -2 -1
  next 1 : -4 -3 -2 -1 -2
  next 2 : -3 -2 -1 -2 -3
  */
  convolveColumns<T>(output.row(output.Height() - 3), buf, kernel);

  buf.row(0) = buf.row(1);
  buf.row(1) = buf.row(2);
  buf.row(2) = buf.row(3);
  buf.row(3) = buf.row(4);
  convolveRow<T>(buf.row(4), input.row(output.Height() - 2), kernel, loop);
  convolveColumns<T>(output.row(output.Height() - 2), buf, kernel);

  buf.row(0) = buf.row(1);
  buf.row(1) = buf.row(2);
  buf.row(2) = buf.row(3);
  buf.row(3) = buf.row(4);
  convolveRow<T>(buf.row(4), input.row(output.Height() - 3), kernel, loop);
  convolveColumns<T>(output.row(output.Height() - 1), buf, kernel);

  return true;
}


template <class T>
bool downscale(aliceVision::image::Image<T> & outputColor, const aliceVision::image::Image<T> & inputColor) {

  size_t output_width = inputColor.Width() / 2;
  size_t output_height = inputColor.Height() / 2;

  for (int i = 0; i < output_height; i++) {
    for (int j = 0; j < output_width; j++) {
      outputColor(i, j) = inputColor(i * 2, j * 2);
    }
  }

  return true;
}

template <class T>
bool upscale(aliceVision::image::Image<T> & outputColor, const aliceVision::image::Image<T> & inputColor) {

  size_t width = inputColor.Width();
  size_t height = inputColor.Height();

  for (int i = 0; i < height; i++) {

    int di = i * 2;

    for (int j = 0; j < width; j++) {
      int dj = j * 2;

      outputColor(di, dj) = T();
      outputColor(di, dj + 1) = T();
      outputColor(di + 1, dj) = T();
      outputColor(di + 1, dj + 1) = inputColor(i, j);
    }
  }

  return true;
}

template <class T>
bool substract(aliceVision::image::Image<T> & AminusB, const aliceVision::image::Image<T> & A, const aliceVision::image::Image<T> & B) {

  size_t width = AminusB.Width();
  size_t height = AminusB.Height();

  if (AminusB.size() != A.size()) {
    return false;
  }

  if (AminusB.size() != B.size()) {
    return false;
  }

  for (int i = 0; i < height; i++) {

    for (int j = 0; j < width; j++) {

      AminusB(i, j) = A(i, j) - B(i, j);
    }
  }

  return true;
}

template <class T>
bool addition(aliceVision::image::Image<T> & AplusB, const aliceVision::image::Image<T> & A, const aliceVision::image::Image<T> & B) {

  size_t width = AplusB.Width();
  size_t height = AplusB.Height();

  if (AplusB.size() != A.size()) {
    return false;
  }

  if (AplusB.size() != B.size()) {
    return false;
  }

  for (int i = 0; i < height; i++) {

    for (int j = 0; j < width; j++) {

      AplusB(i, j) = A(i, j) + B(i, j);
    }
  }

  return true;
}

void removeNegativeValues(aliceVision::image::Image<image::RGBfColor> & img) {
  for (int i = 0; i < img.Height(); i++) {
    for (int j = 0; j < img.Width(); j++) {
      image::RGBfColor & pix = img(i, j);
      image::RGBfColor rpix;
      rpix.r() = std::exp(pix.r());
      rpix.g() = std::exp(pix.g());
      rpix.b() = std::exp(pix.b());

      if (rpix.r() < 0.0) {
        pix.r() = 0.0;
      }

      if (rpix.g() < 0.0) {
        pix.g() = 0.0;
      }

      if (rpix.b() < 0.0) {
        pix.b() = 0.0;
      }
    }
  }
}

class LaplacianPyramid {
public:
  LaplacianPyramid(size_t base_width, size_t base_height, size_t max_levels) {

    size_t width = base_width;
    size_t height = base_height;

    /*Make sure pyramid size can be divided by 2 on each levels*/
    double max_scale = 1.0 / pow(2.0, max_levels - 1);
    //width = size_t(ceil(double(width) * max_scale) / max_scale);
    //height = size_t(ceil(double(height) * max_scale) / max_scale);

    /*Prepare pyramid*/
    for (int lvl = 0; lvl < max_levels; lvl++) {

      _levels.push_back(aliceVision::image::Image<image::RGBfColor>(width, height, true, image::RGBfColor(0.0f,0.0f,0.0f)));
      _weights.push_back(aliceVision::image::Image<float>(width, height, true, 0.0f));
      
      height /= 2;
      width /= 2;
    }
  }

  bool augment(size_t new_max_levels) {

    if (new_max_levels <= _levels.size()) {
      return false;
    }

    size_t old_max_level = _levels.size();

    image::Image<image::RGBfColor> current_color = _levels[_levels.size() - 1];
    image::Image<float> current_weights = _weights[_weights.size() - 1];

    _levels[_levels.size() - 1].fill(image::RGBfColor(0.0f, 0.0f, 0.0f));
    _weights[_weights.size() - 1].fill(0.0f);

    image::Image<unsigned char> current_mask(current_color.Width(), current_color.Height(), true, 0);
    image::Image<image::RGBfColor> current_color_feathered(current_color.Width(), current_color.Height());

    for (int i = 0; i < current_color.Height(); i++) {
      for (int j = 0; j < current_color.Width(); j++) {
        if (current_weights(i, j) < 1e-6) {
          current_color(i, j) = image::RGBfColor(0.0);
          continue;  
        }
        
        current_color(i, j).r() = current_color(i, j).r() / current_weights(i, j);
        current_color(i, j).g() = current_color(i, j).g() / current_weights(i, j);
        current_color(i, j).b() = current_color(i, j).b() / current_weights(i, j);
        current_mask(i ,j) = 255;
      }
    }


    feathering(current_color_feathered, current_color, current_mask);
    current_color = current_color_feathered;


    for (int l = old_max_level; l < new_max_levels; l++) {

      _levels.emplace_back(_levels[l - 1].Width() / 2, _levels[l - 1].Height() / 2, true, image::RGBfColor(0.0f, 0.0f, 0.0f));
      _weights.emplace_back(_weights[l - 1].Width() / 2, _weights[l - 1].Height() / 2, true, 0.0f);
    }

    int width = current_color.Width();
    int height = current_color.Height();
    image::Image<image::RGBfColor> next_color;
    image::Image<float> next_weights;

    for (int l = old_max_level - 1; l < new_max_levels - 1; l++)
    {
      aliceVision::image::Image<image::RGBfColor> buf(width, height);
      aliceVision::image::Image<image::RGBfColor> buf2(width, height);
      aliceVision::image::Image<float> bufw(width, height);
      
      next_color = aliceVision::image::Image<image::RGBfColor>(width / 2, height / 2);
      next_weights = aliceVision::image::Image<float>(width / 2, height / 2);

      convolveGaussian5x5<image::RGBfColor>(buf, current_color);
      downscale(next_color,  buf);

      convolveGaussian5x5<float>(bufw, current_weights);
      downscale(next_weights,  bufw);

      upscale(buf, next_color);
      convolveGaussian5x5<image::RGBfColor>(buf2, buf);

      for (int i = 0; i  < buf2.Height(); i++) {
        for (int j = 0; j < buf2.Width(); j++) {
          buf2(i,j) *= 4.0f;
        }
      }

      substract(current_color, current_color, buf2);

      merge(current_color, current_weights, l, 0, 0);
      
      current_color = next_color;
      current_weights = next_weights;
      width /= 2;
      height /= 2;
    }

    merge(current_color, current_weights, _levels.size() - 1, 0, 0);

    return true;
  }

  bool apply(const aliceVision::image::Image<image::RGBfColor> & source, const aliceVision::image::Image<float> & weights, size_t offset_x, size_t offset_y) {

    int width = source.Width();
    int height = source.Height();

    image::Image<image::RGBfColor> current_color = source;
    image::Image<image::RGBfColor> next_color;
    image::Image<float> current_weights = weights;
    image::Image<float> next_weights;

    for (int l = 0; l < _levels.size() - 1; l++)
    {
      aliceVision::image::Image<image::RGBfColor> buf(width, height);
      aliceVision::image::Image<image::RGBfColor> buf2(width, height);
      aliceVision::image::Image<float> bufw(width, height);
      
      next_color = aliceVision::image::Image<image::RGBfColor>(width / 2, height / 2);
      next_weights = aliceVision::image::Image<float>(width / 2, height / 2);

      convolveGaussian5x5<image::RGBfColor>(buf, current_color);
      downscale(next_color,  buf);

      convolveGaussian5x5<float>(bufw, current_weights);
      downscale(next_weights,  bufw);

      upscale(buf, next_color);
      convolveGaussian5x5<image::RGBfColor>(buf2, buf);

      for (int i = 0; i  < buf2.Height(); i++) {
        for (int j = 0; j < buf2.Width(); j++) {
          buf2(i,j) *= 4.0f;
        }
      }

      substract(current_color, current_color, buf2);

      merge(current_color, current_weights, l, offset_x, offset_y);
      
      current_color = next_color;
      current_weights = next_weights;
      width /= 2;
      height /= 2;
      offset_x /= 2;
      offset_y /= 2;
    }

    merge(current_color, current_weights, _levels.size() - 1, offset_x, offset_y);

    return true;
  }
  
  bool merge(const aliceVision::image::Image<image::RGBfColor> & oimg, const aliceVision::image::Image<float> & oweight, size_t level, size_t offset_x, size_t offset_y) {

    image::Image<image::RGBfColor> & img = _levels[level];
    image::Image<float> & weight = _weights[level];

    for (int i = 0; i  < oimg.Height(); i++) {

      int di = i + offset_y;
      if (di >= img.Height()) continue;

      for (int j = 0; j < oimg.Width(); j++) {
          
        int dj = j + offset_x;
        if (dj >= weight.Width()) {
          dj = dj - weight.Width();
        }

        img(di, dj).r() += oimg(i, j).r() * oweight(i, j);
        img(di, dj).g() += oimg(i, j).g() * oweight(i, j);
        img(di, dj).b() += oimg(i, j).b() * oweight(i, j);
        weight(di, dj) += oweight(i, j);
      }
    }    

    return true;
  }

  bool rebuild(image::Image<image::RGBAfColor> & output) {

    for (int l = 0; l < _levels.size(); l++) {
      for (int i = 0; i < _levels[l].Height(); i++) {
        for (int j = 0; j < _levels[l].Width(); j++) {
          if (_weights[l](i, j) < 1e-6) {
            _levels[l](i, j) = image::RGBfColor(0.0);
            continue;  
          }
          
          _levels[l](i, j).r() = _levels[l](i, j).r() / _weights[l](i, j);
          _levels[l](i, j).g() = _levels[l](i, j).g() / _weights[l](i, j);
          _levels[l](i, j).b() = _levels[l](i, j).b() / _weights[l](i, j);
        }
      }
    }

    removeNegativeValues(_levels[_levels.size() - 1]);

    for (int l = _levels.size() - 2; l >= 0; l--) {

      aliceVision::image::Image<image::RGBfColor> buf(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBfColor> buf2(_levels[l].Width(), _levels[l].Height());

      upscale(buf, _levels[l + 1]);
      convolveGaussian5x5<image::RGBfColor>(buf2, buf, true);
      
      for (int i = 0; i  < buf2.Height(); i++) {
        for (int j = 0; j < buf2.Width(); j++) {
          buf2(i,j) *= 4.0f;
        }
      }

      addition(_levels[l], _levels[l], buf2);
      removeNegativeValues(_levels[l]);
    }

    // Write output to RGBA
    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {
        output(i, j).r() = _levels[0](i, j).r();
        output(i, j).g() = _levels[0](i, j).g();
        output(i, j).b() = _levels[0](i, j).b();

        if (_weights[0](i, j) < 1e-6) {
          output(i, j).a() = 0.0f;
        }
        else {
          output(i, j).a() = 1.0f;
        }
      }
    }

    return true;
  }

private:
  std::vector<aliceVision::image::Image<image::RGBfColor>> _levels;
  std::vector<aliceVision::image::Image<float>> _weights;
};

class DistanceSeams {
public:
  DistanceSeams(size_t outputWidth, size_t outputHeight) :
  _weights(outputWidth, outputHeight, true, 0.0f),
  _labels(outputWidth, outputHeight, true, 255)
  {
  }
  virtual ~DistanceSeams() = default;

  virtual bool append(const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, IndexT currentIndex, size_t offset_x, size_t offset_y)
  {
    if (inputMask.size() != inputWeights.size()) {
      return false;
    }

    for (int i = 0; i < inputMask.Height(); i++) {

      int di = i + offset_y;

      for (int j = 0; j < inputMask.Width(); j++) {

        if (!inputMask(i, j)) {
          continue;
        }
        
        int dj = j + offset_x;
        if (dj >= _weights.Width()) {
          dj = dj - _weights.Width();
        }

        if (inputWeights(i, j) > _weights(di, dj)) {
          _labels(di, dj) = currentIndex;
          _weights(di, dj) = inputWeights(i, j);
        }
      }
    }

    return true;
  }

  const image::Image<IndexT> & getLabels() {
    return _labels;
  }

private:
  image::Image<float> _weights;
  image::Image<IndexT> _labels;
};

class LaplacianCompositer : public Compositer
{
public:

  LaplacianCompositer(size_t outputWidth, size_t outputHeight, size_t bands) :
  Compositer(outputWidth, outputHeight),
  _pyramid_panorama(outputWidth, outputHeight, bands),
  _bands(bands) {

  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y)
  {
    /*Get smalles size*/
    size_t minsize = std::min(color.Height(), color.Width());
      
    /*
    minsize / 2^x = 8
    minsize / 8 = 2^x
    x = log2(minsize/8)
    */
    size_t optimal_scale = size_t(floor(std::log2(double(minsize) / 8.0)));
    if (optimal_scale < _bands) {
      ALICEVISION_LOG_ERROR("Decreasing scale !");
      return false;
    } 

    if (optimal_scale > _bands) {
      _bands = optimal_scale;
      _pyramid_panorama.augment(_bands);
    }

    size_t new_offset_x, new_offset_y;
    aliceVision::image::Image<image::RGBfColor> color_pot;
    aliceVision::image::Image<unsigned char> mask_pot;
    aliceVision::image::Image<float> weights_pot;
    makeImagePyramidCompatible(color_pot, new_offset_x, new_offset_y, color, offset_x, offset_y, _bands);
    makeImagePyramidCompatible(mask_pot, new_offset_x, new_offset_y, inputMask, offset_x, offset_y, _bands);
    makeImagePyramidCompatible(weights_pot, new_offset_x, new_offset_y, inputWeights, offset_x, offset_y, _bands);


    aliceVision::image::Image<image::RGBfColor> feathered;
    feathering(feathered, color_pot, mask_pot);

    /*To log space for hdr*/
    for (int i = 0; i < feathered.Height(); i++) {
      for (int j = 0; j < feathered.Width(); j++) {

        feathered(i, j).r() = std::log(std::max(1e-8f, feathered(i, j).r()));
        feathered(i, j).g() = std::log(std::max(1e-8f, feathered(i, j).g()));
        feathered(i, j).b() = std::log(std::max(1e-8f, feathered(i, j).b()));
      }
    }
  
    _pyramid_panorama.apply(feathered, weights_pot, new_offset_x, new_offset_y);
 
    return true;
  }

  virtual bool terminate() {

    _pyramid_panorama.rebuild(_panorama);

    /*Go back to normal space from log space*/
    for (int i = 0; i  < _panorama.Height(); i++) {
      for (int j = 0; j < _panorama.Width(); j++) {
        _panorama(i, j).r() = std::exp(_panorama(i, j).r());
        _panorama(i, j).g() = std::exp(_panorama(i, j).g());
        _panorama(i, j).b() = std::exp(_panorama(i, j).b());
      }
    }

    return true;
  }

protected:
  LaplacianPyramid _pyramid_panorama;
  size_t _bands;
};

int aliceVision_main(int argc, char **argv)
{
  std::string sfmDataFilepath;
  std::string warpingFolder;
  std::string outputPanorama;

  std::string compositerType = "multiband";
  std::string overlayType = "none";

  system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

  // Program description
  po::options_description allParams (
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
    "AliceVision PanoramaCompositing"
  );

  // Description of mandatory parameters
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
    ("warpingFolder,w", po::value<std::string>(&warpingFolder)->required(), "Folder with warped images.")
    ("output,o", po::value<std::string>(&outputPanorama)->required(), "Path of the output panorama.");
  allParams.add(requiredParams);

  // Description of optional parameters
  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("compositerType,c", po::value<std::string>(&compositerType)->required(), "Compositer Type [replace, alpha, multiband].")
    ("overlayType,c", po::value<std::string>(&overlayType)->required(), "Overlay Type [none, borders, seams].");
  allParams.add(optionalParams);

  // Setup log level given command line
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
  allParams.add(logParams);


  // Effectively parse command line given parse options
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // Set verbose level given command line
  system::Logger::get()->setLogLevel(verboseLevel);

  // load input scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::EXTRINSICS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
    return EXIT_FAILURE;
  }

  std::pair<int, int> panoramaSize;
  {
      const IndexT viewId = *sfmData.getValidViews().begin();
      const std::string viewFilepath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
      ALICEVISION_LOG_TRACE("Read panorama size from file: " << viewFilepath);

      oiio::ParamValueList metadata = image::readImageMetadata(viewFilepath);
      panoramaSize.first = metadata.find("AliceVision:panoramaWidth")->get_int();
      panoramaSize.second = metadata.find("AliceVision:panoramaHeight")->get_int();

      if(panoramaSize.first == 0 || panoramaSize.second == 0)
      {
          ALICEVISION_LOG_ERROR("The output panorama size is empty.");
          return EXIT_FAILURE;
      }
      ALICEVISION_LOG_INFO("Output panorama size set to " << panoramaSize.first << "x" << panoramaSize.second);
  }

  std::unique_ptr<Compositer> compositer;
  bool isMultiBand = false;
  if (compositerType == "multiband")
  {
    compositer = std::unique_ptr<Compositer>(new LaplacianCompositer(panoramaSize.first, panoramaSize.second, 1));
    isMultiBand = true;
  }
  else if (compositerType == "alpha")
  {
    compositer = std::unique_ptr<Compositer>(new AlphaCompositer(panoramaSize.first, panoramaSize.second));
  }
  else
  {
    compositer = std::unique_ptr<Compositer>(new Compositer(panoramaSize.first, panoramaSize.second));
  }



  // Compute seams
  std::vector<std::shared_ptr<sfmData::View>> viewsToDraw;

  std::unique_ptr<DistanceSeams> distanceseams(new DistanceSeams(panoramaSize.first, panoramaSize.second));
  if (isMultiBand)
  {
    std::map<size_t, std::vector<std::shared_ptr<sfmData::View>>> indexed_by_scale;
    for (const auto& viewIt : sfmData.getViews())
    {
      if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
      {
          // skip unreconstructed views
          continue;
      }
      
      // Load mask
      const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + "_mask.exr")).string();
      ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
      image::Image<unsigned char> mask;
      image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

      oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
      const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
      const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

      // Load Weights
      const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + "_weight.exr")).string();
      ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
      image::Image<float> weights;
      image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);

      distanceseams->append(mask, weights, viewIt.first, offsetX, offsetY);

      /*Get smalles size*/
      size_t minsize = std::min(mask.Height(), mask.Width());
      
      /*
      minsize / 2^x = 8
      minsize / 8 = 2^x
      x = log2(minsize/8)
      */
      size_t optimal_scale = size_t(floor(std::log2(double(minsize) / 8.0)));
      indexed_by_scale[optimal_scale].push_back(viewIt.second);
    }

    for (auto item : indexed_by_scale) {
      for (auto view : item.second) {
        viewsToDraw.push_back(view);
      }
    }
  }
  else {
    for (auto& viewIt : sfmData.getViews())
    {
      if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
      {
          // skip unreconstructed views
          continue;
      }
      
      viewsToDraw.push_back(viewIt.second);
    }
  }

  image::Image<IndexT> labels = distanceseams->getLabels();
  distanceseams.reset();
  distanceseams = nullptr;

  oiio::ParamValueList outputMetadata;

  // Do compositing
  for (const auto & view : viewsToDraw)
  {
    IndexT viewId = view->getViewId();

    if(!sfmData.isPoseAndIntrinsicDefined(view.get()))
    {
        // skip unreconstructed views
        continue;
    }

    // Load image and convert it to linear colorspace
    const std::string imagePath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
    ALICEVISION_LOG_INFO("Load image with path " << imagePath);
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

    oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
    if(outputMetadata.empty())
    {
        // the first one will define the output metadata (random selection)
        outputMetadata = metadata;
    }
    const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
    const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

    // Load mask
    const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_mask.exr")).string();
    ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
    image::Image<unsigned char> mask;
    image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

    // Load Weights
    const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_weight.exr")).string();
    ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
    image::Image<float> weights;
    image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);

    // Build weight map
    if (isMultiBand)
    {
      image::Image<float> seams(weights.Width(), weights.Height());
      getMaskFromLabels(seams, labels, viewId, offsetX, offsetY);

      // Composite image into panorama
      compositer->append(source, mask, seams, offsetX, offsetY);
    }
    else
    {
      compositer->append(source, mask, weights, offsetX, offsetY);
    }
  }

  // Build image
  compositer->terminate();

  if (overlayType == "borders")
  {
    for (const auto& viewIt : sfmData.getViews())
    {
      if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
      {
          // skip unreconstructed views
          continue;
      }

      // Load mask
      const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewIt.first) + "_mask.exr")).string();
      ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
      image::Image<unsigned char> mask;
      image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

      oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
      const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
      const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

      drawBorders(compositer->getPanorama(), mask, offsetX, offsetY);
    }
  }
  else if (overlayType == "seams")
  {
    drawSeams(compositer->getPanorama(), labels);
  }

  // Remove Warping-specific metadata
  outputMetadata.remove("AliceVision:offsetX");
  outputMetadata.remove("AliceVision:offsetY");
  outputMetadata.remove("AliceVision:panoramaWidth");
  outputMetadata.remove("AliceVision:panoramaHeight");

  // Store output
  ALICEVISION_LOG_INFO("Write output panorama to file " << outputPanorama);
  const aliceVision::image::Image<image::RGBAfColor> & panorama = compositer->getPanorama();
  image::writeImage(outputPanorama, panorama, image::EImageColorSpace::AUTO, outputMetadata);

  return EXIT_SUCCESS;
}
