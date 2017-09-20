/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2013-2017 Taihú Pire
 * Copyright (C) 2014-2017 Thomas Fischer
 * Copyright (C) 2016-2017 Gastón Castro
 * Copyright (C) 2017 Matias Nitsche
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire
 *           Thomas Fischer
 *           Gastón Castro
 *           Matías Nitsche
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#include "DLDLoopDetector.hpp"

#ifdef SHOW_PROFILING
  #include "../../utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

using namespace std;

template<class F>
DLDLoopDetector<F>::DLDLoopDetector(const string& voc_file_path, const Parameters& params)
: voc(voc_file_path), detector(voc, params)
{}

/* Generic DLD implementations with descriptors as cv::Mat as container with rows as descriptors */
template <class F>
DetectionMatch DLDLoopDetector<F>::detectloop(const sptam::Map::SharedKeyFrame& stereo_frame)
{
  // Process image
  vector<cv::KeyPoint> keys = stereo_frame->GetFrameLeft().GetFeatures().GetKeypoints();

  vector<typename F::TDescriptor> descriptors;
  const cv::Mat image_descriptors = stereo_frame->GetFrameLeft().GetFeatures().GetDescriptors();

  /* cv::Mat with descriptors as rows to vector<cv::Mat> (is not copying data, just cv::Mat pointers) */
  for (int i = 0; i < image_descriptors.rows; i++)
    descriptors.push_back(image_descriptors.row(i));

  // add image to the collection and check if there is some loop
  DLoopDetector::DetectionResult result;
  detector.detectLoop(keys, descriptors, result);

  #ifdef SHOW_PROFILING
    WriteToLog(" lc Detection Result: ", result.status);
  #endif

  DetectionMatch dm = {result.detection(), result.query, result.match, -1};

  return dm;
}

/* DLD expects brief descriptors as boost::dynamic_bitset<>, we need
 * to translate openCV descriptors. */
template < >
DetectionMatch DLDLoopDetector<DBoW2::FBrief>::detectloop(const sptam::Map::SharedKeyFrame& stereo_frame)
{
  // Process image
  vector<cv::KeyPoint> keys;
  vector<FBrief::TDescriptor> descriptors;

  /* cv::Mat descriptor with 8bits uchar's as data (there might be 16,32 or 64 uchar for each row)
   * TODO: Look for a "good" way of translating dynamic_bitset<uchar> to dynamic_bitset<ulong>
   *       as they use diferent block storages, we have to "copy" each bit all over again */
  const cv::Mat image_descriptors = stereo_frame->GetFrameLeft().GetFeatures().GetDescriptors();
  const vector<cv::KeyPoint>& image_kpts = stereo_frame->GetFrameLeft().GetFeatures().GetKeypoints();

  for(int i = 0; i < image_descriptors.rows; i++){

    const uchar* char_desc = image_descriptors.row(i).ptr(0);

    boost::dynamic_bitset<uchar> char_bset_desc; // openCV brief descriptors are in uchars!
    FBrief::TDescriptor bset_desc; // by default dynamic_biset uses ulong as block storage

    for(int i = 0; i < image_descriptors.cols; i++)
      char_bset_desc.append(char_desc[i]); // append each uchar

    bset_desc.resize(char_bset_desc.size());

    for(unsigned int i = 0; i < char_bset_desc.size(); i++)
      bset_desc[i] = char_bset_desc[i]; // copying each bit

    keys.push_back(image_kpts[i]);
    descriptors.push_back(bset_desc);
  }

  // add image to the collection and check if there is some loop
  DLoopDetector::DetectionResult result;
  detector.detectLoop(keys, descriptors, result);

  #ifdef SHOW_PROFILING
    WriteToLog(" lc Detection Result: ", result.status);
  #endif

  DetectionMatch dm = {result.detection(), result.query, result.match, -1};

  return dm;
}

template class DLDLoopDetector<DBoW2::FBrief>;
template class DLDLoopDetector<DBoW2::FBRISK>;
