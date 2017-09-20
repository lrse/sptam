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

#pragma once

#include "../LCDetector.hpp"

// DLoopDetector and DBoW2
#include "DBoW2.h"
#include "FBRISK.h"
#include "DLoopDetector.h"

// F class of descriptor functions and definitions
template<class F>
class DLDLoopDetector : public LCDetector
{
  typedef DBoW2::TemplatedVocabulary<typename F::TDescriptor, F> Vocabulary;
  typedef DLoopDetector::TemplatedLoopDetector<typename F::TDescriptor, F> LoopDetector;
  
  public:
    struct Parameters : LoopDetector::Parameters
    {
      Parameters(int height = 0, int width = 0, float frequency = 1, bool nss = true,
        float _alpha = 0.3, int _k = 0,
        DLoopDetector::GeometricalCheck geom = DLoopDetector::GEOM_NONE, int dilevels = 0)
        : LoopDetector::Parameters(height, width, frequency, nss,
                                        _alpha, _k,
                                        geom, dilevels)
      {}

      static void loadFromYML(const std::string& file_path);
    };

    DLDLoopDetector(const std::string& voc_file_path, const Parameters& params);

    DetectionMatch detectloop(const sptam::Map::SharedKeyFrame& stereo_frame);
  
  private:
    // Descriptors Vocabulary
    Vocabulary voc;
    // Loop Detector
    LoopDetector detector;
};
