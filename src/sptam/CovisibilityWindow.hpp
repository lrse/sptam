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

#include "ILocalWindow.hpp"

// forward declaration
class BundleDriver;

class CovisibilityWindow : public ILocalWindow
{
  public:

    CovisibilityWindow(size_t window_size) : window_size_( window_size ) {}

    void populateBA(const std::list<sptam::Map::SharedKeyFrame>& new_keyframes, BundleDriver& bundle_adjuster, sptam::Map::SharedKeyFrameSet& adjustable_keyframes, sptam::Map::SharedKeyFrameSet& fixed_keyframes, std::function<bool(const sptam::Map::SharedKeyFrame&)> isSafe) override;

  private:

    size_t window_size_;
};
