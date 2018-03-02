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

#include <set>
#include "Map.hpp"

// forward declaration
class BundleDriver;

/**
 * @brief When performing Local Bundle Adjustment, a strategy for defining the
 * set of keyframes to be adjusted is necessary. Since this choice may vary for
 * different problems, it is desirable to be able to swiftly choose or switch
 * between different strategies. This class defines the common interface to
 * which every locality selection strategy must comply.
 */
class ILocalWindow
{
  public:

    // TODO this should actually be the type that is expected by BA, shouldn't it?
    typedef std::set< sptam::Map::SharedKeyFrame > KeyFrameWindow;

    // TODO this is a hack because the current implementation of
    // CovisibilityWindow only looks for covisible keyframes to the LAST
    // inserted one. Ideally the strategy should search for the covisible
    // keyframes to all the new keyframes to be adjusted. Once this is the case,
    // we can eliminate the last parameter. We require that
    // last_keyframe \in new_keyframes to make the future transition easier.
    virtual void populateBA(const std::list<sptam::Map::SharedKeyFrame>& new_keyframes, BundleDriver& bundle_adjuster, sptam::Map::SharedKeyFrameSet& adjustable_keyframes, sptam::Map::SharedKeyFrameSet& fixed_keyframes, std::function<bool(const sptam::Map::SharedKeyFrame&)> isSafe) = 0;
};
