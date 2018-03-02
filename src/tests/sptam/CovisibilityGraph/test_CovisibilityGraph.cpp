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
#include "gtest/gtest.h"
#include "utils/CovisibilityGraph.hpp"
#include "utils/CovisibilityGraph.cpp"

typedef struct {
  size_t id;
} KeyFrame;

typedef struct {
  size_t id;
} MapPoint;

typedef struct {
  size_t id;
} Measurement;

typedef CovisibilityGraph<KeyFrame, MapPoint, Measurement> SimpleGraph;

TEST (BasicTest, Instantiation)
{
  SimpleGraph graph;
}

TEST (BasicTest, Empty)
{
  SimpleGraph graph;

  const SimpleGraph::SharedKeyFrameList& kfs = graph.getKeyframes();
  EXPECT_EQ (0, kfs.size());

  const SimpleGraph::SharedMapPointList& pts = graph.getMapPoints();
  EXPECT_EQ (0, pts.size());
}

TEST (BasicTest, addKeyFrame)
{
  SimpleGraph graph;

  KeyFrame kf({1u});
  graph.addKeyFrame( kf );
}

TEST (BasicTest, addMapPoint)
{
  SimpleGraph graph;

  KeyFrame kf({1u});
  graph.addKeyFrame( kf );
}
