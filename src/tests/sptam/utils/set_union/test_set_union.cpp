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
#include <list>
#include "gtest/gtest.h"
#include "utils/set_union.hpp"

TEST (SetUnion, BothEmpty)
{
  std::set<int> s1, s2;

  std::set<int> u = setUnion(s1, s2);

  ASSERT_EQ(0, u.size());
}

TEST (SetUnion, FirstEmpty)
{
  std::set<int> s1, s2;
  s2.insert( 2 );

  std::set<int> u = setUnion(s1, s2);

  ASSERT_EQ(1, u.size());
  ASSERT_EQ(2, *(u.begin()));
}

TEST (SetUnion, SecondEmpty)
{
  std::set<int> s1, s2;
  s1.insert( 2 );

  std::set<int> u = setUnion(s1, s2);

  ASSERT_EQ(1, u.size());
  ASSERT_EQ(2, *(u.begin()));
}

TEST (SetUnion, EqualSingletons)
{
  std::set<int> s1, s2;
  s1.insert( 2 );
  s2.insert( 2 );

  std::set<int> u = setUnion(s1, s2);

  ASSERT_EQ(1, u.size());
  ASSERT_EQ(2, *(u.begin()));
}

TEST (SetUnion, Disjunct)
{
  std::set<int> s1, s2;
  s1.insert( 1 ); s1.insert( 3 );
  s2.insert( 2 ); s2.insert( 4 );

  std::set<int> u = setUnion(s1, s2);

  ASSERT_EQ(4, u.size());

  std::list<int> expected;
  for(int i=1; i<1+4; i++)
    expected.push_back( i );

  ASSERT_TRUE( std::equal(u.begin(), u.end(), expected.begin()) );
}

TEST (SetUnion, Overlapping)
{
  std::set<int> s1, s2;
  s1.insert( 1 ); s1.insert( 2 );
  s2.insert( 2 ); s2.insert( 3 );

  std::set<int> u = setUnion(s1, s2);

  ASSERT_EQ(3, u.size());

  std::list<int> expected;
  for(int i=1; i<1+3; i++)
    expected.push_back( i );

  ASSERT_TRUE( std::equal(u.begin(), u.end(), expected.begin()) );
}
