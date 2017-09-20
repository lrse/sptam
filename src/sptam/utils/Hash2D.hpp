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

#include "macros.hpp"

#include <list>
#include <vector>

template <typename T>
class Hash2D
{
  public:

    Hash2D(size_t size_x, size_t size_y, size_t cell_size_x, size_t cell_size_y);

    inline size_t rows() const
    { return n_rows_; }

    inline size_t cols() const
    { return n_cols_; }

    /**
     * @brief insert element in bucket (x, y).
     */
    void insert(size_t x, size_t y, const T& elem);

    std::list<T> getNeighborhood(size_t x, size_t y, size_t radius) const;

  private:

    size_t n_rows_, n_cols_;

    size_t cell_size_x_, cell_size_y_;

    // The 2d hash matrix will be stored in 1D using row first order.
    // Each bucket has a list of the elements that fall into it.
    std::vector< std::list<T> > hashed_elements_;

    /*
     * Get the bucket index for real coordinates (x, y).
     **/
    inline size_t getHash(size_t x, size_t y) const
    {
      size_t cell_x, cell_y;
      getHash2D(x, y, cell_x, cell_y);
      return flatten(cell_x, cell_y);
    }

    inline void getHash2D(size_t x, size_t y, size_t& cell_x, size_t& cell_y) const
    {
      cell_x = floor((float) x / (float) cell_size_x_);
      cell_y = floor((float) y / (float) cell_size_y_);
    }

    inline size_t flatten(size_t cell_x, size_t cell_y) const
    {
      return cell_y * n_rows_ + cell_x;
    }
};

template <typename T>
Hash2D<T>::Hash2D(size_t size_x, size_t size_y, size_t cell_size_x, size_t cell_size_y) 
  : n_rows_( ceil( size_y / (double) cell_size_y ) )
  , n_cols_( ceil( size_x / (double) cell_size_x ) )
  , cell_size_x_( cell_size_x ), cell_size_y_( cell_size_y )
  , hashed_elements_( n_rows_ * n_cols_ )
{}

template <typename T>
void Hash2D<T>::insert(size_t x, size_t y, const T& elem)
{
  size_t idx = getHash(x, y);
  assert( idx < hashed_elements_.size() );
  hashed_elements_[ idx ].push_back( elem );
}

template <typename T>
std::list<T> Hash2D<T>::getNeighborhood(size_t x, size_t y, size_t radius) const
{
  size_t center_x, center_y;
  getHash2D(x, y, center_x, center_y);

  size_t fromY = radius < center_y ? center_y - radius : 0;
  size_t toY = std::min( center_y + radius + 1, rows() );

  size_t fromX = radius < center_x ? center_x - radius : 0;
  size_t toX = std::min( center_x + radius + 1, cols() );

  std::list<T> ret;

  forsn (cell_y, fromY, toY)
    forsn (cell_x, fromX, toX)
      for( const T& elem : hashed_elements_[ flatten(cell_x, cell_y) ] )
        ret.push_back( elem );

  return ret;
}
