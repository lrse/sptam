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

#include "FeatureExtractorThread.hpp"

#ifdef SHOW_PROFILING
  #include "utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

#include <memory>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#ifdef ENABLE_PARALLEL_CODE
#include <tbb/parallel_for.h>
#include <tbb/blocked_range2d.h>
#endif


#define GRID_EXTRACTOR 0

FeatureExtractorThread::FeatureExtractorThread(const cv::Mat& image,
  cv::Ptr<cv::FeatureDetector> featureDetector,
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor,
  size_t nFeatures
)
  : image_( image )
  #if GRID_EXTRACTOR
  , nFeatures_(300)
  , grid_size(cv::Size(10, 10))
  , featureDetector_(cv::FastFeatureDetector::create(20))
  , featureDetector2_(cv::FastFeatureDetector::create(7))
  #else
  , nFeatures_(nFeatures)
  , featureDetector_( featureDetector )
  #endif
  , descriptorExtractor_( descriptorExtractor )

{
#if GRID_EXTRACTOR
  featureExtractorThread_ = std::thread(&FeatureExtractorThread::ExtractGrid, this);
#else
  featureExtractorThread_ = std::thread(&FeatureExtractorThread::Extract, this);
#endif
}

#define PROFILE_INTERNAL 0 /* leave this disabled so that logging does not slow down parallel feature extraction */

void FeatureExtractorThread::Extract()
{
  #if defined(SHOW_PROFILING) && PROFILE_INTERNAL
    sptam::Timer t_detect, t_extract;
    t_detect.start();
  #endif

  featureDetector_->detect(image_, keyPoints_);

  assert( not keyPoints_.empty() );

  #if defined(SHOW_PROFILING) && PROFILE_INTERNAL
    t_detect.stop();
    t_extract.start();
  #endif

  descriptorExtractor_->compute(image_, keyPoints_, descriptors_);

  #if defined(SHOW_PROFILING) && PROFILE_INTERNAL
    t_extract.stop();
    std::cout << "extract: " << t_detect << " compute: " << t_extract << std::endl;
    WriteToLog(" tk FeatureDetection: ", t_detect.elapsed());
    WriteToLog(" tk DescriptorExtraction: ", t_extract.elapsed());
  #endif
}

#define PARALLEL_GRID_EXTRACT 0

void FeatureExtractorThread::ExtractGrid(void)
{
#if PROFILE_INTERNAL
  sptam::Timer t_extract, t_compute;
  t_extract.start();
#endif

  /* TODO: check math */
  /* obtain cell size according to grid count and image size */
  cv::Size cell_size(ceilf(image_.size().width / (float)grid_size.width), ceilf(image_.size().height / (float)grid_size.height));

  std::list<cv::KeyPoint> keypoints_list;

#if !(PARALLEL_GRID_EXTRACT)
  /* go through each cell in the grid */
  for (int xi = 0; xi < grid_size.width; xi++)
  {
    for (int yj = 0; yj < grid_size.height; yj++)
    {
      /* TODO: check math */
      /* obtain rectangle corresponding to current cell */
      cv::Rect cell_rect(xi * cell_size.width, yj * cell_size.height, cell_size.width, cell_size.height);
      if (cell_rect.tl().x + cell_rect.width > image_.size().width) cell_rect.width = (image_.size().width - cell_rect.tl().x);
      if (cell_rect.tl().y + cell_rect.height > image_.size().height) cell_rect.height = (image_.size().height - cell_rect.tl().y);
      cv::Mat cell_image = image_(cell_rect);

      /* detect features in sub-image */
      std::vector<cv::KeyPoint> cell_keypoints;
      featureDetector_->detect(cell_image, cell_keypoints);

      if (cell_keypoints.empty())
      {
        cell_keypoints.clear();
        featureDetector2_->detect(cell_image, cell_keypoints);
      }

      //std::cout << "cell " << xi << "," << yj << " " << cell_rect << " " << image_.size() << " " << cell_keypoints.size() << std::endl;

      /* compensate offset position and store to general list */
      for (cv::KeyPoint& kp : cell_keypoints)
      {
        kp.pt = cv::Point2f(kp.pt.x + cell_rect.tl().x, kp.pt.y + cell_rect.tl().y);
        keypoints_list.push_back(kp);
      }
    }
  }
#else
  #ifdef ENABLE_PARALLEL_CODE
  std::vector<std::vector<std::vector<cv::KeyPoint>>> keypoints_grid(grid_size.height, std::vector<std::vector<cv::KeyPoint>>(grid_size.width));

  tbb::parallel_for(tbb::blocked_range2d<size_t, size_t>(0,  grid_size.height, 0, grid_size.width),
    [&](const tbb::blocked_range2d<size_t, size_t>& range) {
      for (size_t xi = range.rows().begin(); xi != range.rows().end(); ++xi)
      {
        for (size_t yj = range.cols().begin(); yj != range.cols().end(); ++yj)
        {
          /* TODO: check math */
          /* obtain rectangle corresponding to current cell */
          cv::Rect cell_rect(xi * cell_size.width, yj * cell_size.height, cell_size.width, cell_size.height);
          if (cell_rect.tl().x + cell_rect.width > image_.size().width) cell_rect.width = (image_.size().width - cell_rect.tl().x);
          if (cell_rect.tl().y + cell_rect.height > image_.size().height) cell_rect.height = (image_.size().height - cell_rect.tl().y);
          cv::Mat cell_image = image_(cell_rect);

          /* detect features in sub-image */
          std::vector<cv::KeyPoint>& cell_keypoints = keypoints_grid[xi][yj];
          featureDetector_->detect(cell_image, cell_keypoints);

          if (cell_keypoints.empty())
          {
            cell_keypoints.clear();
            featureDetector2_->detect(cell_image, cell_keypoints);
          }

          //std::cout << "cell " << xi << "," << yj << " " << cell_rect << " " << image_.size() << " " << cell_keypoints.size() << std::endl;

          /* compensate offset position and store to general list */
          for (cv::KeyPoint& kp : cell_keypoints)
          {
            kp.pt = cv::Point2f(kp.pt.x + cell_rect.tl().x, kp.pt.y + cell_rect.tl().y);
          }
        }
      }
    }, tbb::simple_partitioner() );

  for (size_t i = 0; i < (size_t)grid_size.height; i++)
    for (size_t j = 0; j < (size_t)grid_size.width; j++)
      std::copy(keypoints_grid[i][j].begin(), keypoints_grid[i][j].end(), std::back_inserter(keypoints_list));
  #else
    #error "habilitame parallel code vieja"
  #endif
#endif

#if PROFILE_INTERNAL
  t_extract.stop();
#endif

  quadtreeFilter(keypoints_list, keyPoints_);

#if PROFILE_INTERNAL
  std::cout << "filtered " << keypoints_list.size() << " keypoints to: " << keyPoints_.size() << std::endl;
  t_compute.start();
#endif

  descriptorExtractor_->compute(image_, keyPoints_, descriptors_);

#if PROFILE_INTERNAL
  t_compute.stop();
  std::cout << "extract: " << t_extract << " compute: " << t_compute << std::endl;
#endif
}

class QuadTreeCell
{
  public:
    QuadTreeCell(const cv::Rect& rect) : rect_(rect) { }

    QuadTreeCell(const std::list<cv::KeyPoint>& keypoints, const cv::Rect& rect) :
       keypoints_(keypoints), rect_(rect) { }

    static void subdivide(const std::shared_ptr<QuadTreeCell>& root, std::list<std::shared_ptr<QuadTreeCell>>& final_cells, size_t max_leaves)
    {
#if PROFILE_INTERNAL
      sptam::Timer t_subdivide, t_sort;
      t_subdivide.start();
#endif

      std::list<std::shared_ptr<QuadTreeCell>> to_visit;
      to_visit.push_front(root);

      bool iterate = true;
      while(iterate && !to_visit.empty())
      {
#if PROFILE_INTERNAL
        t_sort.start();
#endif
        /* sort cells to visit according to keypoint size (prioritize, densest cells) */
        std::vector<std::shared_ptr<QuadTreeCell>> to_visit_vector;
        to_visit_vector.reserve(to_visit.size());
        std::copy(to_visit.begin(), to_visit.end(), std::back_inserter(to_visit_vector));
        std::sort(to_visit_vector.begin(), to_visit_vector.end(), [](const auto& a, const auto& b) { return b->keypoints_.size() < a->keypoints_.size(); });
        to_visit.clear();
        std::copy(to_visit_vector.begin(), to_visit_vector.end(), std::back_inserter(to_visit));
#if PROFILE_INTERNAL
        t_sort.stop();
#endif

        /* iterate cells at current subdivision level */
        auto it = to_visit.begin();
        while (it != to_visit.end())
        {
          const std::shared_ptr<QuadTreeCell>& c = *it;

          /* this cell has more than one keypoint and is still large enough, thus create immediate children and distribute its points of this cell to them */
          if (c->keypoints_.size() > 1 && c->rect_.area() > 250)
          {
            c->distribute();

            /* add to queue its children */
            for (int i = 0; i < 4; i++)
            {
              if (!c->children_[i]->keypoints_.empty())
                to_visit.push_front(c->children_[i]);
            }

          }
          /* this cell has only one point, this is a leaf for sure */
          else
          {
            final_cells.push_back(c);
          }

          /* remove element from current level */
          it = to_visit.erase(it);

          //std::cout << "final: " << final_cells.size() << " " << to_visit.size() << std::endl;

          if ((final_cells.size() + to_visit.size()) >= max_leaves)
          {
            iterate = false;
            break;
          }
        }
      }

      /* in case we exceeded the number of cells, make unvisited cells leafs */
      std::copy(to_visit.begin(), to_visit.end(), std::back_inserter(final_cells));

#if PROFILE_INTERNAL
      t_subdivide.stop();
      std::cout << "subdivide: " << t_subdivide << std::endl;
      std::cout << "sort: " << t_sort << std::endl;
#endif
    }

    void distribute(void)
    {
      /* build children, define corresponding rectangles */
      cv::Size2f half_size(rect_.width / 2, rect_.height / 2);
      cv::Rect2f tl = cv::Rect(rect_.tl(), half_size);
      cv::Rect2f tr = cv::Rect(cv::Point2f(tl.tl().x + half_size.width, tl.tl().y), half_size);
      cv::Rect2f bl = cv::Rect(cv::Point2f(tl.tl().x, tl.tl().y + half_size.height), half_size);
      cv::Rect2f br = cv::Rect(cv::Point2f(tl.tl().x + half_size.width, tl.tl().y + half_size.height), half_size);

      children_[TOP_LEFT]  = std::make_shared<QuadTreeCell>(tl);
      children_[TOP_RIGHT] = std::make_shared<QuadTreeCell>(tr);
      children_[BOT_LEFT]  = std::make_shared<QuadTreeCell>(bl);
      children_[BOT_RIGHT] = std::make_shared<QuadTreeCell>(br);

      /* distribute current cell points to children */
      for (const cv::KeyPoint& kp : keypoints_)
      {
        for (int i = 0; i < 4; i++)
        {
          if (children_[i]->rect_.contains(kp.pt)) children_[i]->keypoints_.push_back(kp);
        }
      }
    }

    std::list<cv::KeyPoint> keypoints_;
    cv::Rect2f rect_;
    std::shared_ptr<QuadTreeCell> children_[4];

    enum ChildPosition { TOP_LEFT = 0, TOP_RIGHT, BOT_LEFT, BOT_RIGHT };
};

#define DEBUG_QUADTREE 0

void FeatureExtractorThread::quadtreeFilter(const std::list<cv::KeyPoint> &keypoints_in, std::vector<cv::KeyPoint>& keypoints_out)
{
  assert(nFeatures_ > 0);

  /* create quadtree */
  std::list<std::shared_ptr<QuadTreeCell>> cells;
  QuadTreeCell::subdivide(std::make_shared<QuadTreeCell>(keypoints_in, cv::Rect(cv::Point(0,0), image_.size())), cells, nFeatures_);

  //std::cout << "cells: " << cells.size() << std::endl;

#if DEBUG_QUADTREE
  cv::Mat debug;
  cv::cvtColor(image_, debug, CV_GRAY2BGR);

  for (const auto& kp : keypoints_in)
  {
    cv::circle(debug, kp.pt, 1, cv::Scalar(0,0,255));
  }
#endif

  /* iterate cells and get the maximum response feature of each cell */
  keypoints_out.reserve(keypoints_in.size());
  for (const std::shared_ptr<QuadTreeCell>& c : cells)
  {
    //std::cout << "kpts per cell: " << c->keypoints_.size() << std::endl;
    keypoints_out.push_back(*std::max_element(c->keypoints_.begin(), c->keypoints_.end(), [](const cv::KeyPoint& k1, const cv::KeyPoint& k2) { return k1.response < k2.response; }));

#if DEBUG_QUADTREE
    cv::rectangle(debug, c->rect_, cv::Scalar(255,0,0));
    cv::circle(debug, keypoints_out.back().pt, 1, cv::Scalar(0,255,0));
#endif
  }

#if DEBUG_QUADTREE
  cv::imwrite("debug.png", debug);
#endif
}
