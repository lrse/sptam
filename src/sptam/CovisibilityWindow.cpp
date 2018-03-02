#include "CovisibilityWindow.hpp"
#include "BundleDriver.hpp"
#include "utils/log/ScopedProfiler.hpp"

std::set<sptam::Map::SharedKeyFrame> selectAdjustableKeyframes(const std::list<sptam::Map::SharedKeyFrame>& new_keyframes, size_t n_adjust, std::function<bool(const sptam::Map::SharedKeyFrame&)> isSafe)
{
  std::set< sptam::Map::SharedKeyFrame > keyframe_window;

  //////////////////////////
  // Insert new keyframes //
  //////////////////////////

  for ( auto keyframe : new_keyframes )
    keyframe_window.insert( keyframe );

  ////////////////////////////////////////////
  // Insert keyframes covisible to the last //
  ////////////////////////////////////////////

  const sptam::Map::SharedKeyFrame& last_keyframe = new_keyframes.back();

  // get covisible keyframes for the query keyframe
  std::vector< std::pair<sptam::Map::SharedKeyFrame, size_t> > covisible_keyframes = last_keyframe->covisibilityKeyFramesVector();

  // remove keyframes that are unsafe or already loaded
  const auto covisible_filtered_end = std::remove_if(covisible_keyframes.begin(), covisible_keyframes.end(), [&] (const auto& kv) { return kv.first->isFixed() or keyframe_window.count( kv.first ) or not isSafe( kv.first ); });

  // calculate the size of potential covisible keyframes
  const size_t n_expected = n_adjust - keyframe_window.size();
  const size_t n_avaible = std::distance(covisible_keyframes.begin(), covisible_filtered_end);
  const size_t n_covisibles = std::min(n_expected, n_avaible);
  const auto covisible_sorted_end = covisible_keyframes.begin() + n_covisibles;

  // extract the 'n_covisibles' keyframes with highest covisibility index
  std::partial_sort(covisible_keyframes.begin(), covisible_sorted_end, covisible_filtered_end, [](const auto& a, const auto& b) { return a.second > b.second; });

  // insert selected keyframes into the window
  for(auto it=covisible_keyframes.begin(); it!=covisible_sorted_end; it++)
    keyframe_window.insert( it->first );

  return keyframe_window;
}

sptam::Map::SharedKeyFrameSet selectSafeCovisibleKFeyframes(sptam::Map::SharedKeyFrameSet& query_keyframes, std::function<bool(const sptam::Map::SharedKeyFrame&)> isSafe)
{
  sptam::Map::SharedKeyFrameSet covisible_keyframes;

  for ( const sptam::Map::SharedKeyFrame& keyFrame : query_keyframes )
    for (auto& kv : keyFrame->covisibilityKeyFrames())
    {
      const sptam::Map::SharedKeyFrame& covisible_keyframe = kv.first;

      if( !query_keyframes.count( covisible_keyframe ) )
        if ( isSafe( covisible_keyframe ) )
          covisible_keyframes.insert( covisible_keyframe );
    }

  return covisible_keyframes;
}

void CovisibilityWindow::populateBA(const std::list<sptam::Map::SharedKeyFrame>& new_keyframes, BundleDriver& bundle_adjuster, sptam::Map::SharedKeyFrameSet& adjustable_keyframes, sptam::Map::SharedKeyFrameSet& fixed_keyframes, std::function<bool(const sptam::Map::SharedKeyFrame&)> isSafe)
{
  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_select: ");
  #endif

  adjustable_keyframes = selectAdjustableKeyframes(new_keyframes, window_size_, isSafe);

  /* Any other keyframe inside the safe window that measure above points shared, will be used as fixed keyframe
   * Gaston: Loop Closure safe window its defined in the multi-threaded version through sincronization messages  */
  fixed_keyframes = selectSafeCovisibleKFeyframes(adjustable_keyframes, isSafe);

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_load: ");
  #endif

  bundle_adjuster.SetData(
    ConstSetIterable<sptam::Map::SharedKeyFrame>::from( adjustable_keyframes ),
    ConstSetIterable<sptam::Map::SharedKeyFrame>::from( fixed_keyframes )
  );

  #ifdef SHOW_PROFILING
  }
  #endif
}
