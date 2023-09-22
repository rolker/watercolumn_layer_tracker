#include "layer.h"

namespace layer_tracker
{

Layer::Layer(const Slice &slice, float maximum_size)
  :maximum_size_(maximum_size)
{
  addSlice(slice);
}

float Layer::averageDB() const
{
  if(slices_.empty())
    return 0.0;
  double sum = 0.0;
  double weights = 0.0;
  for(auto &meta_slice: meta_slices_)
  {
    float depth_range = meta_slice.second.depthRange();
    sum += meta_slice.second.averageDB()*depth_range;
    weights += depth_range;
  }
  return sum/weights;
}

float Layer::potentialScore(const Slice &candidate_slice, ros::Time earliest_considered) const
{
  if(slices_.empty())
    return 1.0;

  // calculate an average slice from recent pings
  double min_sum = 0.0;
  double max_sum = 0.0;
  double level_sum = 0.0;
  double weights = 0.0;
  for(auto meta_slice_iterator = meta_slices_.rbegin(); meta_slice_iterator != meta_slices_.rend() && meta_slice_iterator->first >= earliest_considered; meta_slice_iterator++)
  {
    auto & meta_slice = meta_slice_iterator->second;
    float weight = meta_slice.depthRange();
    min_sum += meta_slice.minimumDepth()*weight;
    max_sum += meta_slice.maximumDepth()*weight;
    level_sum += meta_slice.averageDB()*weight;
    weights += weight;
  }

  Slice s(std::shared_ptr<Ping>(), min_sum/weights, max_sum/weights, level_sum/weights);

  if(s.combinedRange(candidate_slice) <= maximum_size_*1.1) // allow a buffer to let layer move up or down
  {
    float score = 0.0;
    float overlap_range = s.overlapRange(candidate_slice);
    if(overlap_range > 0.0)
      score = overlap_range*s.averageDB();
    else
      score = s.averageDB()*(1.0 - (s.gapBetween(candidate_slice)/s.combinedRange(candidate_slice)));


    return std::max(0.0f, .5f - s.gapBetween(candidate_slice)/s.depthRange());
  }

  return 0.0;

}

void Layer::addSlice(const Slice &slice)
{
  auto timestamp = slice.timestamp();
  slices_[timestamp].push_back(slice);
  if(slices_[timestamp].size() == 1)
    meta_slices_[timestamp] = slice;
  else
  {
    float min_depth = slice.minimumDepth();
    float max_depth = slice.maximumDepth();
    for(auto& s: slices_[timestamp])
    {
      min_depth = std::min(min_depth, s.minimumDepth());
      max_depth = std::max(max_depth, s.maximumDepth());
    }
    meta_slices_[timestamp] = Slice(slice.ping(), min_depth, max_depth);
  }
}

std::map<ros::Time, std::pair<float, float> > Layer::depthRangesByTime() const
{
  std::map<ros::Time, std::pair<float, float> > ret;
  for(auto slices_pair: slices_)
  {
    const auto& slices = slices_pair.second;
    if(!slices.empty())
    {
      float min_depth = slices.front().minimumDepth();
      float max_depth = slices.front().maximumDepth();
      for(const auto& slice: slices)
      {
        min_depth = std::min(min_depth, slice.minimumDepth());
        max_depth = std::max(max_depth, slice.maximumDepth());
      }
      ret[slices_pair.first] = std::make_pair(min_depth, max_depth);
    }
  }
  return ret;
}

ros::Time Layer::startTime() const
{
  if(slices_.empty())
    return {};
  return slices_.begin()->first;
}

ros::Time Layer::endTime() const
{
  if(slices_.empty())
    return {};
  return slices_.rbegin()->first;
}

ros::Duration Layer::duration() const
{
  return endTime() - startTime();
}


} // namespace layer_tracker
