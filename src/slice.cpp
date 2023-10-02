#include "slice.h"

namespace layer_tracker
{

Slice::Slice(std::shared_ptr<Ping> ping, float min_depth, float max_depth, float average_db)
  :ping_(ping), minimum_depth_(min_depth), maximum_depth_(max_depth), average_db_(average_db)
{
}

Slice::Slice(std::shared_ptr<Ping> ping, float min_depth, float max_depth)
  :ping_(ping), minimum_depth_(min_depth), maximum_depth_(max_depth)
{
  if(ping)
  {
    int i_begin = std::max<int>(0, minimum_depth_/ping->binSize());
    int i_end = std::min<int>(ping->valuesReBackground().size(), maximum_depth_/ping->binSize());
    if(i_end > i_begin)
    {
      double sum = 0.0;
      for(int i = i_begin; i < i_end; i++)
        sum += ping->valuesReBackground()[i];
      average_db_ = sum/float(i_end-i_begin);
    }
  }
}

std::shared_ptr<Ping> Slice::ping() const
{
  return ping_;
}

ros::Time Slice::timestamp() const
{
  if(ping_)
    return ping_->timestamp();
  return {};
}

const float &Slice::minimumDepth() const
{
  return minimum_depth_;
}

const float &Slice::maximumDepth() const
{
  return maximum_depth_;
}

const float &Slice::averageDB() const
{
  return average_db_;
}

float Slice::totalDB() const
{
  return average_db_*depthRange();
}

float Slice::score() const
{
  return totalDB();
}

bool Slice::operator<(const Slice& other) const
{
  if(score() < other.score())
    return true;
  if(score() > other.score())
    return false;
  // same score
  if(depthRange() < other.depthRange())
    return true;
  if(depthRange() > other.depthRange())
    return false;
  //same size
  if(minimum_depth_ > other.minimum_depth_)
    return true;
  return false;
}

bool Slice::overlaps(const Slice& other) const
{
  return maximum_depth_ > other.minimum_depth_ && minimum_depth_ < other.maximum_depth_;
}

float Slice::overlapRange(const Slice& other) const
{
  if(overlaps(other))
  {
    float max_min_depth = std::max(minimum_depth_, other.minimum_depth_);
    float min_max_depth = std::min(maximum_depth_, other.maximum_depth_);
    return  min_max_depth-max_min_depth;
  }
  return 0.0;
}

float Slice::depthRange() const
{
  return maximum_depth_ - minimum_depth_;
}

float Slice::combinedRange(const Slice &other) const
{
  return std::max(maximum_depth_, other.maximum_depth_) - std::min(minimum_depth_, other.minimum_depth_);
}

float Slice::gapBetween(const Slice &other) const
{
  if(overlaps(other))
    return 0.0;
  return std::max(minimum_depth_, other.minimum_depth_) - std::min(maximum_depth_, other.maximum_depth_);
}


} // namespace layer_tracker
