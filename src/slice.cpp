#include "slice.h"

namespace layer_tracker
{

Slice::Slice(float begin, float end, float average_db, float maximum_db)
  :begin_(begin), end_(end), average_db_(average_db), maximum_db_(maximum_db)
{
}


const float &Slice::begin() const
{
  return begin_;
}

const float &Slice::end() const
{
  return end_;
}

const float &Slice::averageDB() const
{
  return average_db_;
}

const float &Slice::maximumDB() const
{
  return maximum_db_;
}

float Slice::score() const
{
  return average_db_*(end_-begin_);
}

bool Slice::operator<(const Slice& other) const
{
  if(score() < other.score())
    return true;
  if(score() > other.score())
    return false;
  // same score
  if(end_-begin_ < other.end_ - other.begin_)
    return true;
  if(end_-begin_ > other.end_ - other.begin_)
    return false;
  //same size
  if(begin_ > other.begin_)
    return true;
  return false;
}

bool Slice::overlaps(const Slice& other) const
{
  return end_ > other.begin_ && begin_ < other.end_;
}

} // namespace layer_tracker
