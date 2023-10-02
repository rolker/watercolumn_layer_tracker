#ifndef WATERCOLUMN_LAYER_TRACKER_SLICE_H
#define WATERCOLUMN_LAYER_TRACKER_SLICE_H

#include <ros/ros.h>
#include "ping.h"

namespace layer_tracker
{

/// Represent a portion of a ping that may belong
/// to a tracked layer.
class Slice
{
public:
  Slice() = default;
  Slice(std::shared_ptr<Ping> ping, float min_depth, float max_depth);
  Slice(std::shared_ptr<Ping> ping, float min_depth, float max_depth, float average_db);

  ros::Time timestamp() const;
  std::shared_ptr<Ping> ping() const;

  const float& minimumDepth() const;
  const float& maximumDepth() const;

  float depthRange() const;

  const float& averageDB() const;
  float totalDB() const;

  float score() const;
  bool operator<(const Slice &other) const;
  bool overlaps(const Slice &other) const;

  /// range of the intersection
  float overlapRange(const Slice &other) const;

  /// range of the union
  float combinedRange(const Slice &other) const;
  
  float gapBetween(const Slice &other) const;


private:
  std::shared_ptr<Ping> ping_;

  /// distance in meters from sonar the slice begins.
  float minimum_depth_ = 0.0;

  /// distance in meters from the sonar where the slice ends.
  float maximum_depth_ = 0.0;

  /// average level above noise floor in decibels.
  float average_db_ = 0.0;
};

} // namespace layer_tracker

#endif
