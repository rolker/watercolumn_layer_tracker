#ifndef WATERCOLUMN_LAYER_TRACKER_LAYER_H
#define WATERCOLUMN_LAYER_TRACKER_LAYER_H

#include "slice.h"

namespace layer_tracker
{

class Layer
{
public:
  Layer(const Slice &slice, float maximum_size);

  /// Layer's average dB level
  float averageDB() const;

  /// Scores the suitability of a slice to join the layer
  float potentialScore(const Slice &candidate_slice, ros::Time earliest_considered) const;

  void addSlice(const Slice &slice);

  ros::Time startTime() const;
  ros::Time endTime() const;
  ros::Duration duration() const;

  std::map<ros::Time, std::pair<float, float> > depthRangesByTime() const;
private:

  std::map<ros::Time, std::vector<Slice> > slices_;

  /// A slice that encompasses all slices for a given ping
  std::map<ros::Time, Slice> meta_slices_;

  float maximum_size_;
};

} // namespace layer_tracker

#endif
