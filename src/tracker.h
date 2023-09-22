#ifndef WATERCOLUMN_LAYER_TRACKER_TRACKER_H
#define WATERCOLUMN_LAYER_TRACKER_TRACKER_H

#include "marine_acoustic_msgs/RawSonarImage.h"
#include "ping.h"
#include "slice.h"
#include "layer.h"

namespace layer_tracker
{

/// Finds layers in pings
class Tracker
{
public:
  Tracker(float bin_size);

  void addPing(const marine_acoustic_msgs::RawSonarImage& message);

  std::vector<std::shared_ptr<Ping> > pings();

  bool haveSlices(ros::Time timestamp);
  std::vector<Slice> slices(ros::Time timestamp);

  float binSize() const;

  void setLayerMinimumLevel(float minimum_level);
  void setLayerMinimumDepth(float minimum_depth);
  void setLayerMaximumDepth(float maximum_depth);
  void setLayerMinimumSize(float minimum_size);
  void setLayerMaximumSize(float maximum_size);

  std::vector<Layer> getLayers();
  std::vector<Layer> getCurrentLayers();
  std::vector<Layer> getPastLayers();
  std::vector<Layer> getCandidateLayers();

private:
  void extractSlices(ros::Time timestamp);
  void updateLayers(ros::Time timestamp);

  /// size in meters of sample bins
  float bin_size_ = 0.25;

  std::map<ros::Time, std::shared_ptr<Ping> > pings_by_time_;
  std::map<ros::Time, std::vector<Slice> > slices_by_time_;

  /// Minimum level used to generate layers
  float layer_minimum_level_ = 1.0;

  /// Minimum depth to look for layers
  float layer_minimum_depth_ = 0.0;

  /// Maximum depth to look for layers
  float layer_maximum_depth_ = 11000.0; // A bit deeper than Challenger Deep, to default to full ocean coverage.

  /// Minimum size depth-wise in meters of a layer
  float layer_minimum_size_ = 1.0;

  /// Maximum size depth-wise in meter of a layer
  float layer_maximum_size_ = 25.0;

  std::vector<Layer> layers_;
  ros::Duration maximum_layer_gap_ = ros::Duration(10.0);
};

} // namespace layer_tracker

#endif
