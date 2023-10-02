#ifndef WATERCOLUMN_LAYER_TRACKER_TRACKER_NODE_H
#define WATERCOLUMN_LAYER_TRACKER_TRACKER_NODE_H

#include <ros/ros.h>
#include "marine_acoustic_msgs/RawSonarImage.h"

#include "tracker.h"

namespace layer_tracker
{

class TrackerNode
{
public:
  TrackerNode(int &argc, char ** argv);

  void setTopic(std::string topic);

  std::shared_ptr<Tracker> tracker();

private:
  void pingCallback(const marine_acoustic_msgs::RawSonarImage::ConstPtr& message);

  std::unique_ptr<ros::NodeHandle> node_handle_;
  std::string topic_;
  ros::Subscriber subscriber_;
  std::shared_ptr<Tracker> tracker_;
};

} // namespace layer_tracker

#endif
