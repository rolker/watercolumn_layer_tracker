#include "tracker_node.h"

namespace layer_tracker
{

TrackerNode::TrackerNode(int &argc, char ** argv)
{
  ros::init(argc, argv, "watercolumn_layer_tracker");
  node_handle_ = std::make_unique<ros::NodeHandle>();
  ros::NodeHandle private_node_handle("~");
  auto bin_size = private_node_handle.param("bin_size", 0.25);
  tracker_ = std::make_shared<Tracker>(bin_size);
}

void TrackerNode::setTopic(std::string topic)
{
  if(topic != topic_)
  {
    subscriber_.shutdown();
    if(!topic.empty())
      subscriber_ = node_handle_->subscribe(topic, 5, &TrackerNode::pingCallback, this);
    topic_ = topic;
  }
}

void TrackerNode::pingCallback(const marine_acoustic_msgs::RawSonarImage::ConstPtr& message)
{
  tracker_->addPing(*message);
}

std::shared_ptr<Tracker> TrackerNode::tracker()
{
  return tracker_;
}



} // namespace layer_tracker
