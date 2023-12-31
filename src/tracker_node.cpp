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

// #include "tracker.h"
// #include <project11_msgs/AcousticLayer.h>
// #include <marine_acoustic_msgs/RawSonarImage.h>

// std::shared_ptr<layer_tracker::Tracker> tracker;
// ros::Publisher layer_publisher;

// void pingCallback(const marine_acoustic_msgs::RawSonarImage::ConstPtr &ping_message)
// {
//   tracker->addPing(*ping_message);
// }

// int main(int argc, char *argv[])
// {
//   ros::NodeHandle node_handle("~");

//   float bin_size = node_handle.param("bin_size", .25f);

//   float min_intensity = node_handle.param("minimum_intensity", 2.0);
//   float min_depth = node_handle.param("minimum_depth", 0.0f);
//   float max_depth = node_handle.param("maximum_depth", 11000.0f);
//   float min_size = node_handle.param("minimum_size", 1.0f);
//   float max_size = node_handle.param("maximum_size", 30.0f);

//   tracker = std::make_shared<layer_tracker::Tracker>(bin_size);

//   tracker->setLayerMinimumLevel(min_intensity);
//   tracker->setLayerMinimumDepth(min_depth);
//   tracker->setLayerMaximumDepth(max_depth);
//   tracker->setLayerMinimumSize(min_size);
//   tracker->setLayerMaximumSize(max_size);

//   layer_publisher = node_handle.advertise<project11_msgs::AcousticLayer>("layers", 10);

//   ros::Subscriber ping_subscriber = node_handle.subscribe("pings", 10, &pingCallback);

// }
