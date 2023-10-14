
#include <ros/ros.h>
#include <project11_msgs/AcousticLayer.h>
#include <marine_acoustic_msgs/RawSonarImage.h>
#include <std_msgs/Float32.h>

ros::Subscriber sonar_subscription;
ros::Subscriber set_point_subscription;

ros::Publisher layer_publisher;

float set_point = 0.0;
float current_layer_range = 0.0;

project11_msgs::AcousticLayer current_layer;

struct Ping
{
  Ping(const marine_acoustic_msgs::RawSonarImage& message)
    :image(message)
  {
    min_range = 0.5*message.ping_info.sound_speed*message.sample0/message.sample_rate;
    max_range = 0.5*message.ping_info.sound_speed*(message.sample0+message.samples_per_beam)/message.sample_rate;
    bin_size = 0.5*message.ping_info.sound_speed/message.sample_rate;
  }
  const marine_acoustic_msgs::RawSonarImage& image;
  float min_range;
  float max_range;
  float bin_size;
  int rangeToIndex(float range)
  {
    return (range-min_range)/bin_size;
  }
  float valueAtRange(float range)
  {
    int i = rangeToIndex(range);
    if(i >= 0 && i < image.samples_per_beam)
      return reinterpret_cast<const float *>(image.image.data.data())[i];
    return std::nan("");
  }

};

float layer_thinkness = 10.0;

void sonarCallback(const marine_acoustic_msgs::RawSonarImage::ConstPtr& message)
{
  if(current_layer_range > 0.0)
  {
    Ping ping(*message);

    project11_msgs::AcousticSlice slice;
    slice.header = message->header;
    slice.ping_info = message->ping_info;
    slice.minimum_range = current_layer_range-layer_thinkness/2.0;
    slice.maximum_range = current_layer_range+layer_thinkness/2.0;

    float floor = 0.0;
    for (float range = slice.minimum_range; range <= slice.maximum_range; range += ping.bin_size)
    {
      floor = std::min(floor, ping.valueAtRange(range));
    }
    double value_sum = 0.0;
    double range_sum = 0.0;
    for (float range = slice.minimum_range; range <= slice.maximum_range; range += ping.bin_size)
    {
      slice.intensitites.push_back(ping.valueAtRange(range));
      float value = ping.valueAtRange(range)-floor;
      value_sum += value;
      range_sum += range*value;
    }
    slice.center_of_mass_range = range_sum/value_sum;
    current_layer_range = current_layer_range*0.5+slice.center_of_mass_range*0.5;
    slice.center_of_mass_range = current_layer_range;
    current_layer.slices.clear();
    current_layer.slices.push_back(slice);
    layer_publisher.publish(current_layer);
  }
}

void setPointCallback(const std_msgs::Float32::ConstPtr& message)
{
  set_point = message->data;
  current_layer_range = set_point;
  current_layer.slices.clear();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "single_layer_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  sonar_subscription = nh.subscribe("echogram", 10, &sonarCallback);
  set_point_subscription = private_nh.subscribe("track_layer_at_depth", 1, &setPointCallback);

  layer_publisher = private_nh.advertise<project11_msgs::AcousticLayer>("layer", 1);

  ros::spin();

  return 0;
}
