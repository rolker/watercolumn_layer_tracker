#include "tracker.h"

namespace layer_tracker
{

Tracker::Tracker(float bin_size)
  :bin_size_(bin_size)
{

}

void Tracker::addPing(const marine_acoustic_msgs::RawSonarImage& message)
{
  pings_by_time_[message.header.stamp] = std::make_shared<Ping>(message, bin_size_);
}

std::vector<std::shared_ptr<Ping> > Tracker::pings()
{
  std::vector<std::shared_ptr<Ping> > ret;
  for(auto p: pings_by_time_)
    ret.push_back(p.second);
  return ret;
}

bool Tracker::haveSlices(ros::Time timestamp)
{
  return slices_by_time_.count(timestamp) > 0;
}

std::vector<Slice> Tracker::slices(ros::Time timestamp)
{
  auto s = slices_by_time_.find(timestamp);
  if(s != slices_by_time_.end())
    return s->second;

  extractSlices(timestamp);
  s = slices_by_time_.find(timestamp);
  if(s != slices_by_time_.end())
    return s->second;
  return {};
}

float Tracker::binSize() const
{
  return bin_size_;
}

void Tracker::setLayerMinimumLevel(float minimum_level)
{
  layer_minimum_level_ = minimum_level;
  slices_by_time_.clear();
  layers_.clear();
}

void Tracker::setLayerMinimumDepth(float minimum_depth)
{
  layer_minimum_depth_ = minimum_depth;
  slices_by_time_.clear();
  layers_.clear();
}

void Tracker::setLayerMaximumDepth(float maximum_depth)
{
  layer_maximum_depth_ = maximum_depth;
  slices_by_time_.clear();
  layers_.clear();
}

void Tracker::setLayerMinimumSize(float minimum_size)
{
  layer_minimum_size_ = minimum_size;
  slices_by_time_.clear();
  layers_.clear();
}

void Tracker::setLayerMaximumSize(float maximum_size)
{
  layer_maximum_size_ = maximum_size;
  slices_by_time_.clear();
  layers_.clear();
}

void Tracker::extractSlices(ros::Time timestamp)
{
  auto ping_iterator = pings_by_time_.find(timestamp);
  if(ping_iterator == pings_by_time_.end())
    return;
  auto ping = ping_iterator->second;
  auto bin_size = ping->binSize();

  int start_bin =  layer_minimum_depth_/bin_size;
  int end_bin = layer_maximum_depth_/bin_size;
  int min_bins = layer_minimum_size_/bin_size;
  int max_bins = layer_maximum_size_/bin_size;

  std::map<Slice, float> candidates;

  for(int i = start_bin; i < end_bin &&  i < ping->valuesReBackground().size()-min_bins; i++)
  {
    // bail out early if we are starting below min
    if(ping->valuesReBackground()[i] < layer_minimum_level_)
      continue;

    double sum = 0.0;
    auto max = ping->valuesReBackground()[i];

    Slice max_slice;
    for(int j = 0; j < max_bins && i+j < ping->valuesReBackground().size(); j++)
    {
      sum += ping->valuesReBackground()[i+j];
      max = std::max(max, ping->valuesReBackground()[i+j]);
      int bin_count = j+1;
      float average = sum/float(bin_count);
      //if(bin_count == min_bins && average < min_db)
      //  break;
      if(bin_count >= min_bins && average >= layer_minimum_level_)
      {
        Slice s(ping, i*bin_size, (i+j)*bin_size, average);
        if(s.score() > max_slice.score())
          max_slice = s;
      }
    }
    if(max_slice.averageDB() > layer_minimum_level_)
      candidates[max_slice] = max_slice.score();

  }

  std::vector<Slice> slices;

  for(auto slice = candidates.rbegin(); slice != candidates.rend(); slice++)
  {
    bool overlaps = false;
    for(auto & existing_slice: slices)
      if(existing_slice.overlaps(slice->first))
      {
        overlaps = true;
        break;
      }
    if(!overlaps)
      slices.push_back(slice->first);
  }

  slices_by_time_[timestamp] = slices;
  updateLayers(timestamp);
}

void Tracker::updateLayers(ros::Time timestamp)
{
  auto slices_iterator = slices_by_time_.find(timestamp);
  if(slices_iterator == slices_by_time_.end())
    return;
  auto slices = slices_iterator->second;
  for(const auto& slice: slices)
  {
    int best_layer = -1;
    float best_layer_score = 0.0;
    
    for(int i = 0; i < layers_.size(); i++)
    {
      const auto &layer = layers_[i];
      // is it a current layer, which has a slice within gap time?
      if( (timestamp-layer.endTime()) < maximum_layer_gap_)
      {
        auto layer_score = layer.potentialScore(slice, timestamp-maximum_layer_gap_);
        if(layer_score > best_layer_score)
        {
          best_layer = i;
          best_layer_score = layer_score;
        }
      }
    }

    if(best_layer >= 0)
    {
      layers_[best_layer].addSlice(slice);
    }
    else
    {
      layers_.push_back(Layer(slice, layer_maximum_size_));
    }
  }
}

std::vector<Layer> Tracker::getLayers()
{
  return layers_;
}

std::vector<Layer> Tracker::getCurrentLayers()
{
  std::vector<Layer> ret;
  if(!pings_by_time_.empty())
  {
    auto timestamp = pings_by_time_.rbegin()->first;
    for(const auto& layer: layers_)
    {
      if(timestamp - layer.endTime() < maximum_layer_gap_)
        if(layer.duration() >= maximum_layer_gap_)
          ret.push_back(layer);
    }
  }
  return ret;
}


std::vector<Layer> Tracker::getPastLayers()
{
  std::vector<Layer> ret;
  if(!pings_by_time_.empty())
  {
    auto timestamp = pings_by_time_.rbegin()->first;
    for(const auto& layer: layers_)
    {
      if(timestamp - layer.endTime() > maximum_layer_gap_)
        if(layer.duration() >= maximum_layer_gap_)
          ret.push_back(layer);
    }
  }
  return ret;
}

std::vector<Layer> Tracker::getCandidateLayers()
{
  std::vector<Layer> ret;
  if(!pings_by_time_.empty())
  {
    auto timestamp = pings_by_time_.rbegin()->first;
    for(const auto& layer: layers_)
    {
      if(timestamp - layer.endTime() < maximum_layer_gap_)
        if(layer.duration() < maximum_layer_gap_)
          ret.push_back(layer);
    }
  }
  return ret;
}


} // namespace layer_tracker
