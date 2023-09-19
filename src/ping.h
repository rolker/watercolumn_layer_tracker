#ifndef WATERCOLUMN_LAYER_TRACKER_PING_H
#define WATERCOLUMN_LAYER_TRACKER_PING_H

#include "marine_acoustic_msgs/RawSonarImage.h"

#include "slice.h"

namespace layer_tracker
{
  class Ping
  {
    public:
      Ping();
      Ping(const marine_acoustic_msgs::RawSonarImage& message, float bin_size);

      const std::vector<float>& values() const;
      const std::vector<float>& maxValues() const;
      const std::vector<float>& valuesReNoise() const;
      const std::vector<float>& maxValuesReNoise() const;

      float binSize() const;

      /// Find slices in a ping. min_db is minimum average db above noise for a valid slice. min_size and max_size are size bounds in meters of a slice.
      const std::vector<Slice>& extractSlices(float min_db = 3, float min_size = 1.0, float max_size = 50.0);

      const std::vector<Slice>& slices() const;

    private:
      /// size in meters of bins
      float bin_size_;

      std::vector<float> values_;
      std::vector<float> max_values_;
      std::vector<float> values_re_noise_;
      std::vector<float> max_values_re_noise_;

      std::vector<Slice> slices_;
  };

} // namepsace layer_tracker

#endif
