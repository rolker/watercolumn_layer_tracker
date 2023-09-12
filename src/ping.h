#include "marine_acoustic_msgs/RawSonarImage.h"

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
    private:
      /// size in meters of bins
      float bin_size_;

      std::vector<float> values_;
      std::vector<float> max_values_;
      std::vector<float> values_re_noise_;
      std::vector<float> max_values_re_noise_;
  };

} // namepsace layer_tracker