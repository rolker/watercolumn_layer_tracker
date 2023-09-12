#include "ping.h"

#include "PolynomialRegression.h"
#include <QDebug>

namespace layer_tracker
{

Ping::Ping()
  :bin_size_(0.0)
{

}

Ping::Ping(const marine_acoustic_msgs::RawSonarImage& message, float bin_size)
  :bin_size_(bin_size)
{
  uint16_t beam_number = message.image.beam_count/2;
  float max_range = (message.sample0+message.samples_per_beam)*message.ping_info.sound_speed/(2.0*message.sample_rate);
  uint32_t bin_count = ceil(max_range/bin_size);
  values_.resize(bin_count);
  max_values_.resize(bin_count);
  std::vector<int> counts(bin_count, 0);
  if(message.image.dtype == marine_acoustic_msgs::SonarImageData::DTYPE_FLOAT32)
  {
    for(uint32_t i = 0; i < message.samples_per_beam; i++)
    {
      float range = (message.sample0+i)*message.ping_info.sound_speed/(2.0*message.sample_rate);
      uint32_t bin_number = std::min<uint32_t>(bin_count-1, range/bin_size);
      float value = reinterpret_cast<const float*>(message.image.data.data())[beam_number*message.samples_per_beam+i];
      values_[bin_number] += value;
      if(counts[bin_number] == 0)
        max_values_[bin_number] = value;
      else
        max_values_[bin_number] = std::max(max_values_[bin_number], value);
      counts[bin_number] += 1;
    }
  }

  std::vector<float> ranges;
  for(uint32_t i = 0; i < bin_count; i++)
  {
    if(counts[i] > 1)
      values_[i]/=float(counts[i]);
    ranges.push_back(i*bin_size_);
  }

  PolynomialRegression<float> regression;

  std::vector<float> coefficients;
  if(!regression.fitIt(ranges, values_, 2, coefficients))
    coefficients = std::vector<float>(3, 0.0);

  for(uint32_t i = 0; i < bin_count; i++)
  {
    float noise = coefficients[0]+coefficients[1]*ranges[i]+coefficients[2]*ranges[i]*ranges[i];
    values_re_noise_.push_back(values_[i]-noise);
  }

  coefficients.clear();
  if(!regression.fitIt(ranges, max_values_, 2, coefficients))
    coefficients = std::vector<float>(3, 0.0);

  for(uint32_t i = 0; i < bin_count; i++)
  {
    float noise = coefficients[0]+coefficients[1]*ranges[i]+coefficients[2]*ranges[i]*ranges[i];
    max_values_re_noise_.push_back(max_values_[i]-noise);
  }
}

const std::vector<float>& Ping::values() const
{
  return values_;
}

const std::vector<float>& Ping::maxValues() const
{
  return max_values_;
}

const std::vector<float>& Ping::valuesReNoise() const
{
  return values_re_noise_;
}

const std::vector<float>& Ping::maxValuesReNoise() const
{
  return max_values_re_noise_;
}


float Ping::binSize() const
{
  return bin_size_;
}

}

