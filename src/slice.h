#ifndef WATERCOLUMN_LAYER_TRACKER_SLICE_H
#define WATERCOLUMN_LAYER_TRACKER_SLICE_H

namespace layer_tracker
{

/// Represent a portion of a ping that may belong
/// to a tracked layer.
class Slice
{
public:
  Slice() = default;
  Slice(float begin, float end, float average_db, float maximum_db);

  const float& begin() const;
  const float& end() const;
  const float& averageDB() const;
  const float& maximumDB() const;

  float score() const;
  bool operator<(const Slice &other) const;
  bool overlaps(const Slice &other) const;


private:
  /// distance in meters from sonar the slice begins.
  float begin_ = 0.0;

  /// distance in meters from the sonar where the slice ends.
  float end_ = 0.0;

  /// average level above noise floor in decibels.
  float average_db_ = 0.0;

  /// maximum level above noise floor in decibels.
  float maximum_db_ = 0.0;
};

} // namespace layer_tracker

#endif
