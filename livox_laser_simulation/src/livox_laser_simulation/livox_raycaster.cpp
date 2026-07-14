#include "livox_laser_simulation/livox_raycaster.hh"

#include <gz/math/Quaternion.hh>

#include <algorithm>
#include <array>
#include <limits>

#include "livox_laser_simulation/csv_reader.hpp"

namespace livox_laser_simulation
{

bool LivoxRaycaster::LoadScanPattern(const std::string &_csv_file,
                                     int _line_count, double _point_rate)
{
  if (_line_count < 1 || _line_count > 256 || _point_rate <= 0.0)
    return false;

  std::vector<std::vector<double>> data;
  if (!CsvReader::ReadCsvFile(_csv_file, data))
    return false;

  scan_points_.clear();
  scan_points_.reserve(data.size());
  constexpr double pi = 3.14159265358979323846;
  constexpr double deg_to_rad = pi / 180.0;
  for (size_t i = 0; i < data.size(); ++i)
  {
    if (data[i].size() != 3)
      continue;
    scan_points_.push_back({data[i][1] * deg_to_rad,
      data[i][2] * deg_to_rad - pi / 2.0,
      static_cast<uint8_t>(i % static_cast<size_t>(_line_count))});
  }
  point_period_ns_ = 1e9 / _point_rate;
  return !scan_points_.empty();
}

void LivoxRaycaster::SetParameters(int _downsample, double _min_range,
                                   double _max_range, bool _use_inf)
{
  downsample_ = std::max(1, _downsample);
  min_range_ = _min_range;
  max_range_ = _max_range;
  use_inf_ = _use_inf;
}

bool LivoxRaycaster::SetScene(const gz::rendering::ScenePtr &_scene)
{
  if (!_scene)
    return false;
  ray_query_ = _scene->CreateRayQuery();
  return static_cast<bool>(ray_query_);
}

std::vector<LivoxRayResult> LivoxRaycaster::CastRays(
  size_t _start_index, size_t _num_rays, const gz::math::Pose3d &_sensor_pose)
{
  std::vector<LivoxRayResult> results;
  if (!ray_query_ || scan_points_.empty())
    return results;

  results.reserve((_num_rays + downsample_ - 1) / downsample_);
  const auto inverse_rotation = _sensor_pose.Rot().Inverse();
  const auto origin = _sensor_pose.Pos();
  std::array<size_t, 256> line_sample_counts{};
  const size_t downsample = static_cast<size_t>(downsample_);
  for (size_t block_start = 0; block_start < _num_rays;
       block_start += downsample)
  {
    // A fixed stride can repeatedly pick only a subset of interleaved lines
    // (for example, stride 10 selects lines 0 and 2 from a four-line pattern).
    // Pick the least represented line from each downsampling block instead.
    const size_t block_end = std::min(block_start + downsample, _num_rays);
    size_t offset = block_start;
    size_t best_count = std::numeric_limits<size_t>::max();
    for (size_t candidate = block_start; candidate < block_end; ++candidate)
    {
      const auto &candidate_scan =
        scan_points_[(_start_index + candidate) % scan_points_.size()];
      const size_t count = line_sample_counts[candidate_scan.line];
      if (count < best_count)
      {
        offset = candidate;
        best_count = count;
      }
    }

    const auto &scan = scan_points_[(_start_index + offset) % scan_points_.size()];
    ++line_sample_counts[scan.line];
    gz::math::Quaterniond scan_rotation;
    scan_rotation.Euler(0.0, scan.zenith, scan.azimuth);
    const auto local_direction = scan_rotation * gz::math::Vector3d::UnitX;
    const auto world_direction = _sensor_pose.Rot() * local_direction;

    ray_query_->SetOrigin(origin);
    ray_query_->SetDirection(world_direction);
    const auto hit = ray_query_->ClosestPoint();

    const double range = hit ? origin.Distance(hit.point) : max_range_;
    if ((!hit && !use_inf_) || range < min_range_ || range > max_range_)
      continue;

    LivoxRayResult result;
    const auto world_point = hit ? hit.point : origin + world_direction * range;
    result.point = inverse_rotation * (world_point - origin);
    result.intensity = hit ? 100.0 : 0.0;
    result.offset_time = static_cast<uint32_t>(std::min(
      point_period_ns_ * static_cast<double>(offset),
      static_cast<double>(std::numeric_limits<uint32_t>::max())));
    result.line = scan.line;
    results.push_back(result);
  }
  return results;
}

}  // namespace livox_laser_simulation
