#ifndef LIVOX_LASER_SIMULATION_LIVOX_RAYCASTER_HH_
#define LIVOX_LASER_SIMULATION_LIVOX_RAYCASTER_HH_

#include <gz/math/Pose3.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace livox_laser_simulation
{

struct ScanPoint
{
  double azimuth;
  double zenith;
  uint8_t line;
};

struct LivoxRayResult
{
  gz::math::Vector3d point;
  double intensity{0.0};
  uint32_t offset_time{0};
  uint8_t line{0};
};

class LivoxRaycaster
{
public:
  bool LoadScanPattern(const std::string &_csv_file, int _line_count,
                       double _point_rate);
  void SetParameters(int _downsample, double _min_range, double _max_range,
                     bool _use_inf);
  bool SetScene(const gz::rendering::ScenePtr &_scene);

  std::vector<LivoxRayResult> CastRays(size_t _start_index, size_t _num_rays,
                                      const gz::math::Pose3d &_sensor_pose);

  size_t PointCount() const { return scan_points_.size(); }

private:
  std::vector<ScanPoint> scan_points_;
  gz::rendering::RayQueryPtr ray_query_;
  int downsample_{1};
  double min_range_{0.1};
  double max_range_{40.0};
  double point_period_ns_{5000.0};
  bool use_inf_{false};
};

}  // namespace livox_laser_simulation

#endif  // LIVOX_LASER_SIMULATION_LIVOX_RAYCASTER_HH_
