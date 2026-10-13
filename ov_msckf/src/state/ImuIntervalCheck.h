#ifndef OV_MSCKF_IMU_INTERVAL_CHECK_H
#define OV_MSCKF_IMU_INTERVAL_CHECK_H

#include <cmath>
#include <vector>

namespace ov_msckf {

/// 抽出したIMU系列が要求積分区間の両端を持つか、状態更新前に検査する。
/// 時刻は秒。1 usの許容差はUnix秒表現の丸めを吸収するもので、欠測を埋める幅ではない。
/// 区間内部の長い欠測や外挿の妥当性は、この端点検査だけでは保証できない。
template <typename Measurement>
bool has_complete_imu_interval(const std::vector<Measurement> &data, double time0, double time1) {
  constexpr double tolerance_s = 1e-6;
  if (data.size() < 2 || !std::isfinite(time0) || !std::isfinite(time1) || time1 <= time0 ||
      !std::isfinite(data.front().timestamp) || !std::isfinite(data.back().timestamp) ||
      std::abs(data.front().timestamp - time0) > tolerance_s ||
      std::abs(data.back().timestamp - time1) > tolerance_s) {
    return false;
  }
  // 積分途中の逆順・同時刻・NaNも、predictで状態を部分変更する前に拒否する。
  for (size_t i = 1; i < data.size(); ++i) {
    if (!std::isfinite(data[i].timestamp) || data[i].timestamp <= data[i - 1].timestamp) {
      return false;
    }
  }
  return true;
}

} // namespace ov_msckf
#endif
