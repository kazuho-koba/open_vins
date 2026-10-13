// ROSやEigenに依存せず、欠測時の更新前検査を再現する単体試験。
// 実行例: g++ -std=c++14 -Iov_msckf/src このファイル -o /tmp/imu_interval_test
#include "state/ImuIntervalCheck.h"
#include <cassert>
#include <limits>

struct Measurement {
  double timestamp;
};

int main() {
  using ov_msckf::has_complete_imu_interval;
  using Series = std::vector<Measurement>;
  assert(has_complete_imu_interval(Series{{1.0}, {1.01}, {1.02}}, 1.0, 1.02));
  assert(!has_complete_imu_interval(Series{}, 1.0, 1.02));
  assert(!has_complete_imu_interval(Series{{1.0}}, 1.0, 1.02));
  assert(!has_complete_imu_interval(Series{{1.01}, {1.02}}, 1.0, 1.02));
  assert(!has_complete_imu_interval(Series{{1.0}, {1.01}}, 1.0, 1.02));
  assert(!has_complete_imu_interval(Series{{1.0}, {1.0}, {1.02}}, 1.0, 1.02));
  assert(!has_complete_imu_interval(Series{{1.0}, {1.02}, {1.01}, {1.02}}, 1.0, 1.02));
  assert(!has_complete_imu_interval(Series{{1.0}, {std::numeric_limits<double>::quiet_NaN()}, {1.02}}, 1.0, 1.02));
  // 実ログの失敗区間: 保持データの先頭が要求終了時刻より後。積分してはいけない。
  assert(!has_complete_imu_interval(Series{{1791626219.584646940}, {1791626219.868165970}},
                                    1791626219.474582911, 1791626219.574581861));
  // 端点が揃っていても内部欠測の品質は別問題。ここで補間やgap閾値を勝手に追加しない。
  assert(has_complete_imu_interval(Series{{1.0}, {1.3}}, 1.0, 1.3));
}
