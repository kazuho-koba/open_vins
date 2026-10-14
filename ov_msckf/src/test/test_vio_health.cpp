// 実機を開かず、欠落判定→診断topic→heartbeatの経路を検証するfixture。
// argv[1]は校正YAML。running表示だけはcacheへ注入し、真のVIO初期化試験とは区別する。
#include "core/VioManager.h"
#include "ros/ROS2Visualizer.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include <iostream>
#include <stdexcept>
#include <thread>

class HealthFixture : public ov_msckf::ROS2Visualizer {
public:
  using ROS2Visualizer::ROS2Visualizer;
  void running_for_test() { health_initialized = true; publish_health(); }
};

int main(int argc, char **argv) {
  if (argc < 2) return 2;
  const std::string config = argv[1];
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("health_fixture", "/ov_msckf");
  auto parser = std::make_shared<ov_core::YamlParser>(config);
  ov_msckf::VioManagerOptions options;
  options.print_and_load(parser);
  auto app = std::make_shared<ov_msckf::VioManager>(options);
  auto viz = std::make_shared<HealthFixture>(node, app);
  std::vector<diagnostic_msgs::msg::DiagnosticArray> received;
  auto sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics/health", rclcpp::QoS(10).reliable().transient_local(),
      [&received](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { received.push_back(*msg); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_for = [&executor](int milliseconds) {
    const auto end = std::chrono::steady_clock::now()+std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now()<end) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  };
  spin_for(2200);
  viz->running_for_test();
  spin_for(1200);
  // 状態はIMU時刻1秒、保持IMUは2秒以降。実Propagatorの保護条件を踏ませる。
  app->get_state()->_timestamp = 1.0;
  app->get_state()->_calib_dt_CAMtoIMU->set_value(Eigen::VectorXd::Zero(1));
  auto prop = app->get_propagator();
  ov_core::ImuData imu;
  imu.timestamp = 2.0; imu.wm.setZero(); imu.am.setZero();
  prop->feed_imu(imu);
  imu.timestamp = 2.1; prop->feed_imu(imu);
  prop->propagate_and_clone(app->get_state(), 1.1);
  const auto evidence = prop->reset_evidence();
  if (!evidence.required || evidence.required_start!=1.0 || evidence.buffer_first!=2.0)
    throw std::runtime_error("停止証拠snapshotの不一致");
  viz->publish_health();
  spin_for(2200);
  int initializing=0, running=0, reset=0;
  for (const auto &msg : received) {
    const auto &status=msg.status.at(0);
    initializing += status.message=="initializing";
    running += status.message=="running";
    reset += status.message=="reset_required";
    if (status.message=="reset_required" && status.level!=diagnostic_msgs::msg::DiagnosticStatus::ERROR)
      throw std::runtime_error("reset_requiredの診断levelがERRORでない");
  }
  if (initializing<2 || running<1 || reset<2)
    throw std::runtime_error("状態遷移またはheartbeatを受信できない");
  std::cout << "HEALTH_TEST_PASS initializing=" << initializing << " running=" << running << " reset=" << reset << std::endl;
  viz->stop_workers(); executor.remove_node(node);
  viz.reset(); app.reset(); sub.reset(); node.reset();
  rclcpp::shutdown();
}
