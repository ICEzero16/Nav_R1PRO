// Offline reader only: no ROS node, sensor subscription or hardware command.
// g++ -std=c++17 -I/opt/ros/humble/include inspect_slam_dataset.cpp \
//   -L/opt/ros/humble/lib -Wl,-rpath,/opt/ros/humble/lib \
//   -lkartoSlamToolbox -lboost_serialization -lboost_thread -o inspect_slam_dataset
#include <karto_sdk/Karto.h>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::cout.setf(std::ios::unitbuf);
  try {
  karto::Dataset dataset;
  dataset.LoadFromFile(argv[1]);
  for (auto* object : dataset.GetLasers()) {
    auto* sensor = dynamic_cast<karto::Sensor*>(object);
    if (sensor) karto::SensorManager::GetInstance()->RegisterSensor(sensor, true);
  }
  std::cout << "DATASET_RECORDS " << dataset.GetData().size() << '\n';
  for (const auto& item : dataset.GetData()) {
    auto* scan = dynamic_cast<karto::LocalizedRangeScan*>(item.second);
    if (!scan) continue;
    const auto& odom = scan->GetOdometricPose();
    const auto& pose = scan->GetCorrectedPose();
    std::cout << "SCAN " << item.first << " time " << scan->GetTime()
              << " odom " << odom.GetX() << ' ' << odom.GetY() << ' ' << odom.GetHeading()
              << " corrected " << pose.GetX() << ' ' << pose.GetY() << ' ' << pose.GetHeading() << '\n';
  }
  } catch (const karto::Exception& e) {
    std::cerr << e.GetErrorMessage() << '\n';return 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';return 1;
  }
}
