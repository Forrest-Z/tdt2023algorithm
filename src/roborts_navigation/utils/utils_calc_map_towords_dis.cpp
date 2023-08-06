#include <opencv2/core/hal/interface.h>
#include <roborts_utils/base_msg.h>

#include <cstddef>
#include <cstdint>
#include <opencv2/highgui.hpp>
#pragma GCC optimize(2)
#include <omp.h>
#include <roborts_utils/base_param.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/directory.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "roborts_utils/base_param.h"
#include "roborts_utils/base_toolkit.h"

int main() {
  tdttoolkit::Time::Init();
  auto path = boost::filesystem::path(__FILE__).parent_path().parent_path();
  auto map_path = path / "show_map";
  // auto map_info_path = path / "map_info";
  std::cout << "请选择需要计算各方向上障碍物距离的地图文件:" << std::endl;
  std::vector<boost::filesystem::directory_entry> maps;
  // std::vector<boost::filesystem::path> map_infos;
  int index = 0;
  for (auto &p : boost::filesystem::directory_iterator(map_path)) {
    if (!p.path().has_extension() || p.path().extension() != ".png") continue;
    // if (boost::filesystem::is_regular_file(
    //         map_info_path / (p.path().stem().string() + ".jsonc"))) {
    std::cout << std::to_string(index) + ". " << p.path().stem() << std::endl;
    maps.push_back(p);
    // map_infos.push_back(
    //     (map_info_path / (p.path().stem().string() + ".jsonc")));
    index++;
    // }
  }
  int i;
  std::cin >> i;

  if (i < 0 || i >= maps.size()) {
    std::cout << "输入错误" << std::endl;
    return 0;
  }
  auto file1 = maps[i];
  // auto file2 = map_infos[i];
  std::cout << "您选择的文件是:" << file1.path().stem() << std::endl;
  auto map = cv::imread(file1.path().string(), cv::IMREAD_GRAYSCALE);
  // LoadParam::InitParam("map", file2.string());
  size_t size = map.cols * map.rows * 8;
  std::unique_ptr<uint32_t[]> ans = std::make_unique<uint32_t[]>(size);
  std::cout << "开始计算" << std::endl;
  auto start_time = tdttoolkit::Time::GetTimeNow();
#pragma omp parallel for
  for (long long i = 0; i < map.cols; i++) {
    for (long long j = 0; j < map.rows; j++) {
      for (int t = 0; t < 8; t++) {
        uint32_t l = 0;
        for (l = 0;; l++) {
          bool stop = false;
          switch (t) {
            case 0:  // 0 为 x轴正方向
              if (i + l >= map.cols) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j, i + l) == 0) stop = true;
              break;
            case 1:  // 1 为 x正y正方向
              if (i + l >= map.cols || j + l >= map.rows) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j + l, i + l) == 0) stop = true;
              break;
            case 2:  // 2 为 y轴正方向
              if (j + l >= map.rows) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j + l, i) == 0) stop = true;
              break;
            case 3:  // 3 为 x负y正方向
              if (i - l < 0 || j + l >= map.rows) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j + l, i - l) == 0) stop = true;
              break;
            case 4:  // 4 为 x轴负方向
              if (i - l < 0) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j, i - l) == 0) stop = true;
              break;
            case 5:  // 5 为 x负y负方向
              if (i - l < 0 || j - l < 0) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j - l, i - l) == 0) stop = true;
              break;
            case 6:  // 6 为 y轴负方向
              if (j - l < 0) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j - l, i) == 0) stop = true;
              break;
            case 7:  // 7 为 x正y负方向
              if (i + l >= map.cols || j - l < 0) {
                stop = true;
                break;
              }
              if (map.at<uint8_t>(j - l, i + l) == 0) stop = true;
              break;
            default:
              [[unlikely]] TDT_FATAL("错误的方向");
          }
          if (stop) break;
        }
        ans[i * map.rows * 8 + j * 8 + t] = t % 2 ? l * 1.414 : l;
      }
    }
  }
  auto end_time = tdttoolkit::Time::GetTimeNow();
  std::cout << "计算完成，耗时" << (end_time - start_time) / 1000000.0 << "s"
            << std::endl;
  std::cout << "开始保存" << std::endl;
  auto save_path = path / "cost_map/towords_dis_map";
  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }
  std::ofstream out(
      (save_path / (file1.path().stem().string() + ".map")).string(),
      std::ios::out | std::ios::binary);
  out.write((char *)ans.get(), size * sizeof(uint32_t));
  out.close();
  std::cout << "保存完成" << std::endl;

  // debug
  for (int t = 0; t < 8; t++) {
    auto show = cv::Mat(map.rows, map.cols, CV_8UC1);
    for (size_t i = 0; i < map.cols; i++) {
      for (size_t j = 0; j < map.rows; j++) {
        show.at<uchar>(j, i) = ans[i * map.rows * 8 + j * 8 + t];
      }
    }
    cv::imshow("show" + std::to_string(t), show);
  }
  cv::waitKey(0);
  return 0;
}