#include <cstddef>
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
  std::cout << "请选择需要计算连通的地图文件:" << std::endl;
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
  size_t size = map.cols * map.rows;
  size = (size * size - 1) / 8 + 1;
  std::unique_ptr<uint8_t[]> ans = std::make_unique<uint8_t[]>(size);
  std::cout << "开始计算" << std::endl;
  auto start_time = tdttoolkit::Time::GetTimeNow();
  int last_percent = 0;
  for (size_t i = 0; i < map.cols; i++) {
    if (i * 100 / map.cols > last_percent) {
      last_percent = i * 100 / map.cols;
      auto cnt_time = tdttoolkit::Time::GetTimeNow();
      auto cost_time = cnt_time - start_time;
      std::cout << "已完成:" << last_percent << "% 已花费"
                << cost_time / 1000000.0 << "s" << std::endl
                << " 剩余时间:"
                << cost_time / (last_percent / 100.0) / 1000000.0 *
                       (1 - last_percent / 100.0)
                << "s" << std::endl;
    }
#pragma omp parallel for
    for (size_t j = 0; j < map.rows; j++) {
      for (size_t m = 0; m < map.cols; m++) {
        for (size_t n = 0; n < map.rows; n++) {
          cv::LineIterator it(map, cv::Point(i, j), cv::Point(m, n));
          bool con = true;
          for (size_t i = 0; i < it.count; i++, ++it) {
            if (*it.ptr == 0) {
              con = false;
              break;
            }
          }
          if (con) {
            size_t index =
                (i * map.rows + j) * map.rows * map.cols + m * map.rows + n;
            size_t byte = index / 8;
            int bit = index % 8;
            ans[byte] |= (1 << bit);

            size_t index2 =
                (m * map.rows + n) * map.rows * map.cols + i * map.rows + j;
            size_t byte2 = index2 / 8;
            int bit2 = index2 % 8;
            ans[byte2] |= (1 << bit2);
          }
        }
      }
    }
  }
  auto end_time = tdttoolkit::Time::GetTimeNow();
  std::cout << "计算完成，耗时" << (end_time - start_time) / 1000000.0 << "s"
            << std::endl;
  std::cout << "开始保存" << std::endl;
  auto save_path = path / "con_map";
  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }
  std::ofstream out(
      (save_path / (file1.path().stem().string() + ".map")).string(),
      std::ios::out | std::ios::binary);
  out.write((char *)ans.get(), size);
  out.close();
  std::cout << "保存完成" << std::endl;
  return 0;
}