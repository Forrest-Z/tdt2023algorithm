//
// Created by castle on 2020/1/14.
//

#ifndef VIDEORECODER_VIDEORECODER_H
#define VIDEORECODER_VIDEORECODER_H

#include <opencv2/videoio/videoio_c.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <string>
namespace robots_perception {
class VideoRecoder {
 public:
  VideoRecoder();
  void Init(const std::string &path = "../../video");

  void Recorder(cv::Mat src);
  void Recorder_(std::list<cv::Mat> *imgs);

  void Release() {
    // if (!init_flag_) return;
    std::cout<<"sleep前"<<std::endl;
    usleep(500000);
        std::cout<<"sleep后"<<std::endl;

    // sleep(2);
    video_.release();
        std::cout<<"release后"<<std::endl;

    // exit(0);
  }

 private:
int skip_frame;
int total_frame;
  cv::VideoWriter video_;
  bool init_flag_;
  std::string path_;
  int start1;
  std::list<cv::Mat> *frames;
  int k;
  std::chrono::steady_clock::time_point start_time;
};
}  // namespace robots_perception

#endif  // VIDEORECODER_VIDEORECODER_H
