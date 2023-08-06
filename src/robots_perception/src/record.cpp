//
// Created by castle on 2020/1/14.
//

#include "record.h"

#include <stdio.h>
#include <string.h>
#include <sys/statfs.h>
#include <time.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <thread>
namespace robots_perception {

VideoRecoder::VideoRecoder() {}

std::string get_cur_executable_path_() {
  char *p = NULL;

  const int len = 256;
  /// to keep the absolute path of executable's path
  char arr_tmp[len] = {0};

  int n = readlink("/proc/self/exe", arr_tmp, len);
  if (NULL != (p = strrchr(arr_tmp, '/')))
    *p = '\0';
  else {
    printf("wrong process path");
    std::string("");
  }

  return std::string(arr_tmp);
}
void VideoRecoder::Init(const std::string &path) {
//   video_ = cv::VideoWriter();
  init_flag_ = false;
  k = 1;
  start1 = 10;
  start_time = std::chrono::steady_clock::now();
  path_ = path;
  skip_frame=3;
  total_frame=0;
  if (*(path_.end() - 1) == '/') {
    path_.erase(path_.end() - 1);
  }
  if (access(path_.c_str(), F_OK)) {
    mkdir(path_.c_str(), 0755);
  }
  uint64_t i = 0;
  char VideoName[50];
  //		while(true)
  //		{
  //			sprintf(num, "%ld.avi", i);
  //			if(access((path_ + "/" + num).c_str(), F_OK))
  //			{
  //				path_ = path_ + "/" + num;
  //				break;
  //			}
  //			i++;
  //		}
  struct tm p;
  time_t time_;
  time(&time_);
  p = *(gmtime(&time_));
  sprintf(VideoName, "%d-%d-%d--%d-%d-%d-%d.avi", p.tm_year + 1900,
          p.tm_mon + 1, p.tm_mday, p.tm_hour + 8, p.tm_min, p.tm_min, p.tm_sec);
  if (access((path_ + "/" + VideoName).c_str(), F_OK)) {
    path_ = path_ + "/" + VideoName;
  }
  if (video_.isOpened()) {
    video_.release();
  }
  init_flag_ = true;
  frames = new std::list<cv::Mat>;
}
void VideoRecoder::Recorder(cv::Mat src) {
  if (init_flag_) {
    if (src.size().width < 360 || src.size().height <360) {
      return;
    }
    video_.open(path_.c_str(), cv::CAP_FFMPEG, CV_FOURCC('M', 'J', 'P', 'G'),
                40, src.size());
    if (!video_.isOpened()) {
      // int wait_second = 5;
      // std::cout << "wait " << wait_second << "s for confirming...";
      // while (wait_second > 0) {
      //     sleep(1);
      //     wait_second--;
      //     std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" <<
      //     wait_second << "s for confirming..." << std::flush;
      // }
      std::cout<<"打开摄像头失败"<<std::endl;
    } else {
      frames->push_back(src);
      init_flag_ = false;
    }
  } else {
  total_frame++;

    if(total_frame%skip_frame==0)
    {
    std::cout<<"录制"<<std::endl;
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end_time - start_time)
                          .count();

    // 计算期望的帧率
    int expected_fps = 40;
    int expected_frame_time = 1000 / expected_fps;

    // 如果处理时间小于期望的帧率，等待一段时间
    // if (elapsed_ms < expected_frame_time) {
    //   return;
    // }
    start_time = end_time;

    // TDT_INFO("Recorder process %d frames", (int)frames->size());

    frames->push_back(src);
total_frame=0;

    k++;
    // TDT_INFO("Write frame %d", k);
  }

  }
  if (start1 == 1) {
    std::thread th(&VideoRecoder::Recorder_, this, frames);
    th.detach();

    //           frames=new std::list<cv::Mat>;
  }
  if (start1 > 0) {
    start1 = start1 - 1;
  }
  // TDT_INFO("Video Record Status%d", start1);
}

void VideoRecoder::Recorder_(std::list<cv::Mat> *imgs) {
  /// 读取executable所在绝对路径
  std::string exec_str = get_cur_executable_path_();

  /// 用于获取磁盘剩余空间
  struct statfs diskInfo;
  statfs(exec_str.c_str(), &diskInfo);

  unsigned long long blocksize = diskInfo.f_bsize;  // 每个block里包含的字节数
  unsigned long long totalsize =
      blocksize * diskInfo.f_blocks;  // 总的字节数，f_blocks为block的数目

  unsigned long long availableDisk =
      diskInfo.f_bavail * blocksize;  // 可用空间大小

  long int trueavailableDisk = availableDisk >> 30;
  int t = 0;
  while (!imgs[0].empty()) {
    auto d_start = std::chrono::high_resolution_clock::now();

    if (trueavailableDisk < 10) {
      //      TDT_ERROR("磁盘空间不足10G");
      if (imgs[0].size() == 1) {
        continue;
      } else {
        auto i = imgs[0];

        //        t++;
        // printf("save %d\n", t);
        //        video_ << i.front();

        imgs[0].pop_front();
      }
      // 释放内存并退出
      //      imgs->clear();
      //      free(imgs);
      //      return;

    } else {
      if (imgs[0].size() == 1) {
        continue;
      } else {
        auto i = imgs[0];

        t++;
        video_ <<i.front();
        imgs[0].pop_front();
      }
    }
    auto d_end = std::chrono::high_resolution_clock::now();
    auto time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(
        d_end - d_start);
  }
  imgs->clear();
  free(imgs);
  std::cout << "free imgs" << std::endl;
}
}  // namespace robots_perception