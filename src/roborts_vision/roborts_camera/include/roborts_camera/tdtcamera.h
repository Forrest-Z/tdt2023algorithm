#ifndef LIB_TDTCAMERA_TDTCAMERA_H
#define LIB_TDTCAMERA_TDTCAMERA_H

#include <sys/time.h>

#include <condition_variable>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
// 海康头文件
#include "roborts_camera/hikvisioncam.h"

namespace tdtcamera {

class Header {
 public:
  Header(uint64_t seq);
  Header() = default;

 public:
  uint64_t stamp_;
  uint64_t seq_ = 0;
  std::string frame_id_;
};

class TImage {
 public:
  TImage(cv::Mat src, uint64_t seq) : cvimage_(std::move(src)), header_(seq){};
  TImage(){};

 public:
  Header header_;
  cv::Mat cvimage_;
};

class Camera {
 public:
  /**
   * 相机类型
   */
  enum TDT_CAMERA_CAMTYPE {
    TDT_CAMERA_CAMTYPE_UVCCAM = 0,
    TDT_CAMERA_CAMTYPE_HIKVISION = 1,
    TDT_CAMERA_CAMTYPE_VIDEODEBUG = 2,
  };
  /**
   * 相机格式
   */
  enum TDT_CAMERA_FORMAT {
    TDT_CAMERA_FORMAT_PIXEL = 0,
    TDT_CAMERA_FORMAT_WIDTH = 1,
    TDT_CAMERA_FORMAT_HEIGHT = 2,
    TDT_CAMERA_FORMAT_FPS = 3,
  };
  /**
   * 相机设置
   */
  enum TDT_CAMERA_SETTING {
    TDT_CAMERA_SETTING_EXPOSURE = 0,
    TDT_CAMERA_SETTING_GAIN = 1,
    TDT_CAMERA_SETTING_BRIGHTNESS = 2,
    TDT_CAMERA_SETTING_BALANCE_VAL = 3,
    TDT_CAMERA_SETTING_BALANCE_RED = 4,
    TDT_CAMERA_SETTING_BALANCE_GREEN = 5,
    TDT_CAMERA_SETTING_BALANCE_BLUE = 6,
    TDT_CAMERA_SETTING_HUE = 7,
    TDT_CAMERA_SETTING_SATURATION = 8,
    TDT_CAMERA_SETTING_CONTRAST = 9,
    TDT_CAMERA_SETTING_GAMMA = 10,
    TDT_CAMERA_SETTING_SHARPNESS = 11,
    TDT_CAMERA_SETTING_BLACK_LEVEL = 12,
    TDT_CAMERA_SETTING_TRIGER_IMAGE = 13,
    TDT_CAMERA_SETTING_REVERSE_X = 14,
    TDT_CAMERA_SETTING_REVERSE_Y = 15
  };

  /**
   * 标定变量设置
   * */
  cv::Mat frame;
  cv::Mat gray;
  cv::Mat copyImage;
  cv::Mat copyImage1;
  cv::Mat copyImage2;
  cv::Mat copyImage3;
  cv::Size PatternSize = cv::Size(8, 11);
  cv::Size image_size;
  std::vector<cv::Point2f> corner_points;
  std::vector<std::vector<cv::Point2f>> corner_allimages;
  int maxNum = 15;
  int times = 0;
  int times1 = 1;
  int key = 2;
  std::vector<int> times2;
  cv::Mat camera_matrix =
      cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));  // 内外参矩阵，H——单应性矩阵
  cv::Mat rotation_Matrix =
      cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));  // 旋转矩阵
  cv::Mat dist_coeffs =
      cv::Mat(1, 5, CV_32FC1,
              cv::Scalar::all(0));  // 摄像机的5个畸变系数：k1,k2,p1,p2,k3
  std::vector<cv::Mat> tvecsMat;  // 每幅图像的平移向量，t
  std::vector<cv::Mat> rvecsMat;  // 每幅图像的旋转向量（罗德里格斯旋转向量）
  std::vector<std::vector<cv::Point3f>> objectPoints;
  std::vector<double> cal_distance;
  std::vector<double> project_error;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  explicit Camera(std::string config_path);
  explicit Camera() = default;
  ~Camera();
  virtual int CameraCalibrate(const cv::Mat &frame);
  virtual bool solvePnP_cal(const cv::Mat &frame);
  virtual double PnP_points(const std::vector<cv::Point2f> &corner_points);
  virtual void calibrate();
  virtual cv::Mat GetMatrix() { return identity_.camera_matrix; }
  virtual cv::Mat GetDistCoeffs() { return identity_.dist_coeffs; };
  virtual bool SetIntVal(TDT_CAMERA_FORMAT tdt_camera_format,
                         unsigned int val){};
  virtual bool find_corner(const cv::Mat &src);
  virtual bool SetIntVal(TDT_CAMERA_SETTING tdt_camera_setting,
                         unsigned int val){};
  virtual bool SetFloatVal(TDT_CAMERA_FORMAT tdt_camera_format, float val){};
  virtual bool SetFloatVal(TDT_CAMERA_SETTING tdt_camera_setting, float val){};
  virtual bool CloseCamera() = 0;
  // TODO 增加相机参数读取
  //    virtual int GetIntVal(TDT_CA> identity_.nameMERA_FORMAT
  //    tdt_camera_format)=0; virtual int GetIntVal(TDT_CAMERA_SETTING
  //    tdt_camera_setting)=0; virtual float GetFloatVal(TDT_CAMERA_FORMAT
  //    tdt_camera_format)=0; virtual float GetFloatVal(TDT_CAMERA_SETTING
  //    tdt_camera_setting)=0;

  virtual bool GetImage(TImage &timage) = 0;
  virtual bool RestartCamera() = 0;
  virtual double GetImageTime() = 0;
  virtual bool ReatartCamRunningProgram() {
    std::cout << "Camera ReatartCamRunningProgram" << std::endl;
    return true;
  }
  virtual bool SetMode(int mode_) { this->mode = mode_; }

 protected:
  static uint8_t cam_online_num_;
  static uint8_t cam_total_num_;
  uint64_t seq_ = 0;
  struct CameraIdentity {
    uint8_t param_index;
    uint8_t camera_id;
    std::string name;
    TDT_CAMERA_CAMTYPE type;
    std::string guid;
    uint8_t dev_index;
    std::string dev_path;
    std::string video_path;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
  } identity_;

  struct CameraFormat {
    uint8_t pixel_format;
    uint32_t width;
    uint32_t height;
    float fps;
  } format_;

  struct CameraSetting {
    float exposure;
    float gain;
    uint32_t brightness;
    uint32_t balance_val;
    uint32_t balance_red;
    uint32_t balance_green;
    uint32_t balance_blue;
    uint32_t hue;
    uint32_t saturation;
    uint32_t contrast;
    float gamma;
    uint32_t sharpness;
    uint32_t black_level;
    bool trigger;
    bool get_timeStamp;
    bool reverse_x;
    bool reverse_y;
  } setting_;

  bool LoadParam();
  int mode;

 private:
  std::vector<std::vector<cv::Point2f>> camera_calibrate_points_sequence_;
  std::string path;
  bool LoadParam(std::string config_path);
};

class HikvisionCam : public Camera {
 public:
  HikvisionCam(std::string config_path);
  ~HikvisionCam() = default;
  bool Set(TDT_CAMERA_FORMAT tdt_camera_format, uint32_t val);
  bool Set(TDT_CAMERA_SETTING tdt_camera_setting, uint32_t val);
  bool Set(TDT_CAMERA_FORMAT tdt_camera_format, float val);
  bool Set(TDT_CAMERA_SETTING tdt_camera_setting, float val);
  //    int GetIntVal(TDT_CAMERA_FORMAT tdt_camera_format);
  //    int GetIntVal(TDT_CAMERA_SETTING tdt_camera_setting);
  //    float GetFloatVal(TDT_CAMERA_FORMAT tdt_camera_format);
  //    float GetFloatVal(TDT_CAMERA_SETTING tdt_camera_setting);

  void TakeFrame(sensor_msgs::msg::Image &image_msg);
  bool GetImage(TImage &timage);
  bool RestartCamera();

  /**************************************************************************
   * @name            GetImageTime
   * @brief           获取当前帧的拍摄时间
   * @author          严俊涵
   ***************************************************************************/
  double GetImageTime() { return srcTime_; };

  /*
   * @brief:为了可以在VideoDebug界面中可以设置相机的参数设置
   * @auther:张程林(1434555239@qq.com)
   *
   */
  bool ReatartCamRunningProgram() {
    std::cout << "-----------------kikvi ReatartCamRunningProgram" << std::endl;
    LoadParam();
    std::cout << "kikvi " << std::endl;
    std::unique_lock<std::mutex> locker(stop_mtx_);
    Stop = true;
    bool ret = true;
    TDT_INFO("===RESTART HIKVISION CAMERA  Resetting Parameter===");
    ret &= hikvisioncam_.CloseGrabbing();
    std::cout << "CloseGrabbing " << ret << std::endl;
    //            ret &= hikvisioncam_.CloseCamera();
    //            std::cout<<"CloseCamera "<<ret<<std::endl;
    std::cout << "pass1" << std::endl;
    Set(TDT_CAMERA_SETTING_EXPOSURE, setting_.exposure);
    Set(TDT_CAMERA_SETTING_GAIN, setting_.gain);
    Set(TDT_CAMERA_SETTING_BRIGHTNESS, setting_.brightness);
    Set(TDT_CAMERA_SETTING_BALANCE_RED, setting_.balance_red);
    Set(TDT_CAMERA_SETTING_BALANCE_GREEN, setting_.balance_green);
    Set(TDT_CAMERA_SETTING_BALANCE_BLUE, setting_.balance_blue);
    Set(TDT_CAMERA_SETTING_HUE, setting_.hue);
    Set(TDT_CAMERA_SETTING_SATURATION, setting_.saturation);
    Set(TDT_CAMERA_SETTING_GAMMA, setting_.gamma);
    Set(TDT_CAMERA_SETTING_SHARPNESS, setting_.sharpness);
    Set(TDT_CAMERA_SETTING_TRIGER_IMAGE, (float)1.0f);

    Set(TDT_CAMERA_SETTING_REVERSE_X,(float) setting_.reverse_x);
    Set(TDT_CAMERA_SETTING_REVERSE_Y, (float)setting_.reverse_y);

    //            std::cout << setting_.sharpness << std::endl;
    //            std::cout << 0xFFFFFFFF << std::endl;

    Set(TDT_CAMERA_SETTING_BLACK_LEVEL, setting_.black_level);
    ret &= hikvisioncam_.StartGrabbing();
    Stop = false;
    // std::thread t(&HikvisionCam::TakeFrame, this);
    // t.detach();
    if (ret) {
      return true;
    } else {
      return false;
    }
  }
  /*
   *@brief:这个函数是关闭相机的操作，调用的地方是：程序无论是意外结束还是ctrl+c都要关闭相机否则会对相机造成损坏
   */
  virtual bool CloseCamera() {
    // 先关闭取帧，在断开相机，顺序不能变
    bool ret = true;
    ret &= hikvisioncam_.CloseGrabbing();
    ret &= hikvisioncam_.CloseCamera();
    return ret;
  }
  virtual bool SetMode(int mode_) {
    if (mode == mode_) return false;
    mode = mode_;
    std::cout << "changed mode =--" << mode_ << std::endl;
    //            std::unique_lock<std::mutex>locker(stop_mtx_);
    //            hikvisioncam_.CloseGrabbing();
    switch (mode_) {
      case 0:  // 装甲板
        LoadParam::ReadParam("camera","gain", setting_.gain);
        Set(TDT_CAMERA_SETTING_GAIN, setting_.gain);
        LoadParam::ReadParam("camera","armor_exposure", setting_.exposure);
        Set(TDT_CAMERA_SETTING_EXPOSURE, setting_.exposure);
        break;
      case 1:  // 开符
        LoadParam::ReadParam("camera","buff_exposure", setting_.exposure);
        Set(TDT_CAMERA_SETTING_EXPOSURE, setting_.exposure);
        break;
      case 2:  // 反符
        LoadParam::ReadParam("camera","buff_exposure", setting_.exposure);
        Set(TDT_CAMERA_SETTING_EXPOSURE, setting_.exposure);
        break;
    }
    //            hikvisioncam_.StartGrabbing();
    hikvisioncam_.sendTrigger();
    //            locker.unlock();
    std::cout << "changed mode endl" << std::endl;
    return true;
  }
  
  inline std::string get_guid(){
    return hikvisioncam_.get_guid();
  }
 private:
  // Stop用来判断是否让相机停下来
  bool Stop;
  std::mutex stop_mtx_;
  std::mutex img_mtx_;
  std::condition_variable condVar_;
  cv::Mat takeImg_, swapImg_, srcImg_;
  double takeTime_, swapTime_, srcTime_;
  tdtbasecam::HikvisionBasicCam hikvisioncam_;
};

class VideoDebug : public Camera {
 public:
  VideoDebug(std::string config_path);
  VideoDebug();
  ~VideoDebug() = default;

 public:
  void TakeFrame();
  bool GetImage(TImage &timage);
  bool Set(TDT_CAMERA_FORMAT tdt_camera_format, uint32_t val){};
  bool Set(TDT_CAMERA_SETTING tdt_camera_setting, uint32_t val){};
  bool Set(TDT_CAMERA_FORMAT tdt_camera_format, float val){};
  bool Set(TDT_CAMERA_SETTING tdt_camera_setting, float val){};
  bool RestartCamera(){};
  virtual bool CloseCamera() { return true; }
  /**************************************************************************
   * @name            GetImageTime
   * @brief           用于对应海康相机类的获取拍摄时间，始终返回-1
   * @author          严俊涵
   ***************************************************************************/
  double GetImageTime() { return -1; };

 private:
  std::mutex img_mtx_;
  std::condition_variable condVar_;
  cv::Mat takeImg_, swapImg_, srcImg_;
  cv::VideoCapture capture_;
};

}  // namespace tdtcamera

#endif
