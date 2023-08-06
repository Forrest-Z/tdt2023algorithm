/*
 * @Name: file name
 * @Description:
 * @Version: 1.0.0.1
 * @Author: your name
 * @Date: 2019-10-14 19:42:40
 * @LastEditors: your name
 * @LastEditTime: 2019-11-22 20:37:14
 */

#include <chrono>

#include "roborts_utils/base_param.h"
#include "roborts_utils/config.h"
#include "tdtcamera.h"

// #include "parameter/load_param.h"
// #include "tdt_config/config.h"

namespace tdtcamera {
uint8_t Camera::cam_online_num_ = 0;
uint8_t Camera::cam_total_num_ = 0;
Header::Header(uint64_t seq) : seq_(seq) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  stamp_ = tv.tv_sec * 1E6 + tv.tv_usec;
}

Camera::Camera(std::string config_path) {
  path = config_path;
  LoadParam(config_path);
  identity_.camera_id = cam_total_num_;
  cam_total_num_++;
  cam_online_num_++;
}
Camera::~Camera() { cam_online_num_--; }

bool Camera::find_corner(const cv::Mat &src) {
  copyImage2 = src.clone();
  cvtColor(src, gray, cv::COLOR_RGB2GRAY);
  if (findChessboardCorners(gray, PatternSize, corner_points) == 0) {
    std::cout << "无法找到角点\n";  // 找不到角点
    putText(src, std::string("can not find chessboard corners!"),
            cv::Point(20, 200), cv::FONT_HERSHEY_COMPLEX, 2,
            cv::Scalar(0, 0, 255), 2, 8);
    imshow("camera calibration", src);
    cv::waitKey(1000);
    return false;
  } else {
    if (times2.size() == 0) {
      find4QuadCornerSubpix(gray, corner_points, cv::Size(5, 5));
      putText(src,
              std::string("successfully found " + std::to_string(times + 1) +
                          " board corners!"),
              cv::Point(10, 200), cv::FONT_HERSHEY_COMPLEX, 2,
              cv::Scalar(0, 255, 0), 2, 8);
      imshow("camera calibration", src);
      int input1 = cv::waitKey(0);
      if (input1 == 115) {
        // 保存标定图片，不需要可注释
        //                    std::string path6 =
        //                    "/home/liujia/calibration_picture/9/" +
        //                    std::to_string(times+1) + ".jpg"; imwrite(path6,
        //                    copyImage2);
        // 将角点存入容器
        corner_allimages.push_back(corner_points);
        times++;
        std::cout << "第" << times << "张图片已提取出角点" << std::endl;
        // 显示角点位置
        drawChessboardCorners(src, PatternSize, corner_points, true);
        for (int i = 0; i < corner_points.size(); i++)
          putText(src, std::to_string(i), corner_points[i], 2, 1,
                  cv::Scalar(255, 255, 255));
        imshow("camera calibration", src);
        cv::waitKey(1000);
      } else {
        std::cout << "图片不符合预期，不保存角点" << std::endl;
      }
    } else {
      find4QuadCornerSubpix(gray, corner_points, cv::Size(5, 5));
      putText(src,
              std::string("successfully found " + std::to_string(times2[0]) +
                          " new corners!"),
              cv::Point(10, 200), cv::FONT_HERSHEY_COMPLEX, 2,
              cv::Scalar(0, 255, 0), 2, 8);
      imshow("camera calibration", src);
      int input1 = cv::waitKey(0);
      if (input1 == 115) {
        // 新图片替换原图片，不需要可注释
        //                        std::string path6 =
        //                        "/home/liujia/calibration_picture/9/" +
        //                        std::to_string(times2[0]) + ".jpg";
        //                        imwrite(path6,  copyImage2);
        // 新角点替换原角点
        corner_allimages[times2[0] - 1] = corner_points;
        times++;
        std::cout << "第" << times2[0] << "张图片已重新提取出角点" << std::endl;
        times2.erase(times2.begin());
        // 显示新角点位置
        drawChessboardCorners(src, PatternSize, corner_points, true);
        for (int i = 0; i < corner_points.size(); i++)
          putText(src, std::to_string(i), corner_points[i], 2, 1,
                  cv::Scalar(255, 255, 255));
        imshow("camera calibration", src);
        cv::waitKey(1000);
      } else {
        std::cout << "新图片不符合预期，不保存角点" << std::endl;
      }
    }
  }
  return true;
}

void Camera::calibrate() {
  // 角点提取完成
  project_error.clear();
  cal_distance.clear();
  objectPoints.clear();             // 别忘了清空
  for (int k = 0; k < maxNum; k++)  // 遍历每一张图片
  {
    // 开始标定
    std::vector<cv::Point3f> tempCornerPoints;  // 每一幅图片对应的角点数组
    // 遍历所有的角点
    for (int i = 0; i < PatternSize.height; i++) {
      for (int j = 0; j < PatternSize.width; j++) {
        cv::Point3f singleRealPoint;  // 一个角点的坐标
        singleRealPoint.x = i * 10;
        singleRealPoint.y = j * 10;
        singleRealPoint.z = 0;  // 假设z=0
        tempCornerPoints.push_back(singleRealPoint);
      }
    }
    objectPoints.push_back(tempCornerPoints);
  }
  calibrateCamera(objectPoints, corner_allimages, image_size, camera_matrix,
                  dist_coeffs, rvecsMat, tvecsMat, 0);
  std::cout << "对标定时标定板进行解算……\n";
  for (int i = 0; i < maxNum; i++) {
    double distance = PnP_points(corner_allimages[i]);
    cal_distance.push_back(distance);
  }

  for (int i = 0; i < maxNum; i++) {
    tdttoolkit::Rodrigues(rvecsMat[i],
                          rotation_Matrix);  // 将旋转向量转换为相对应的旋转矩阵
  }

  // 开始评定标定结果
  int corner_points_counts;
  corner_points_counts = PatternSize.width * PatternSize.height;

  std::cout << "每幅图像的误差……" << std::endl;
  double error = 0;        // 单张图像的误差
  double total_error = 0;  // 所有图像的平均误差
  for (int i = 0; i < maxNum; i++) {
    std::vector<cv::Point2f>
        image_points_calculated;  // 存放新计算出的投影点的坐标
    std::vector<cv::Point3f> tempPointSet = objectPoints[i];
    projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], camera_matrix,
                  dist_coeffs, image_points_calculated);
    // 计算新的投影点与旧的投影点之间的误差
    std::vector<cv::Point2f> image_points_old = corner_allimages[i];
    // 将两组数据换成Mat格式
    cv::Mat image_points_calculated_mat =
        cv::Mat(1, image_points_calculated.size(), CV_32FC2);
    cv::Mat image_points_old_mat =
        cv::Mat(1, image_points_old.size(), CV_32FC2);
    for (int j = 0; j < tempPointSet.size(); j++) {  // 用于计算error
      image_points_calculated_mat.at<cv::Vec2f>(0, j) =
          cv::Vec2f(image_points_calculated[j].x, image_points_calculated[j].y);
      image_points_old_mat.at<cv::Vec2f>(0, j) =
          cv::Vec2f(image_points_old[j].x, image_points_old[j].y);
    }
    error = norm(image_points_calculated_mat, image_points_old_mat,
                 cv::NORM_L2);  // 计算error
    error /= corner_points_counts;
    total_error += error;
    project_error.push_back(error);
    //            std::cout << "第" << i + 1 << "幅图像的重投影误差：" << error
    //            << "像素" << std::endl;
  }
  std::cout << "序号" << ' ' << '|' << "标定板到相机解算距离" << ' ' << '|'
            << "重投影误差" << std::endl;
  for (int i = 1; i < maxNum + 1; ++i) {
    std::cout << i << "\t\t" << cal_distance[i - 1] << "\t\t\t"
              << project_error[i - 1] << std::endl;
  }
  std::cout << "总体平均重投影误差：" << total_error / maxNum << "像素"
            << std::endl;
  LoadParam::WriteParam("camera", "camera_matrix", camera_matrix);
  LoadParam::WriteParam("camera", "dist_coeffs", dist_coeffs);
  LoadParam::OutPutParam("camera");
  std::cout << "评价完成,第" + std::to_string(times1) + "组相机参数已保存"
            << std::endl;
  times1++;
  std::cout << "times2.size() = " << times2.size() << std::endl;
  std::cout << "手动选择不需要的标定图片重新进行标定(输入1-"
               "15分别表示重新拍摄第相应张图片，要退出最后一次输入0即可，若只输"
               "入0表示所有图片已符合要求):\n";
  int input2;
  while (std::cin >> input2) {
    if (input2 == 0) {
      std::cout << "已选取完不需要图片，回到获取图片程序或退出\n";
      cv::waitKey(1000);
      break;
    } else
      times2.push_back(input2);
  }
}

int Camera::CameraCalibrate(const cv::Mat &frame) {
  if (frame.empty()) return 0;
  if (times == maxNum) {
    cv::destroyAllWindows();
    calibrate();
    if (times2.size() == 0) {
      return 1;
    } else {
      times = maxNum - times2.size();
      cv::namedWindow("camera calibration", cv::WINDOW_NORMAL);
      return 0;
    }
  }
  copyImage = frame.clone();
  copyImage1 = frame.clone();
  image_size.width = frame.cols;
  image_size.height = frame.rows;
  putText(frame, std::string("press F to calibrate, ESC to exit"),
          cv::Point(20, 100), cv::FONT_HERSHEY_COMPLEX, 2,
          cv::Scalar(255, 255, 255), 2, 8);
  imshow("camera calibration", frame);
  int input = cv::waitKey(1);
  if (input != -1) std::cout << "input=" << input << std::endl;
  if (input == 102) {
    putText(copyImage, std::string("detecting..."), cv::Point(100, 200),
            cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(255, 255, 255), 2, 8);
    imshow("camera calibration", copyImage);
    cv::waitKey(1);
    find_corner(copyImage1);
  }
  if (input == 27) {
    return 1;
  }
  return 0;
}
bool Camera::solvePnP_cal(const cv::Mat &frame) {
  copyImage3 = frame.clone();
  if (key == 2)
    putText(frame, std::string("press i to solvePnP, ESC to exit"),
            cv::Point(20, 100), cv::FONT_HERSHEY_COMPLEX, 2,
            cv::Scalar(255, 255, 255), 2, 8);
  cv::imshow("camera calibration", frame);
  int input3 = cv::waitKey(1);
  if (input3 != -1) {
    std::cout << "input3=" << input3 << std::endl;
  }
  if (input3 == 27) {
    key = 0;
  }
  if (input3 == 105) {
    key = 1;
  }
  if (key == 1) {
    cvtColor(copyImage3, gray, cv::COLOR_RGB2GRAY);
    if (findChessboardCorners(gray, PatternSize, corner_points) == 0) {
      std::cout << "无法找到角点\n";  // 找不到角点
      putText(copyImage3, std::string("can not find chessboard corners!"),
              cv::Point(20, 100), cv::FONT_HERSHEY_COMPLEX, 2,
              cv::Scalar(0, 0, 255), 2, 8);
      imshow("camera calibration", copyImage3);
      int input4 = cv::waitKey(1000);
      if (input4 == 27) {
        std::cout << "input4=" << input4 << std::endl;
        key = 0;
      }
      if (input4 == 111) {
        std::cout << "input4=" << input4 << std::endl;
        key = 2;
      }
    } else {
      find4QuadCornerSubpix(gray, corner_points, cv::Size(5, 5));
      std::cout << "成功找到角点\n";  // 找不到角点
      double distance1 = PnP_points(corner_points);
      putText(copyImage3, std::string("distance:" + std::to_string(distance1)),
              cv::Point(100, 200), cv::FONT_HERSHEY_COMPLEX, 2,
              cv::Scalar(0, 255, 0), 2, 8);
      imshow("camera calibration", copyImage3);
      int input4 = cv::waitKey(1000);
      if (input4 == 27) {
        std::cout << "input4=" << input4 << std::endl;
        key = 0;
      }
      if (input4 == 111) {
        std::cout << "input4=" << input4 << std::endl;
        key = 2;
      }
    }
  }
  if (key == 0) {
    return false;
  }
  return true;
}
double Camera::PnP_points(const std::vector<cv::Point2f> &corner_points) {
  std::vector<cv::Point2f> pnp_points;
  pnp_points.push_back(corner_points[2]);   //(0,2)
  pnp_points.push_back(corner_points[4]);   //(0,4)
  pnp_points.push_back(corner_points[6]);   //(0,6)
  pnp_points.push_back(corner_points[82]);  //(10,2)
  pnp_points.push_back(corner_points[84]);  //(10,4)
  pnp_points.push_back(corner_points[86]);  //(10,6)
  std::vector<cv::Point3f> objpoints;
  objpoints.push_back(cv::Point3f(0, 5, 0));
  objpoints.push_back(cv::Point3f(0, 10, 0));
  objpoints.push_back(cv::Point3f(0, 15, 0));
  objpoints.push_back(cv::Point3f(25, 5, 0));
  objpoints.push_back(cv::Point3f(25, 10, 0));
  objpoints.push_back(cv::Point3f(25, 15, 0));
  cv::Mat rvec = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));
  cv::Mat tvec = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));
  if (key == 2)
    cv::solvePnP(objpoints, pnp_points, camera_matrix, dist_coeffs, rvec, tvec);
  if (key == 1) {
    LoadParam::ReadParam("camera", "camera_matrix",
                         camera_matrix_);  // 内参矩阵
    LoadParam::ReadParam("camera", "dist_coeffs", dist_coeffs_);  // 畸变系数
    cv::solvePnP(objpoints, pnp_points, camera_matrix_, dist_coeffs_, rvec,
                 tvec);
  }
  double dist = norm(tvec, cv::NORM_L2);
  return dist;
}
bool Camera::LoadParam(std::string config_path) {
  mode = 0;
  LoadParam::InitParam("camera", config_path);
  identity_.type = TDT_CAMERA_CAMTYPE_HIKVISION;
  identity_.dev_index = 0;
  LoadParam::ReadParam("camera", "name", identity_.name);
  int camera_type = 0;
  LoadParam::ReadParam("camera", "type", camera_type);
  identity_.type = (TDT_CAMERA_CAMTYPE)(camera_type);
  LoadParam::ReadParam("camera", "guid", identity_.guid);
  int value = 0;
  LoadParam::ReadParam("camera", "dev_index", value);
  identity_.dev_index = value;
  LoadParam::ReadParam("camera", "dev_path", identity_.dev_path);
  LoadParam::ReadParam("camera", "VideoPath", identity_.video_path);
  if (!tdtconfig::CALIBRATE) {
    LoadParam::ReadParam("camera", "camera_matrix", identity_.camera_matrix);
    LoadParam::ReadParam("camera", "dist_coeffs", identity_.dist_coeffs);
  }
  LoadParam::ReadParam("camera", "width", value);
  format_.width = value;
  LoadParam::ReadParam("camera", "height", value);
  format_.height = value;
  LoadParam::ReadParam("camera", "fps", format_.fps);
  LoadParam::ReadParam("camera", "pixel_format", value);
  format_.pixel_format = value;
  LoadParam::ReadParam("camera", "armor_exposure", setting_.exposure);
  LoadParam::ReadParam("camera", "gain", setting_.gain);
  LoadParam::ReadParam("camera", "balance_val", value);
  setting_.balance_val = value;
  LoadParam::ReadParam("camera", "balance_red", value);
  setting_.balance_red = value;
  LoadParam::ReadParam("camera", "balance_green", value);
  setting_.balance_green = value;
  LoadParam::ReadParam("camera", "balance_blue", value);
  setting_.balance_blue = value;
  LoadParam::ReadParam("camera", "brightness", value);
  setting_.brightness = value;
  LoadParam::ReadParam("camera", "saturation", value);
  setting_.saturation = value;
  LoadParam::ReadParam("camera", "contrast", value);
  setting_.contrast = value;
  LoadParam::ReadParam("camera", "gamma", setting_.gamma);
  LoadParam::ReadParam("camera", "sharpness", value);
  setting_.sharpness = value;
  LoadParam::ReadParam("camera", "black_level", value);
  setting_.black_level = value;
  LoadParam::ReadParam("camera", "hue", value);
  setting_.hue = value;
  LoadParam::ReadParam("camera", "trigger", value);
  setting_.trigger = value;
  LoadParam::ReadParam("camera", "get_timeStamp", value);
  setting_.get_timeStamp = value;

  LoadParam::ReadParam("camera", "reverse_x", value);
  setting_.reverse_x = value;
  LoadParam::ReadParam("camera", "reverse_y", value);
  setting_.reverse_y = value;
  return true;
}
// 用于在debug中动态的设置相机参数
bool Camera::LoadParam() { return true; }

HikvisionCam::HikvisionCam(std::string config_path) : Camera(config_path) {
  Stop = false;

  std::cout << "pass1" << std::endl;
  if (identity_.type != TDT_CAMERA_CAMTYPE_HIKVISION) {
    TDT_FATAL("Failed to open hikvision camera. (incorrect type %d)",
              identity_.type);
  }
  if (identity_.guid != "") {
    if (!hikvisioncam_.InitHandle(identity_.guid)) {
      TDT_FATAL("Failed to open hikvision camera. (incorrect guid %s)",
                identity_.guid.c_str());
    }
  } else {
    if (!hikvisioncam_.InitHandle(identity_.dev_index)) {
      TDT_FATAL("Failed to open hikvision camera. (incorrect dev_index %d)",
                identity_.dev_index);
    }
    identity_.guid = hikvisioncam_.get_guid();
  }
  if (!hikvisioncam_.OpenCamera()) exit(-1);  // 相机存在但是相机错误，就exit
  if (format_.pixel_format == 0) {
    hikvisioncam_.SetPixelformat(PixelType_Gvsp_RGB8_Packed);

  } else if (format_.pixel_format == 1) {
    hikvisioncam_.SetPixelformat(PixelType_Gvsp_BayerRG8);
  }
  hikvisioncam_.SetResolution(format_.width, format_.height);
  hikvisioncam_.SetFps(format_.fps);
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
  Set(TDT_CAMERA_SETTING_TRIGER_IMAGE, (float)setting_.trigger);
  //        std::cout << setting_.sharpness << std::endl;
  //        std::cout << 0xFFFFFFFF << std::endl;
  Set(TDT_CAMERA_SETTING_BLACK_LEVEL, setting_.black_level);
  Set(TDT_CAMERA_SETTING_REVERSE_X, (float)setting_.reverse_x);
  Set(TDT_CAMERA_SETTING_REVERSE_Y, (float)setting_.reverse_y);
  hikvisioncam_.StartGrabbing();
  // std::thread t(&HikvisionCam::TakeFrame, this);
  // t.detach();
}

bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_FORMAT tdt_camera_format,
                       unsigned int val) {
  bool ret = true;
  //        hikvisioncam_.CloseGrabbing();
  switch (tdt_camera_format) {
    case TDT_CAMERA_FORMAT_PIXEL:
      if (val == 0) {
        ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_RGB8_Packed);
      } else if (val == 1) {
        ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_BayerRG8);
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera format. (incorrect pixel_format "
            "%d)",
            val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      if (ret) {
        format_.pixel_format = val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (pixel_format %d)",
                  val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    case TDT_CAMERA_FORMAT_WIDTH:
      ret &= hikvisioncam_.SetResolution(val, format_.height);
      if (ret) {
        format_.width = val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (width %d)", val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    case TDT_CAMERA_FORMAT_HEIGHT:
      ret &= hikvisioncam_.SetResolution(format_.width, val);
      if (ret) {
        format_.height = val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (height %d)", val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    case TDT_CAMERA_FORMAT_FPS:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetFpsDisable(true);
      } else {
        ret &= hikvisioncam_.SetFps(val);
      }
      if (ret) {
        format_.fps = val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (fps %d)", val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    default:
      TDT_ERROR(
          "Failed to set hikvision camera format. (TDT_CAMERA_SETTING : %d)",
          tdt_camera_format);
      return false;
      break;
  }
}

bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_FORMAT tdt_camera_format,
                       float val) {
  bool ret = true;
  //        hikvisioncam_.CloseGrabbing();
  switch (tdt_camera_format) {
    case TDT_CAMERA_FORMAT_PIXEL:
      if ((unsigned int)val == 0) {
        ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_RGB8_Packed);
      } else if ((unsigned int)val == 1) {
        ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_BayerRG8);
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera format. (incorrect pixel_format "
            "%d)",
            (unsigned int)val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      if (ret) {
        format_.pixel_format = (unsigned int)val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (pixel_format %d)",
                  (unsigned int)val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    case TDT_CAMERA_FORMAT_WIDTH:
      ret &= hikvisioncam_.SetResolution((unsigned int)val, format_.height);
      if (ret) {
        format_.width = (unsigned int)val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (width %d)",
                  (unsigned int)val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    case TDT_CAMERA_FORMAT_HEIGHT:
      ret &= hikvisioncam_.SetResolution(format_.width, (unsigned int)val);
      if (ret) {
        format_.height = (unsigned int)val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (height %d)",
                  (unsigned int)val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    case TDT_CAMERA_FORMAT_FPS:
      if (val < 0) {
        ret &= hikvisioncam_.SetFpsDisable(true);
      } else {
        ret &= hikvisioncam_.SetFps(val);
      }
      if (ret) {
        format_.fps = val;
        //                hikvisioncam_.StartGrabbing();
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (fps %d)", val);
        //                hikvisioncam_.StartGrabbing();
        return false;
      }
      break;
    default:
      TDT_ERROR(
          "Failed to set hikvision camera format. (TDT_CAMERA_SETTING : %d)",
          tdt_camera_format);
      return false;
      break;
  }
}

bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_SETTING tdt_camera_setting,
                       unsigned int val) {
  bool ret = true;
  switch (tdt_camera_setting) {
    case TDT_CAMERA_SETTING_EXPOSURE:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetExposureAuto(true);
      } else {
        ret &= hikvisioncam_.SetExposureAuto(false);
        ret &= hikvisioncam_.SetExposure(val);
      }
      if (ret) {
        setting_.exposure = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (exposure : %d)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_GAIN:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetGainAuto(true);
      } else {
        ret &= hikvisioncam_.SetGainAuto(false);
        ret &= hikvisioncam_.SetGain(val);
      }
      if (ret) {
        setting_.gain = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (gain : %d)", val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BRIGHTNESS:
      if (val != 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetBrightness(val);
      }
      if (ret) {
        setting_.brightness = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (brightness : %d)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BALANCE_RED:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(true);
      } else if (setting_.balance_green != 0xFFFFFFFF &&
                 setting_.balance_blue != 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(false);
        ret &= hikvisioncam_.SetWhitebalance(val, setting_.balance_green,
                                             setting_.balance_blue);
      }
      if (ret) {
        setting_.balance_red = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (balance_red : %d)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BALANCE_GREEN:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(true);
      } else if (setting_.balance_red != 0xFFFFFFFF &&
                 setting_.balance_blue != 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(false);
        ret &= hikvisioncam_.SetWhitebalance(setting_.balance_red, val,
                                             setting_.balance_blue);
      }
      if (ret) {
        setting_.balance_green = val;
        return true;
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera setting. (balance_green : %d)",
            val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BALANCE_BLUE:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(true);
      } else if (setting_.balance_green != 0xFFFFFFFF &&
                 setting_.balance_red != 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(false);
        ret &= hikvisioncam_.SetWhitebalance(setting_.balance_red,
                                             setting_.balance_green, val);
      }
      if (ret) {
        setting_.balance_blue = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (balance_blue : %d)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_HUE:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetHueDisable(true);
      } else {
        ret &= hikvisioncam_.SetHueDisable(false);
        ret &= hikvisioncam_.SetHue(val);
      }
      if (ret) {
        setting_.hue = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (hue : %d)", val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_SATURATION:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetSaturationDisable(true);
      } else {
        ret &= hikvisioncam_.SetSaturationDisable(false);
        ret &= hikvisioncam_.SetSaturation(val);
      }
      if (ret) {
        setting_.saturation = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (saturation : %d)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_GAMMA:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetGammaDisable(true);
      } else {
        ret &= hikvisioncam_.SetGammaDisable(true);
        ret &= hikvisioncam_.SetGammaDisable(false);
        if (val < 1) {
          ret &= hikvisioncam_.SetGamma(val, MV_GAMMA_SELECTOR_USER);
        } else {
          ret &= hikvisioncam_.SetGamma();
        }
      }
      if (ret) {
        setting_.gamma = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (gamma : %d)", val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_SHARPNESS:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetSharpnessDisable(true);
      } else {
        ret &= hikvisioncam_.SetSharpnessDisable(false);
        ret &= hikvisioncam_.SetSharpness(val);
      }
      if (ret) {
        setting_.sharpness = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (sharpness : %d)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BLACK_LEVEL:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetBlacklevelDisable(true);
      } else {
        ret &= hikvisioncam_.SetBlacklevelDisable(false);
        ret &= hikvisioncam_.SetBlacklevel(val);
      }
      if (ret) {
        setting_.black_level = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (black_level : %d)",
                  val);
        return false;
      }
      break;

    case TDT_CAMERA_SETTING_REVERSE_X:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetReverse_X(false);

      } else {
        ret &= hikvisioncam_.SetReverse_X(val);
      }
      if (ret) {
        setting_.reverse_x = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (Reverse_X : %d)",
                  val);
        return false;
      }
      break;

    case TDT_CAMERA_SETTING_REVERSE_Y:
      if (val == 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetReverse_Y(false);

      } else {
        ret &= hikvisioncam_.SetReverse_Y(val);
      }
      if (ret) {
        setting_.reverse_y = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (Reverse_Y : %d)",
                  val);
        return false;
      }
      break;

    default:
      TDT_ERROR(
          "Failed to set hikvision camera setting. (TDT_CAMERA_SETTING : %d)",
          tdt_camera_setting);
      return false;
      break;
  }
}

bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_SETTING tdt_camera_setting,
                       float val) {
  bool ret = true;
  switch (tdt_camera_setting) {
    case TDT_CAMERA_SETTING_EXPOSURE:
      std::cout << "set exposure " << val << std::endl;
      if (val < 0) {
        ret &= hikvisioncam_.SetExposureAuto(true);
      } else {
        ret &= hikvisioncam_.SetExposureAuto(false);
        ret &= hikvisioncam_.SetExposure(val);
      }
      if (ret) {
        setting_.exposure = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (exposure : %f)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_GAIN:
      if (val < 0) {
        ret &= hikvisioncam_.SetGainAuto(true);
      } else {
        ret &= hikvisioncam_.SetGainAuto(false);
        ret &= hikvisioncam_.SetGain(val);
      }
      if (ret) {
        setting_.gain = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (gain : %f)", val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BRIGHTNESS:
      if (val < 0) {
        ret &= hikvisioncam_.SetBrightness((unsigned int)val);
      }
      if (ret) {
        setting_.brightness = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (brightness : %f)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BALANCE_RED:
      if (val < 0) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(true);
      } else if (setting_.balance_green != 0xFFFFFFFF ||
                 setting_.balance_blue != 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(false);
        ret &= hikvisioncam_.SetWhitebalance(
            (unsigned int)val, setting_.balance_green, setting_.balance_blue);
      }
      if (ret) {
        setting_.balance_red = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (balance_red : %f)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BALANCE_GREEN:
      if (val < 0) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(true);
      } else if (setting_.balance_red != 0xFFFFFFFF ||
                 setting_.balance_blue != 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(false);
        ret &= hikvisioncam_.SetWhitebalance(
            setting_.balance_red, (unsigned int)val, setting_.balance_blue);
      }
      if (ret) {
        setting_.balance_green = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera setting. (balance_green : %f)",
            val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BALANCE_BLUE:
      if (val < 0) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(true);
      } else if (setting_.balance_green != 0xFFFFFFFF ||
                 setting_.balance_red != 0xFFFFFFFF) {
        ret &= hikvisioncam_.SetWhitebalanceAuto(false);
        ret &= hikvisioncam_.SetWhitebalance(
            setting_.balance_red, setting_.balance_green, (unsigned int)val);
      }
      if (ret) {
        setting_.balance_blue = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (balance_blue : %f)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_HUE:
      if (val < 0) {
        ret &= hikvisioncam_.SetHueDisable(true);
      } else {
        ret &= hikvisioncam_.SetHueDisable(false);
        ret &= hikvisioncam_.SetHue((unsigned int)val);
      }
      if (ret) {
        setting_.hue = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (hue : %f)", val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_SATURATION:
      if (val < 0) {
        ret &= hikvisioncam_.SetSaturationDisable(true);
      } else {
        ret &= hikvisioncam_.SetSaturationDisable(false);
        ret &= hikvisioncam_.SetSaturation((unsigned int)val);
      }
      if (ret) {
        setting_.saturation = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (saturation : %f)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_GAMMA:
      if (val < 0) {
        ret &= hikvisioncam_.SetGammaDisable(true);
      } else {
        ret &= hikvisioncam_.SetGammaDisable(true);
        ret &= hikvisioncam_.SetGammaDisable(false);
        if (val < 1) {
          ret &= hikvisioncam_.SetGamma(val, MV_GAMMA_SELECTOR_USER);
        } else {
          ret &= hikvisioncam_.SetGamma();
        }
      }
      if (ret) {
        setting_.gamma = val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (gamma : %f)", val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_SHARPNESS:
      if (val < 0) {
        ret &= hikvisioncam_.SetSharpnessDisable(true);
      } else {
        ret &= hikvisioncam_.SetSharpnessDisable(false);
        ret &= hikvisioncam_.SetSharpness((unsigned int)val);
      }
      if (ret) {
        setting_.sharpness = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (sharpness : %f)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_BLACK_LEVEL:
      if (val < 0) {
        ret &= hikvisioncam_.SetBlacklevelDisable(true);
      } else {
        ret &= hikvisioncam_.SetBlacklevelDisable(false);
        ret &= hikvisioncam_.SetBlacklevel((unsigned int)val);
      }
      if (ret) {
        setting_.black_level = (unsigned int)val;
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (black_level : %f)",
                  val);
        return false;
      }
      break;
    case TDT_CAMERA_SETTING_TRIGER_IMAGE:
      if (val == 0.0f) {
        ret &= hikvisioncam_.SetTrigger(false);
      } else {
        ret &= hikvisioncam_.SetTrigger(true);
      }
      if(ret) {
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (triger_image : %f)",
                  val);
        return false;
      }
      break;

    case TDT_CAMERA_SETTING_REVERSE_X:
      if (val == 0.0f) {
        ret &= hikvisioncam_.SetReverse_X(false);
      } else {
        ret &= hikvisioncam_.SetReverse_X(val);
      }
      if(ret) {
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (reverse_x : %f)",
                  val);
        return false;
      }
      break;

    case TDT_CAMERA_SETTING_REVERSE_Y:
      if (val == 0.0f) {
        ret &= hikvisioncam_.SetReverse_Y(false);
      } else {
        ret &= hikvisioncam_.SetReverse_Y(val);
      }
      if(ret) {
        return true;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (reverse_y : %f)",
                  val);
        return false;
      }
      break;

    default:
      TDT_ERROR(
          "Failed to set hikvision camera setting. (TDT_CAMERA_SETTING : %d)",
          tdt_camera_setting);
      return false;
      break;
  }
}

void HikvisionCam::TakeFrame(sensor_msgs::msg::Image &image_msg) {
  // while (true) {
  // double last_tick = cv::getTickCount();
  // TDT_INFO("相机帧数FPS %lf", (cv::getTickFrequency()) / (cv::getTickCount()
  // - last_tick)); last_tick = cv::getTickCount(); std::unique_lock<std::mutex>
  // locker_(stop_mtx_); if (Stop) {
  //     std::cout << "stop" << std::endl;
  //     break;
  // }
  if (setting_.trigger) {
    if (!hikvisioncam_.GetMat_Triger(image_msg)) {
      TDT_ERROR("Failed to get image from hikvision cam");
    }
  }

  else {
    if (!hikvisioncam_.GetMat(image_msg)) {
      TDT_ERROR("Failed to get image from hikvision cam");
    }
    if (!setting_.get_timeStamp) takeTime_ = -1;
  }

  // locker_.unlock();
  // std::unique_lock<std::mutex> locker(img_mtx_);
  // swap(takeImg_, swapImg_);
  // swapTime_ = takeTime_;
  // locker.unlock();
  // condVar_.notify_all();
  // }
}

bool HikvisionCam::GetImage(tdtcamera::TImage &timage) {
  double start_tick = cv::getTickCount();
  cv::Mat tmp1;
  swap(tmp1, srcImg_);

  std::unique_lock<std::mutex> locker(img_mtx_);
  while (swapImg_.empty()) {
    condVar_.wait(locker);
  }
  // condVar_.wait(locker);
  // srcImg_ = swapImg_.clone();
  swap(srcImg_, swapImg_);
  srcTime_ = swapTime_;
  locker.unlock();
  timage = TImage(srcImg_, ++seq_);
  double cost_time =
      (cv::getTickCount() - start_tick) / cv::getTickFrequency() * 1000;
  if (cost_time > 1) {
    TDT_WARNING("HikvisionCam::GetImage cost time %f ms", cost_time);
  }
}

bool HikvisionCam::RestartCamera() { return hikvisioncam_.RestartCamera(); }

VideoDebug::VideoDebug(std::string config_path) : Camera(config_path) {
  if (!capture_.open(identity_.dev_path)) {
    if (!capture_.open(identity_.video_path)) {
      // char path[40];
      // sprintf(path, "../videodebug/%d.avi", identity_.dev_index);
      // if (!capture_.open(path)) {
      //   sprintf(path, "../videodebug/%s", identity_.dev_path.c_str());
      //   if (!capture_.open(path)) {
      //     sprintf(path, "../videodebug/%s", identity_.video_path.c_str());
      //     if (!capture_.open(path)) {
      //       TDT_ERROR("open video file fail!\n");
      //     }
      //   }
      // }
    }
  }
  if (capture_.isOpened()) {
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, format_.width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, format_.height);
    capture_.set(cv::CAP_PROP_FPS, format_.fps);
    capture_.set(cv::CAP_PROP_FOURCC,
                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // capture_.set(cv::CAP_PROP_CONVERT_RGB, false);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // capture_.set(cv::CAP_PROP_AUTOFOCUS, 0);
    capture_.set(cv::CAP_PROP_FOCUS, 0);
    // capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
    capture_.set(cv::CAP_PROP_EXPOSURE, setting_.exposure);
    // capture_.set(cv::CAP_PROP_AUTO_WB, 0);
    // capture_.set(cv::CAP_PROP_WB_TEMPERATURE, 0);
    capture_.set(cv::CAP_PROP_BRIGHTNESS, setting_.brightness);
    capture_.set(cv::CAP_PROP_CONTRAST, setting_.contrast);
    capture_.set(cv::CAP_PROP_SATURATION, setting_.saturation);
    capture_.set(cv::CAP_PROP_HUE, setting_.hue);
    capture_.set(cv::CAP_PROP_GAIN, setting_.gain);
    capture_.set(cv::CAP_PROP_GAMMA, setting_.gamma);
    capture_.set(cv::CAP_PROP_SHARPNESS, setting_.sharpness);
    // capture_.set(cv::CAP_PROP_BACKLIGHT, 0);
    // capture_.set(cv::CAP_PROP_PAN, 0);
    // capture_.set(cv::CAP_PROP_TILT, 0);
    // capture_.set(cv::CAP_PROP_ZOOM, 0);
    // capture_.set(cv::CAP_PROP_ROLL, 0);
    // capture_.set(cv::CAP_PROP_IRIS, 0);
    // capture_.set(cv::CAP_PROP_SETTINGS, 0);
    std::cout << "VideoDebug::VideoDebug" <<"width"<< capture_.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"height"<< capture_.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"fps"<< capture_.get(cv::CAP_PROP_FPS) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"fourcc"<< capture_.get(cv::CAP_PROP_FOURCC) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"buffersize"<< capture_.get(cv::CAP_PROP_BUFFERSIZE) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"exposure"<< capture_.get(cv::CAP_PROP_EXPOSURE) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"brightness"<< capture_.get(cv::CAP_PROP_BRIGHTNESS) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"contrast"<< capture_.get(cv::CAP_PROP_CONTRAST) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"saturation"<< capture_.get(cv::CAP_PROP_SATURATION) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"hue"<< capture_.get(cv::CAP_PROP_HUE) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"gain"<< capture_.get(cv::CAP_PROP_GAIN) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"gamma"<< capture_.get(cv::CAP_PROP_GAMMA) << std::endl;
    std::cout << "VideoDebug::VideoDebug" <<"sharpness"<< capture_.get(cv::CAP_PROP_SHARPNESS) << std::endl;
    std::thread t(&VideoDebug::TakeFrame, this);
    t.detach();
  }
}

void VideoDebug::TakeFrame() {
  while (true) {
    // 虚拟摄像头特有: 一帧一帧读
    if (swapImg_.empty()) {
      capture_ >> takeImg_;

      std::unique_lock<std::mutex> locker(img_mtx_);
      swap(takeImg_, swapImg_);
      locker.unlock();
      condVar_.notify_all();
    }
  }
}

bool VideoDebug::GetImage(tdtcamera::TImage &timage) {
  std::unique_lock<std::mutex> locker(img_mtx_, std::try_to_lock);
  cv::Mat tmp;
  swap(tmp, srcImg_);
  // if (swapImg_.empty()) return 0;
  // condVar_.wait(locker);
  swap(srcImg_, swapImg_);

  // locker.unlock();
  timage = TImage(srcImg_, ++seq_);
  return true;
}

}  // namespace tdtcamera
