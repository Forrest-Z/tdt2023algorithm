#ifndef _YOLO_DETECTOR_H_
#define _YOLO_DETECTOR_H_
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <openvino/opsets/opset1.hpp>
#include <openvino/runtime/intel_cpu/properties.hpp>
#include <openvino/runtime/intel_gpu/properties.hpp>
using namespace std;
using namespace cv;

namespace robots_perception {

class YoloDetector {
 public:
  typedef struct {
    float prob;  // 置信度
    int name;    // yolo输出13装甲板和14是车
               // ，若为14车则直接13+8*2转29为车，若为13装甲板则数字识别加类型0-7，之后传统颜色面积比较若为红再+8
    // 最终13-29为蓝1-8,红1-8；30为车
    cv::Rect rect;              // yolo原始Rect后处理在画面内
    std::vector<float> points;  // 特征点8个float转2*4
  } Object;
  YoloDetector();
  ~YoloDetector();
  // 初始化
  //  TODO
  //  还需要添加输出图像尺寸，是否使用gpu模式，以及使用的网络参数文件，后续再添加输入尺寸
  bool init(string xml_path, double cof_threshold, double nms_area_threshold,
            int output_width, int output_height, int use_gpu);
  // 释放资源
  bool uninit();
  void detet2origin(const Rect &dete_rect, Rect &out_rect,
                    const vector<float> &points, vector<float> &out_points,
                    float rate_to, int top, int left);

  // 处理图像获取结果
  bool process_frame(Mat &inframe, vector<Object> &detected_objects);

 private:
  double sigmoid(double x);
  vector<int> get_anchors(int net_grid);
  bool parse_yolov5(float *output_blob, float cof_threshold,
                    vector<Rect> &o_rect, vector<float> &o_rect_cof,
                    vector<vector<float>> &points, vector<int> &id);

  Rect detet2origin(const Rect &dete_rect, float rate_to, int top, int left);
  // 存储初始化获得的可执行网络
  ov::Core core;
  ov::CompiledModel compiled_model;
  ov::InferRequest infer_request;
  ov::Tensor input_tensor;
  ov::Tensor output_tensor;

  // 参数区
  string _xml_path;  // OpenVINO模型xml文件路径
  double _cof_threshold;  // 置信度阈值,计算方法是框置信度乘以物品种类置信度
  double _nms_area_threshold;  // nms最小重叠面积阈值
  int gpu_mode = 0;
  float f = 114.0f / 255.0f;
  float r = 640.0f / 640.0f;
  int top = 80;
  int left = 0;
};

}  // namespace robots_perception
#endif