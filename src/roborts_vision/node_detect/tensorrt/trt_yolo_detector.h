#ifndef YOLOV5_YOLOV5_PREDICTOR_H
#define YOLOV5_YOLOV5_PREDICTOR_H
#include <NvInfer.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "NvInferRuntime.h"
#include "NvOnnxParser.h"
#include "calibrator.h"
// #include "config.h"
#include "cuda_runtime_api.h"
#include "cuda_utils.h"
#include "logging.h"
#include "preprocess.h"
#include "utils.h"
#include "cpm.hpp"
#include "infer.hpp"
#include "yolo.hpp"
using namespace nvinfer1;

namespace trt_detector {
class trt_yolo_detector {
 public:
        typedef struct {
            float prob;
            cv::Rect rect;
            std::vector<float> points;
            cv::Mat number_image;
            bool big_armor;
            int armor_id;
    /* NO=0,
    BLUE  1=1, BLUE_2=2, BLUE_3=3, BLUE_4=4, BLUE_5=5, OUTPOST=6, SENTRY=7,
    BASE=8, RED  1=9, RED_2=10, RED_3=11, RED_4=12, RED_5=13, OUTPOST=14,
    SENTRY=15, BASE=16
    */
        } Object;
  trt_yolo_detector(){};
  ~trt_yolo_detector();
  void init(int srcH,int srcW,bool gen_engine,std::string yolo_engine_name,std::string yolo_onnx_name,float kNmsThresh,float kConfThresh,int kBatchSize);
  void initParams();
  void detect(cv::Mat &img, std::vector<Object> &detected_armors,
              std::vector<Object> &detected_robots);
  void batch_detect(std::vector<cv::Mat> &img_batch,
                    std::vector<std::vector<Object>> &detected_armors,
                    std::vector<std::vector<Object>> &detected_robots);

  void single_detect(cv::Mat &img, std::vector<Object> &detected_armors,
                     std::vector<Object> &detected_robots);
  void saveToTrtModel(const std::string &TrtSaveFileName,
                      IHostMemory *trtModelStream);
  void onnxToTRTModel(const std::string &modelFile,
                      const std::string &TrtSaveFileName);
  void onnxToTRTModelDynamicBatch(
      const std::string &modelFile,
      const std::string
          &TrtSaveFileName);  // output buffer for the TensorRT model 动态batch
  void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer,
                       float **gpu_output_buffer, float **cpu_output_buffer);
  void deserialize_engine(const std::string &engine_name, IRuntime **runtime,
                          ICudaEngine **engine, IExecutionContext **context);
  void infer(IExecutionContext &context, cudaStream_t &stream,
             void **gpu_buffers, float *output);
  void detet2origin(const cv::Rect &dete_rect, cv::Rect &out_rect,
                    const std::vector<float> &points,
                    std::vector<float> &out_points, float rate_to, int top,
                    int left);
  void to_final_objects(
      std::vector<trt_detector::trt_yolo_detector::Object> &final_objects,
      std::vector<int> &final_id, std::vector<cv::Rect> &origin_rect,
      std::vector<float> &origin_rect_cof,
      std::vector<std::vector<float>> &origin_point);
  bool parse_yolov5(float *output_blob, float cof_threshold,
                    std::vector<cv::Rect> &armors_rect,
                    std::vector<float> &armors_rect_cof,
                    std::vector<std::vector<float>> &armors_points,
                    std::vector<cv::Rect> &robots_rect,
                    std::vector<float> &robots_rect_cof,
                    std::vector<std::vector<float>> &robots_points);
  cv::Mat letterbox(cv::Mat &src);

 private:
 std::shared_ptr<yolo::Infer> yolo;


  nvinfer1::IRuntime *runtime;
  nvinfer1::ICudaEngine *engine;
  nvinfer1::IExecutionContext *context;
  Logger gLogger;

  float *prob;
  uint8_t *img_buffer_host = nullptr;
  float *input_data = nullptr;
  float *output_data = nullptr;
  float *gpu_buffers[2];
  float *cpu_output_buffer;

  cudaStream_t stream;

  int inputIndex;
  int outputIndex;
  int kOutputSize = 1;
  float _ratio;
  int resize_w;
  int resize_h;
  int _topPad;
  int _btmPad;
  int _leftPad;
  int _rightPad;
  int srcH;
  int srcW;

  const static int kInputH = 480;
  const static int kInputW = 640;
  int kBatchSize;
  const static constexpr char *kInputTensorName = "input";
  const static constexpr char *kOutputTensorName = "output";

  bool gen_engine;
  std::string yolo_engine_name;
  std::string yolo_onnx_name;
  float kNmsThresh;
  float kConfThresh;
  const static int kGpuId=0;
};
inline static bool RectSafety(cv::Rect2f &rect, const cv::Size2f &size) {
  // cv::Rect2f out_rect=cv::Rect2f(0,0,size.width,size.height);
  // out_rect=rect&out_rect;
  // return !out_rect.area()<rect.area();
  if (rect.x > 0 && rect.y > 0 && (rect.x + rect.width) < size.width &&
      (rect.y + rect.height) < size.height) {
    return true;
  } else {
    return false;
  }
}

}  // namespace trt_detector

#endif  // YOLOV5_YOLOV5_PREDICTOR_H
