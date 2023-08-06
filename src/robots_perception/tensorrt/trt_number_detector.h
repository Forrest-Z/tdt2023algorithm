#ifndef TRT_NUMBER_DETECTOR
#define TRT_NUMBER_DETECTOR

#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "NvOnnxParser.h"
// #include "config.h"
#include "cuda_runtime_api.h"
#include "cuda_utils.h"
#include "logging.h"
#include "preprocess.h"
#include "trt_yolo_detector.h"

using namespace nvinfer1;
namespace trt_detector {
class trt_number_detector {
 public:
  trt_number_detector(){};
  ~trt_number_detector();
  void init(std::string number_engine_name, std::string number_onnx_name,
            bool gen_engine);
  void deserialize_engine(const std::string &engine_name, IRuntime **runtime,
                          ICudaEngine **engine, IExecutionContext **context);
  void saveToTrtModel(const std::string &TrtSaveFileName,
                      IHostMemory *trtModelStream);
  void onnxToTRTModel(const std::string &modelFile,
                      const std::string &TrtSaveFileName);
  void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer,
                       float **gpu_output_buffer, float **cpu_output_buffer);
  void infer(IExecutionContext &context, cudaStream_t &stream,
             void **gpu_buffers, float *output);
  int detect(
      std::vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
      cv::Mat &img);
  //   int detect(std::vector<tdtml::YoloDetector::Object> &detected_armors,
  //              std::vector<armor_detect::ArmorDetectInfo> &armors,
  //              cv::Mat &src, cv::Mat &src_padding);

 private:
  nvinfer1::IRuntime *runtime;
  nvinfer1::ICudaEngine *engine;
  nvinfer1::IExecutionContext *context;
  Logger gLogger;

  uint8_t *img_buffer_host = nullptr;
  float *prob;
  float *input_data = nullptr;
  float *output_data = nullptr;
  float *gpu_buffers[2];
  float *cpu_output_buffer;

  cudaStream_t stream;

  int inputIndex;
  int outputIndex;
  int kInputSize = 1;
  int kOutputSize = 1;

  const static int kMaxInputImageSize = 640 * 480;
  const static int kInputH = 28;
  const static int kInputW = 20;
  const static int kBatchSize = 1;
  const static constexpr char *kInputTensorName = "input";
  const static constexpr char *kOutputTensorName = "output";

  std::string number_engine_name;
  std::string number_onnx_name;
  bool gen_engine;
  const static int kGpuId = 0;
};
}  // namespace trt_detector

#endif