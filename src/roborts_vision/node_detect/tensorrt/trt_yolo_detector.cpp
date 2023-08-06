
#include "trt_yolo_detector.h"
// #include "postprocess.h"
#include "chrono"

void trt_detector::trt_yolo_detector::saveToTrtModel(
    const std::string &TrtSaveFileName, IHostMemory *trtModelStream) {
  std::ofstream out(TrtSaveFileName, std::ios::binary);
  if (!out.is_open()) {
    std::cout << "打开文件失败!" << std::endl;
  }
  out.write(reinterpret_cast<const char *>(trtModelStream->data()),
            trtModelStream->size());
  out.close();
}

void trt_detector::trt_yolo_detector::onnxToTRTModel(
    const std::string &modelFile, const std::string &TrtSaveFileName) {
  IHostMemory *trtModelStream;
  int verbosity = (int)nvinfer1::ILogger::Severity::kWARNING;

  // create the builder
  IBuilder *builder =
      createInferBuilder(gLogger);  // 创建构建器(即指向Ibuilder类型对象的指针)
  IBuilderConfig *config = builder->createBuilderConfig();
  const auto explicitBatch =
      1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::
                                      kEXPLICIT_BATCH);  // 必须加不然报错
  nvinfer1::INetworkDefinition *network = builder->createNetworkV2(
      explicitBatch);  // 创建网络(即指向INetworkDefinition类型对象的指针)
  // explicitBatch指示TensorRT网络定义器（Network Definition
  // Builder）显式地支持可变批次大小（Variable batch
  // size）。当使用该标志构建引擎时，TensorRT会在构建引擎时明确指定输入数据的批次大小，从而允许在推理时动态地更改批次大小，从而获得更好的灵活性和效率。
  // enqueue()方法已被弃用。请改用enqueueV2()方法。此外，传递给此函数的batchSize参数不会改变输入形状。请改用setBindingDimensions()函数来改变输入形状。
  /*等价于*bulider.createNetwork(),通过Ibulider定义的
  名为creatNetwork()方法，创建INetworkDefinition的对象，ntework这个指针指向这个对象*/

  auto parser = nvonnxparser::createParser(
      *network, gLogger.getTRTLogger());  // 创建解析器

  // Optional - uncomment below lines to view network layer information
  // config->setPrintLayerInfo(true);
  // parser->reportParsingInfo();

  if (!parser->parseFromFile(modelFile.c_str(),
                             verbosity))  // 解析onnx文件，并填充网络
  {
    std::string msg("failed to parse onnx file");
    gLogger.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  // Build the engine
  builder->setMaxBatchSize(kBatchSize);

  config->setMaxWorkspaceSize(1 << 30);

  auto input_tensor = network->getInput(0);
  auto input_dims = input_tensor->getDimensions();
  int net_num_input = network->getNbInputs();
  auto output_tensor = network->getOutput(0);
  auto output_dims = output_tensor->getDimensions();
  int net_num_output = network->getNbOutputs();
  std::cout << "input_dims: " << input_dims.d[0] << " " << input_dims.d[1]
            << " " << input_dims.d[2] << " " << input_dims.d[3] << std::endl;

  // builder->setMaxWorkspaceSize(1 << 30);
#ifdef USE_FP16
  config->setFlag(BuilderFlag::kFP16);
#endif
  // samplesCommon::enableDLA(builder, gUseDLACore);
  // 当引擎建立起来时，TensorRT会复制
  // ICudaEngine* engine =
  // builder->buildCudaEngine(*network);//通过Ibuilder类的buildCudaEngine()方法创建IcudaEngine对象，
  ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
  assert(engine);

  // we can destroy the parser
  parser->destroy();

  // serialize the engine,
  // then close everything down
  trtModelStream = engine->serialize();  // 将引擎序列化，保存到文件中
  saveToTrtModel(TrtSaveFileName, trtModelStream);

  engine->destroy();
  network->destroy();
  config->destroy();

  builder->destroy();
}

void trt_detector::trt_yolo_detector::onnxToTRTModelDynamicBatch(
    const std::string &modelFile,
    const std::string
        &TrtSaveFileName)  // output buffer for the TensorRT model 动态batch
{
  int verbosity = (int)nvinfer1::ILogger::Severity::kWARNING;
  // create the builder
  IBuilder *builder =
      createInferBuilder(gLogger);  // 创建构建器(即指向Ibuilder类型对象的指针)
  IBuilderConfig *config = builder->createBuilderConfig();

  const auto explicitBatch =
      1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::
                                      kEXPLICIT_BATCH);  // 必须加不然报错
  nvinfer1::INetworkDefinition *network =
      builder->createNetworkV2(explicitBatch);
  /*等价于*bulider.createNetwork(),通过Ibulider定义的
  名为creatNetwork()方法，创建INetworkDefinition的对象，ntework这个指针指向这个对象*/

  auto parser = nvonnxparser::createParser(
      *network, gLogger.getTRTLogger());  // 创建解析器

  // Optional - uncomment below lines to view network layer information
  // config->setPrintLayerInfo(true);
  // parser->reportParsingInfo();

  if (!parser->parseFromFile(modelFile.c_str(),
                             verbosity))  // 解析onnx文件，并填充网络
  {
    std::string msg("failed to parse onnx file");
    gLogger.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  auto input_tensor = network->getInput(0);
  auto input_dims = input_tensor->getDimensions();
  int net_num_input = network->getNbInputs();
  auto output_tensor = network->getOutput(0);
  auto output_dims = output_tensor->getDimensions();
  int net_num_output = network->getNbOutputs();

  builder->setMaxBatchSize(kBatchSize);
  config->setMaxWorkspaceSize(1 << 30);

  // Dims dims         = Dims4{1, 3, kInputH, kInputW};
  auto profile = builder->createOptimizationProfile();
  input_dims.d[0] = kBatchSize;
  profile->setDimensions(input_tensor->getName(), OptProfileSelector::kMIN,
                         input_dims);
  profile->setDimensions(input_tensor->getName(), OptProfileSelector::kOPT,
                         input_dims);
  input_dims.d[0] = kBatchSize;
  profile->setDimensions(input_tensor->getName(), OptProfileSelector::kMAX,
                         input_dims);
  config->addOptimizationProfile(profile);

  // Build the engine
  // builder->setMaxBatchSize(maxBatchSize);
  // builder->setMaxWorkspaceSize(1 << 30);
#ifdef USE_FP16
  config->setFlag(BuilderFlag::kFP16);
#endif
  // samplesCommon::enableDLA(builder, gUseDLACore);
  // 当引擎建立起来时，TensorRT会复制
  // ICudaEngine* engine =
  // builder->buildCudaEngine(*network);//通过Ibuilder类的buildCudaEngine()方法创建IcudaEngine对象，
  ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
  assert(engine);

  // we can destroy the parser
  parser->destroy();

  // serialize the engine,
  // then close everything down
  IHostMemory *trtModelStream;

  trtModelStream = engine->serialize();  // 将引擎序列化，保存到文件中
  saveToTrtModel(TrtSaveFileName, trtModelStream);

  engine->destroy();
  network->destroy();
  config->destroy();
  builder->destroy();
}

void trt_detector::trt_yolo_detector::prepare_buffers(
    ICudaEngine *engine, float **gpu_input_buffer, float **gpu_output_buffer,
    float **cpu_output_buffer) {
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and
  // output tensors. Note that indices are guaranteed to be less than
  // IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  auto out_dims = engine->getBindingDimensions(1);
  auto in_dims = engine->getBindingDimensions(0);

  for (int j = 0; j < out_dims.nbDims; j++) {
    kOutputSize *= out_dims.d[j];
  }
  kOutputSize /= kBatchSize;
  // // Create GPU buffers on device
  // CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer,
  //                       kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  // CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer,
  //                       kBatchSize * kOutputSize * sizeof(float)));

  // *cpu_output_buffer = new float[kBatchSize * kOutputSize];

  CUDA_CHECK(cudaHostAlloc((void **)&input_data,
                           kBatchSize * 3 * kInputH * kInputW * sizeof(float),
                           cudaHostAllocMapped));
  CUDA_CHECK(cudaHostAlloc((void **)&output_data,
                           kBatchSize * kOutputSize * sizeof(float),
                           cudaHostAllocMapped));

  // // Set input and output GPU buffers
  CUDA_CHECK(cudaHostGetDevicePointer((void **)gpu_input_buffer,
                                      (void *)input_data, 0));
  CUDA_CHECK(cudaHostGetDevicePointer((void **)gpu_output_buffer,
                                      (void *)output_data, 0));

  // // Save CPU output buffer
  *cpu_output_buffer = output_data;
}
void trt_detector::trt_yolo_detector::deserialize_engine(
    const std::string &engine_name, IRuntime **runtime, ICudaEngine **engine,
    IExecutionContext **context) {
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good()) {
    std::cerr << "read " << engine_name << " error!" << std::endl;
    assert(false);
  }
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char *serialized_engine = new char[size];
  assert(serialized_engine);
  file.read(serialized_engine, size);
  file.close();

  *runtime = createInferRuntime(gLogger);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}

void trt_detector::trt_yolo_detector::infer(IExecutionContext &context,
                                            cudaStream_t &stream,
                                            void **gpu_buffers, float *output) {
  //    context.enqueueV2((void**)buffers, stream, nullptr);
  // context.setBindingDimensions(inputIndex, Dims{4, {kBatchSize, 3, kInputH,
  // kInputW}});
  // std::chrono ::steady_clock::time_point t1 =
  // std::chrono::steady_clock::now();
  context.enqueueV2(gpu_buffers, stream, nullptr);
  // std::chrono ::steady_clock::time_point t2 =
  // std::chrono::steady_clock::now(); std::chrono ::duration<double> time_used
  // =
  //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // std::cout << "enqueueV2 time: " << time_used.count() * 1000 << "ms"
  // << std::endl;

  // CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1],
  //                            kBatchSize * kOutputSize * sizeof(float),
  //                            cudaMemcpyDeviceToHost, stream));
  // std::chrono ::steady_clock::time_point t3 =
  // std::chrono::steady_clock::now(); time_used =
  //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
  // std::cout << "cudaMemcpyAsync time: " << time_used.count() * 1000 << "ms"
  //           << std::endl;

  cudaStreamSynchronize(stream);
  // std::chrono ::steady_clock::time_point t4 =
  // std::chrono::steady_clock::now(); time_used =
  //     std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
  // std::cout << "cudaStreamSynchronize time: " << time_used.count() * 1000
  //           << "ms" << std::endl;
}

// void batch_nms(std::vector<std::vector<Yolo::Detection>>& res_batch, float
// *output, int batch_size, int output_size, float conf_thresh, float
// nms_thresh) {
//     res_batch.resize(batch_size);
//     for (int i = 0; i < batch_size; i++) {
//         nms(res_batch[i], &output[i * output_size], conf_thresh, nms_thresh);
//     }
// }

bool trt_detector::trt_yolo_detector::parse_yolov5(
    float *output_blob, float cof_threshold, std::vector<cv::Rect> &armors_rect,
    std::vector<float> &armors_rect_cof,
    std::vector<std::vector<float>> &armors_points,
    std::vector<cv::Rect> &robots_rect, std::vector<float> &robots_rect_cof,
    std::vector<std::vector<float>> &robots_points) {
  // 25200 if 640
  // 14175 if 480
  // 0~3是xywh
  // 4是置信度
  // 5~6是各类置信度
  // 7~14是特征点xy

  //   for (int i = 0; i < 18900 * 15; i += 15) {
  //     double box_prob = output_blob[i + 4];

  //     // 框置信度不满足则整体置信度不满足
  //     if (box_prob < cof_threshold) {
  //       continue;
  //     }

  //     // 注意此处输出为中心点坐标,需要转化为角点坐标

  //     double x = output_blob[i + 0];
  //     double y = output_blob[i + 1];
  //     double w = output_blob[i + 2];
  //     double h = output_blob[i + 3];

  //     double max_prob = 0;
  //     int idx = 0;
  //     for (int t = 5; t < 7; ++t) {
  //       double tp = output_blob[i + t];
  //       if (tp > max_prob) {
  //         max_prob = tp;
  //         idx = t;
  //       }
  //     }
  //     float cof = box_prob * max_prob;
  //     // 对于边框置信度小于阈值的边框,不关心其他数值,不进行计算减少计算量
  //     // 出于精度考虑还是增加上去
  //     cv::Rect rect =
  //         cv::Rect(round(x - w / 2), round(y - h / 2), round(w), round(h));
  //     std::vector<float> point;
  //     for (int j = 7; j < 15; ++j) {
  //       point.push_back(output_blob[i + j]);
  //     }

  //     if (idx == 5) {  // 装甲板
  //       armors_points.push_back(point);
  //       armors_rect.push_back(rect);
  //       armors_rect_cof.push_back(cof);
  //     } else {  // 车
  //       robots_points.push_back(point);
  //       robots_rect.push_back(rect);
  //       robots_rect_cof.push_back(cof);
  //     }
  //   }
  //   return true;
  // }
  for (int i = 0; i < 18900 * 15; i += 15) {
    double box_prob = output_blob[i + 4];

    //框置信度不满足则整体置信度不满足
    if (box_prob < cof_threshold) {
      continue;
    }

    //注意此处输出为中心点坐标,需要转化为角点坐标

    double x = output_blob[i + 0];
    double y = output_blob[i + 1];
    double w = output_blob[i + 2];
    double h = output_blob[i + 3];

    double max_prob = 0;
    int idx = 0;
    for (int t = 13; t < 15; ++t) {
      double tp = output_blob[i + t];
      if (tp > max_prob) {
        max_prob = tp;
        idx = t;
      }
    }
    float cof = box_prob * max_prob;
    //对于边框置信度小于阈值的边框,不关心其他数值,不进行计算减少计算量
    //出于精度考虑还是增加上去

    cv::Rect rect =
        cv::Rect(round(x - w / 2), round(y - h / 2), round(w), round(h));

    std::vector<float> point;
    for (int j = 5; j < 13; ++j) {
      point.push_back(output_blob[i + j]);
    }
    if (idx == 13) {  //装甲板
      armors_points.push_back(point);
      armors_rect.push_back(rect);
      armors_rect_cof.push_back(cof);

    } else {  //车
      robots_points.push_back(point);
      robots_rect.push_back(rect);
      robots_rect_cof.push_back(cof);
    }
  }
  return true;
}
trt_detector::trt_yolo_detector::~trt_yolo_detector() {
  cudaStreamDestroy(stream);
  // cuda_preprocess_destroy();
  // CUDA_CHECK(cudaFree(gpu_buffers[0]));
  // CUDA_CHECK(cudaFree(gpu_buffers[1]));
  // delete[] cpu_output_buffer;
  CUDA_CHECK(cudaFreeHost(input_data));
  CUDA_CHECK(cudaFreeHost(output_data));
  CUDA_CHECK(cudaFreeHost(img_buffer_host));

  context->destroy();
  engine->destroy();
  runtime->destroy();
}

void trt_detector::trt_yolo_detector::init(int srcH, int srcW, bool gen_engine,
                                           std::string yolo_engine_name,
                                           std::string yolo_onnx_name,
                                           float kNmsThresh, float kConfThresh,
                                           int kBatchSize) {
  this->srcH = srcH;
  this->srcW = srcW;
  this->gen_engine = gen_engine;
  this->yolo_engine_name = yolo_engine_name;
  this->yolo_onnx_name = yolo_onnx_name;
  this->kNmsThresh = kNmsThresh;
  this->kConfThresh = kConfThresh;
  this->kBatchSize = kBatchSize;
  std::cout << "init trt_yolo_detector\n";
  cudaSetDevice(this->kGpuId);
  if (gen_engine) {
    onnxToTRTModel(yolo_onnx_name, yolo_engine_name);
  }
  yolo = yolo::load(yolo_engine_name, yolo::Type::V5Face,kConfThresh,kNmsThresh);
  if (yolo == nullptr) {
    std::cout << "load engine failed!" << std::endl;
    return;
  }
  std::cout << "init success!" << std::endl;
  // runtime = nullptr;
  // engine = nullptr;
  // context = nullptr;
  // deserialize_engine(yolo_engine_name, &runtime, &engine, &context);
  // std::cout << "init trt_yolo_detector done\n";
  // CUDA_CHECK(cudaStreamCreate(&stream));
  // // cuda_preprocess_init(kMaxInputImageSize);
  // CUDA_CHECK(cudaMallocHost((void **)&img_buffer_host, srcH*srcW * 3));

  // cpu_output_buffer = nullptr;
  // prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1],
  // &cpu_output_buffer);
  // // static float temp[kBatchSize * OUTPUT_SIZE];
  // // this->prob=temp;
  // initParams();
}
void trt_detector::trt_yolo_detector::initParams() {
  _ratio = std::min(float(kInputH) / srcH, float(kInputW) / srcW);
  resize_w = std::round(srcW * _ratio);
  resize_h = std::round(srcH * _ratio);
  int pad_w = kInputW - resize_w;
  int pad_h = kInputH - resize_h;
  pad_w = pad_w / 2;
  pad_h = pad_h / 2;
  _topPad = int(std::round(pad_h - 0.1));
  _btmPad = int(std::round(pad_h + 0.1));
  _leftPad = int(std::round(pad_w - 0.1));
  _rightPad = int(std::round(pad_w + 0.1));
}

// bool trt_detector::trt_yolo_detector::process_frame(
//     Mat &inframe, vector<Object> &detected_armors,
//     vector<Object> &detected_robots) {
//   if (inframe.empty()) {
//     cout << "无效图片输入" << endl;
//     return false;
//   }

//   //变换尺寸兼载入数据

//   auto in_data = input_tensor.data<uint8_t>();
//   Mat data_mat(480, 640, CV_8UC3, in_data);
//   resize(inframe, data_mat, Size(640, 480));

//   vector<Rect> armors_rect;
//   vector<float> armors_rect_cof;
//   vector<vector<float>> armors_points;

//   vector<Rect> robots_rect;
//   vector<float> robots_rect_cof;
//   vector<vector<float>> robots_points;

//   parse_yolov5(out_data, _cof_threshold, armors_rect, armors_rect_cof,
//                armors_points, robots_rect, robots_rect_cof, robots_points);

//   vector<int> armors_final_id;
//   vector<int> robots_final_id;

//   dnn::NMSBoxes(armors_rect, armors_rect_cof, _cof_threshold, 0,
//                 armors_final_id);
//   dnn::NMSBoxes(robots_rect, robots_rect_cof, _cof_threshold,
//                 _nms_area_threshold, robots_final_id);

//   // 根据final_id获取最终结果
//   to_final_objects(detected_armors, armors_final_id, armors_rect,
//                    armors_rect_cof, armors_points);
//   to_final_objects(detected_robots, robots_final_id, robots_rect,
//                    robots_rect_cof, robots_points);

//   return true;
// }
cv::Mat trt_detector::trt_yolo_detector::letterbox(cv::Mat &src) {
  if (src.empty()) {
    std::cout << "input image invalid" << std::endl;
    return cv::Mat();
  }
  if (src.rows == kInputH && src.cols == kInputW) {
    return src;
  }

  cv::Mat resize_img;

  cv::resize(src, resize_img, cv::Size(resize_w, resize_h));

  cv::copyMakeBorder(resize_img, resize_img, _topPad, _btmPad, _leftPad,
                     _rightPad, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return resize_img;
}

void trt_detector::trt_yolo_detector::detet2origin(
    const cv::Rect &dete_rect, cv::Rect &out_rect,
    const std::vector<float> &points, std::vector<float> &out_points,
    float rate_to, int top, int left) {
  // detect坐标转换到内部纯图坐标
  int inside_x = dete_rect.x - left;
  int inside_y = dete_rect.y - top;
  int ox = round(float(inside_x) / rate_to);
  int oy = round(float(inside_y) / rate_to);
  int ow = round(float(dete_rect.width) / rate_to);
  int oh = round(float(dete_rect.height) / rate_to);
  out_rect = cv::Rect(ox, oy, ow, oh);
  for (int i = 0; i < points.size(); i = i + 2) {
    float x = (points[i] - left) / rate_to;
    float y = (points[i + 1] - top) / rate_to;
    out_points.push_back(x);
    out_points.push_back(y);
  }
}

void trt_detector::trt_yolo_detector::to_final_objects(
    std::vector<trt_detector::trt_yolo_detector::Object> &final_objects,
    std::vector<int> &final_id, std::vector<cv::Rect> &origin_rect,
    std::vector<float> &origin_rect_cof,
    std::vector<std::vector<float>> &origin_point) {
  for (int i = 0; i < final_id.size(); ++i) {
    cv::Rect resize_rect = origin_rect[final_id[i]];
    cv::Rect rawrect;
    std::vector<float> points = origin_point[final_id[i]];
    std::vector<float> output_points;
    detet2origin(resize_rect, rawrect, points, output_points, _ratio, _topPad,
                 _leftPad);
    final_objects.push_back(trt_detector::trt_yolo_detector::Object{
        origin_rect_cof[final_id[i]], rawrect, output_points});
  }
}

yolo::Image cvimg(const cv::Mat &image) {
  return yolo::Image(image.data, image.cols, image.rows);
}
void trt_detector::trt_yolo_detector::detect(
    cv::Mat &img, std::vector<Object> &detected_armors,
    std::vector<Object> &detected_robots) {
  yolo::BoxArray objs;

  objs = yolo->forward(cvimg(img));

  for (auto &obj : objs) {
    cv::Rect rect = cv::Rect(cv::Point(obj.left, obj.top),
                             cv::Point(obj.right, obj.bottom));
    if (obj.class_label == 0) {
      detected_armors.push_back(trt_detector::trt_yolo_detector::Object{
          obj.confidence, rect, obj.points, cv::Mat(), false, 0});
    } else if (obj.class_label == 1) {
      detected_robots.push_back(trt_detector::trt_yolo_detector::Object{
          obj.confidence, rect, obj.points, cv::Mat(), false, 0});
    }
  }
  // int dst_size = kInputW * kInputH * 3;

  // cv::Mat resize_mat;
  // resize_mat = letterbox(img);

  // cuda_preprocess(img_buffer_host, resize_mat.ptr(), resize_mat.cols,
  //                 resize_mat.rows, &gpu_buffers[0][0], kInputW, kInputH,
  //                 stream);
  // CUDA_CHECK(cudaStreamSynchronize(stream));

  // infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer);

  // std::vector<cv::Rect> armors_rect;
  // std::vector<float> armors_rect_cof;
  // std::vector<std::vector<float>> armors_points;

  // std::vector<cv::Rect> robots_rect;
  // std::vector<float> robots_rect_cof;
  // std::vector<std::vector<float>> robots_points;
  // parse_yolov5(&cpu_output_buffer[0], kConfThresh, armors_rect,
  // armors_rect_cof,
  //              armors_points, robots_rect, robots_rect_cof, robots_points);
  // std::vector<int> armors_final_id;
  // std::vector<int> robots_final_id;

  // cv::dnn::NMSBoxes(armors_rect, armors_rect_cof, kConfThresh, 0,
  //                   armors_final_id);
  // cv::dnn::NMSBoxes(robots_rect, robots_rect_cof, kConfThresh, kNmsThresh,
  //                   robots_final_id);
  // to_final_objects(detected_armors, armors_final_id, armors_rect,
  //                  armors_rect_cof, armors_points);
  // to_final_objects(detected_robots, robots_final_id, robots_rect,
  //                  robots_rect_cof, robots_points);
  // std::cout << "detected_armors size:" << detected_armors.size() <<
  // std::endl; std::cout << "detected_robots size:" << detected_robots.size()
  // << std::endl; for (int j = 0; j < detected_armors.size(); ++j) {
  //   cv::rectangle(img, detected_armors[j].rect, cv::Scalar(0, 0, 255), 2);
  //   for (int k = 0; k < detected_armors[j].points.size(); k = k + 2) {
  //     cv::circle(img,
  //                cv::Point(detected_armors[j].points[k],
  //                          detected_armors[j].points[k + 1]),
  //                2, cv::Scalar(0, 0, 255), 2);
  //   }
  //   // std::cout << "detected_armors prob:" << detected_armors[j].prob
  //   //           << std::endl;
  // }
  // for (int j = 0; j < detected_robots.size(); ++j) {
  //   cv::rectangle(img, detected_robots[j].rect, cv::Scalar(0, 255, 0), 2);
  //   for (int k = 0; k < detected_robots[j].points.size(); k = k + 2) {
  //     cv::circle(img,
  //                cv::Point(detected_robots[j].points[k],
  //                          detected_robots[j].points[k + 1]),
  //                2, cv::Scalar(0, 255, 0), 2);
  //   }
  //   // std::cout << "detected_robots prob:" << detected_robots[j].prob
  //   //           << std::endl;
  // }
  // cv::imshow("img", img);
  // cv::waitKey(1);
}

void trt_detector::trt_yolo_detector::batch_detect(
    std::vector<cv::Mat> &img_batch,
    std::vector<std::vector<Object>> &detected_armors,
    std::vector<std::vector<Object>> &detected_robots) {
  std::vector<yolo::Image> yoloimages(img_batch.size());
  std::transform(img_batch.begin(), img_batch.end(), yoloimages.begin(), cvimg);

  auto batched_result = yolo->forwards(yoloimages);
  for (int i = 0; i < batched_result.size(); ++i) {
    auto &objs = batched_result[i];
    for (auto &obj : objs) {
      cv::Rect rect = cv::Rect(cv::Point(obj.left, obj.top),
                               cv::Point(obj.right, obj.bottom));
      if (obj.class_label == 0) {
        detected_armors[i].push_back(trt_detector::trt_yolo_detector::Object{
            obj.confidence, rect, obj.points, cv::Mat(), false, 0});
      } else if (obj.class_label == 1) {
        detected_robots[i].push_back(trt_detector::trt_yolo_detector::Object{
            obj.confidence, rect, obj.points, cv::Mat(), false, 0});
      }
    }
  }

  /*
//
将图像输入cuda流,TODO这个里面图像处理看看，图像缩放很多开源用的仿射变换，这里用的是resize
//  TODO耗时差不多，主要还是看内存交流

std::vector<cv::Mat> resize_mat_batch;
for (int i = 0; i < img_batch.size(); ++i) {
resize_mat_batch.push_back(letterbox(img_batch[i]));
}
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
cuda_batch_preprocess(img_buffer_host, resize_mat_batch, gpu_buffers[0],
                    kInputW, kInputH, stream);

std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
auto time_used =
  std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
// std::cout << "preprocess time: " << time_used.count() * 1000 << " ms."
//           << std::endl;
// std::cout<<"infer\n";

infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer);

std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
time_used =
  std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
// std::cout << "infer time: " << time_used.count() * 1000 << " ms."
//           << std::endl;
for (size_t i = 0; i < img_batch.size(); i++) {
// std::vector<Object> detected_armors;
// std::vector<Object> detected_robots;
std::vector<cv::Rect> armors_rect;
std::vector<float> armors_rect_cof;
std::vector<std::vector<float>> armors_points;

std::vector<cv::Rect> robots_rect;
std::vector<float> robots_rect_cof;
std::vector<std::vector<float>> robots_points;
parse_yolov5(&cpu_output_buffer[i * kOutputSize], kConfThresh, armors_rect,
             armors_rect_cof, armors_points, robots_rect, robots_rect_cof,
             robots_points);
std::vector<int> armors_final_id;
std::vector<int> robots_final_id;

cv::dnn::NMSBoxes(armors_rect, armors_rect_cof, kConfThresh, 0,
                  armors_final_id);
cv::dnn::NMSBoxes(robots_rect, robots_rect_cof, kConfThresh, kNmsThresh,
                  robots_final_id);
to_final_objects(detected_armors[i], armors_final_id, armors_rect,
                 armors_rect_cof, armors_points);
to_final_objects(detected_robots[i], robots_final_id, robots_rect,
                 robots_rect_cof, robots_points);
// std::cout << "detected_armors size:" << detected_armors.size() <<
// std::endl; std::cout << "detected_robots size:" << detected_robots.size()
// << std::endl;
// for (int j = 0; j < detected_armors.size(); ++j) {
//   cv::rectangle(img_batch[i], detected_armors[i][j].rect,
//                 cv::Scalar(0, 0, 255), 2);
//   for (int k = 0; k < detected_armors[i][j].points.size(); k = k + 2) {
//     cv::circle(img_batch[i],
//                cv::Point(detected_armors[i][j].points[k],
//                          detected_armors[i][j].points[k + 1]),
//                2, cv::Scalar(0, 0, 255), 2);
//   }
//   // std::cout << "detected_armors prob:" << detected_armors[j].prob
//   //           << std::endl;
// }
// for (int j = 0; j < detected_robots.size(); ++j) {
//   cv::rectangle(img_batch[i], detected_robots[i][j].rect,
//                 cv::Scalar(0, 255, 0), 2);
//   for (int k = 0; k < detected_robots[i][j].points.size(); k = k + 2) {
//     cv::circle(img_batch[i],
//                cv::Point(detected_robots[i][j].points[k],
//                          detected_robots[i][j].points[k + 1]),
//                2, cv::Scalar(0, 255, 0), 2);
//   }
//   // std::cout << "detected_robots prob:" << detected_robots[j].prob
//   //           << std::endl;
// }
// cv::imshow("img", img_batch[i]);
// cv::waitKey(1);

}
*/
}
// void single_detect(cv::Mat &img,std::vector<Object> &detected_armors,
//                     std::vector<Object> &detected_robots){

//                     }

// int main() {
//   // std::cout << "1" << std::endl;
//   trt_detector::trt_yolo_detector predictor;
//   // std::cout << "2" << std::endl;
//   predictor.init();
//   for (int i = 0; i < 10000; i++) {
//     cv::VideoCapture cap;
//     cap.open("/home/tdt/视频/2022-7-11--15-43-43-54.avi");
//     if (!cap.isOpened()) {
//       std::cout << "open video failed" << std::endl;
//       return -1;
//     }
//     while (1) {
//       std::vector<cv::Mat> img_batch;
//       cv::Mat img1;
//       // cv::Mat img2;
//       // cv::Mat img3;
//       // cv::Mat img4;
//       cap >> img1;
//       // cap >> img2;
//       // cap >> img3;
//       // cap >> img4;
//       // cv::Mat              img1 = cv::imread("/home/tdt/图片/1.png");
//       // cv::Mat              img2 = cv::imread("/home/tdt/图片/2.png");
//       // cv::Mat              img3 = cv::imread("/home/tdt/图片/3.png");
//       // cv::Mat              img4 = cv::imread("/home/tdt/图片/4.png");
//       std::chrono ::steady_clock::time_point a1 =
//           std::chrono::steady_clock::now();
//       img_batch.push_back(img1);
//       std::chrono::steady_clock::time_point a2 =
//           std::chrono::steady_clock::now();
//       auto time_used2 =
//       std::chrono::duration_cast<std::chrono::duration<double>>(a2 - a1);
//       std::cout << "push_back time: " << time_used2.count() << std::endl;
//       // img_batch.push_back(img2);
//       // img_batch.push_back(img3);
//       // img_batch.push_back(img4);
//       std::chrono ::steady_clock::time_point t1 =
//           std::chrono::steady_clock::now();
//       std::vector<trt_detector::trt_yolo_detector::Object> detected_armors;
//       std::vector<trt_detector::trt_yolo_detector::Object> detected_robots;
//       // predictor.batch_detect(img_batch, detected_armors, detected_robots);
//       predictor.detect(img1, detected_armors, detected_robots);
//       std::chrono::steady_clock::time_point t2 =
//           std::chrono::steady_clock::now();
//       auto time_used =
//           std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//       std::cout << "batch_detect time: " << time_used.count() * 1000 << "
//       ms."
//                 << std::endl;
//       // cv::waitKey(10);
//     }
//   }
// }