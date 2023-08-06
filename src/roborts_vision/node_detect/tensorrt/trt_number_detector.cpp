#include "trt_number_detector.h"

void trt_detector::trt_number_detector::init(std::string number_engine_name,
                                             std::string number_onnx_name,
                                             bool gen_engine) {
  std::cout << "init trt_number_detector" << std::endl;
  this->number_engine_name = number_engine_name;
  this->number_onnx_name = number_onnx_name;
  this->gen_engine = gen_engine;
  cudaSetDevice(kGpuId);
  cudaSetDeviceFlags(cudaDeviceMapHost);
  if (gen_engine) {
    onnxToTRTModel(number_onnx_name, number_engine_name);
  }
  runtime = nullptr;
  engine = nullptr;
  context = nullptr;
  deserialize_engine(number_engine_name, &runtime, &engine, &context);
  std::cout << "init trt_number_detector done" << std::endl;
  CUDA_CHECK(cudaStreamCreate(&stream));
  // Allocate device memory for inputs.
  CUDA_CHECK(cudaMallocHost((void **)&img_buffer_host, kMaxInputImageSize));
  cpu_output_buffer = nullptr;
  prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);
}

void trt_detector::trt_number_detector::prepare_buffers(
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

  for (int j = 0; j < in_dims.nbDims; j++) {
    kInputSize *= in_dims.d[j];
  }
  kInputSize /= kBatchSize;
  std::cout << "kInputSize:" << kInputSize << std::endl;
  for (int j = 0; j < out_dims.nbDims; j++) {
    kOutputSize *= out_dims.d[j];
  }
  kOutputSize /= kBatchSize;
  std::cout << "kOutputSize:" << kOutputSize << std::endl;
  // // Create GPU buffers on device
  // CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer,
  //                       kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  // CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer,
  //                       kBatchSize * kOutputSize * sizeof(float)));

  // *cpu_output_buffer = new float[kBatchSize * kOutputSize];

  CUDA_CHECK(cudaHostAlloc((void **)&input_data,
                           kBatchSize * kInputSize * sizeof(float),
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

trt_detector::trt_number_detector::~trt_number_detector() {
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

void trt_detector::trt_number_detector::onnxToTRTModel(
    const std::string &modelFile, const std::string &TrtSaveFileName) {
  IHostMemory *trtModelStream;
  int verbosity = (int)nvinfer1::ILogger::Severity::kWARNING;
  // create the builder
  IBuilder *builder =
      createInferBuilder(gLogger);  //创建构建器(即指向Ibuilder类型对象的指针)
  IBuilderConfig *config = builder->createBuilderConfig();
  const auto explicitBatch =
      1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::
                                      kEXPLICIT_BATCH);  //必须加不然报错
  nvinfer1::INetworkDefinition *network = builder->createNetworkV2(
      explicitBatch);  //创建网络(即指向INetworkDefinition类型对象的指针)
  // explicitBatch指示TensorRT网络定义器（Network Definition
  // Builder）显式地支持可变批次大小（Variable batch
  // size）。当使用该标志构建引擎时，TensorRT会在构建引擎时明确指定输入数据的批次大小，从而允许在推理时动态地更改批次大小，从而获得更好的灵活性和效率。
  // enqueue()方法已被弃用。请改用enqueueV2()方法。此外，传递给此函数的batchSize参数不会改变输入形状。请改用setBindingDimensions()函数来改变输入形状。
  /*等价于*bulider.createNetwork(),通过Ibulider定义的
  名为creatNetwork()方法，创建INetworkDefinition的对象，ntework这个指针指向这个对象*/

  auto parser = nvonnxparser::createParser(
      *network, gLogger.getTRTLogger());  //创建解析器

  // Optional - uncomment below lines to view network layer information
  // config->setPrintLayerInfo(true);
  // parser->reportParsingInfo();

  if (!parser->parseFromFile(modelFile.c_str(),
                             verbosity))  //解析onnx文件，并填充网络
  {
    std::string msg("failed to parse onnx file");
    gLogger.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  // Build the engine
  // builder->setMaxBatchSize(kBatchSize);

  config->setMaxWorkspaceSize(1 << 30);

  auto input_tensor = network->getInput(0);
  auto input_dims = input_tensor->getDimensions();
  int net_num_input = network->getNbInputs();
  auto output_tensor = network->getOutput(0);
  auto output_dims = output_tensor->getDimensions();
  int net_num_output = network->getNbOutputs();
  // builder->setMaxWorkspaceSize(1 << 30);
  // #ifdef USE_FP16
  //   config->setFlag(BuilderFlag::kFP16);
  // #endif
  // samplesCommon::enableDLA(builder, gUseDLACore);
  //当引擎建立起来时，TensorRT会复制
  // ICudaEngine* engine =
  // builder->buildCudaEngine(*network);//通过Ibuilder类的buildCudaEngine()方法创建IcudaEngine对象，
  ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
  assert(engine);

  // we can destroy the parser
  parser->destroy();

  // serialize the engine,
  // then close everything down
  trtModelStream = engine->serialize();  //将引擎序列化，保存到文件中
  saveToTrtModel(TrtSaveFileName, trtModelStream);

  engine->destroy();
  network->destroy();
  config->destroy();

  builder->destroy();
}
void trt_detector::trt_number_detector::saveToTrtModel(
    const std::string &TrtSaveFileName, IHostMemory *trtModelStream) {
  std::ofstream out(TrtSaveFileName, std::ios::binary);
  if (!out.is_open()) {
    std::cout << "打开文件失败!" << std::endl;
  }
  out.write(reinterpret_cast<const char *>(trtModelStream->data()),
            trtModelStream->size());
  out.close();
}

void trt_detector::trt_number_detector::deserialize_engine(
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

void trt_detector::trt_number_detector::infer(IExecutionContext &context,
                                              cudaStream_t &stream,
                                              void **gpu_buffers,
                                              float *output) {
  //    context.enqueueV2((void**)buffers, stream, nullptr);
  // context.setBindingDimensions(inputIndex, Dims{4, {kBatchSize, 3, kInputH,
  // kInputW}});
  std::chrono ::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  context.enqueueV2(gpu_buffers, stream, nullptr);
  std::chrono ::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono ::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "enqueueV2 time: " << time_used.count() * 1000 << "ms"
            << std::endl;

  // CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1],
  //                            kBatchSize * kOutputSize * sizeof(float),
  //                            cudaMemcpyDeviceToHost, stream));

  cudaStreamSynchronize(stream);
}

int trt_detector::trt_number_detector::detect(
    std::vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
    cv::Mat &src) {
  for (trt_detector::trt_yolo_detector::Object &detected_armor :
       detected_armors) {
    detected_armor.armor_id = 0;
    cv::Mat image = detected_armor.number_image;
    cv::resize(image, image, cv::Size(kInputW, kInputH));
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    cv::Mat normalize_mat;
    cv::normalize(image, normalize_mat, 0, 1, cv::NORM_MINMAX, CV_32FC1);
    auto pointer = normalize_mat.ptr();
    memcpy(gpu_buffers[0], pointer, kInputW * kInputH * sizeof(float));
    infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer);
    int max = 0;
    float prob = 0.0;
    for (int i = 0; i < 9; i++) {
      if (cpu_output_buffer[i] > prob) {
        prob = cpu_output_buffer[i];
        max = i;
      }
    }
    if(max == 8)
    {
      detected_armor.armor_id = 0;
      continue;
    }

    // cv::Mat rect, dst, photo;
    // // rect = armor_rect;
    // cvtColor(armor_rect, dst, cv::COLOR_BGR2GRAY);
    // // TODO: 二值化参数
    // cv::threshold(dst, dst, 180, 255, cv::THRESH_BINARY);
    // bitwise_and(armor_rect, armor_rect, photo, dst);

    cv::Scalar area_sum = sum(src(detected_armor.rect));

    std::cout << max << std::endl;
    if ((area_sum[2] >= area_sum[0])) {
      max += 8;
    }
    detected_armor.armor_id = max + 1;
  }
  return 0;
}