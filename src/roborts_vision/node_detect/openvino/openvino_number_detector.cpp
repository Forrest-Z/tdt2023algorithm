/**
 * @discrption:
 * @version: 2.0
 * @author:李子健 qq2293253590
 *
 */

#include "openvino_number_detector.h"

tdtml::openvino_number_detector::openvino_number_detector() = default;

void tdtml::openvino_number_detector::init(String xml_path, int use_gpu) {
  std::cout << "openvino numdetector init " << xml_path << std::endl;

  this->gpu_mode = use_gpu;

  auto model = core.read_model(xml_path);

  ov::preprocess::PrePostProcessor ppp(model);
  ov::preprocess::InputInfo &input = ppp.input(0);
  input.tensor()
      .set_layout("NHWC")
      .set_element_type(ov::element::u8)
      .set_shape({1, img_height, img_width, 1});
  input.model().set_layout("NCHW");
  input.preprocess().convert_layout("NCHW");
  if (this->gpu_mode == 1) {
    input.preprocess().convert_element_type(ov::element::f16);
  } else {
    input.preprocess().convert_element_type(ov::element::f32);
  }

  input.preprocess().scale({255.0f});
  ov::preprocess::OutputInfo &output = ppp.output(0);
  output.postprocess().convert_element_type(ov::element::f32);
  model = ppp.build();

  if (this->gpu_mode == 0 || this->gpu_mode == 2 || this->gpu_mode == 3) {
    // CPU设置
    auto cpu_properties = core.get_property("CPU", ov::supported_properties);
    for (auto &&property : cpu_properties) {
      if (property != ov::supported_properties.name()) {
        if (property.is_mutable()) {
          if (property == "AFFINITY") {
            core.set_property("CPU", ov::affinity(ov::Affinity::NUMA));
            continue;
          }
          if (property == "INFERENCE_NUM_THREADS") {
            core.set_property("CPU", ov::inference_num_threads(16));
            continue;
          }
          if (property == "INFERENCE_PRECISION_HINT") {
            core.set_property("CPU",
                              ov::hint::inference_precision(ov::element::f32));
            continue;
          }
          if (property == "PERFORMANCE_HINT") {
            core.set_property("CPU", ov::hint::performance_mode(
                                         ov::hint::PerformanceMode::LATENCY));
            continue;
          }
          if (property == "NUM_STREAMS") {
            core.set_property("CPU", ov::streams::num(ov::streams::NUMA));
            continue;
          }
          if (property == "PERF_COUNT") {
            core.set_property("CPU", ov::enable_profiling(false));
            continue;
          }
        }
      }
    }
    /*CPU设置
     * ov::affinity(ov::Affinity::HYBRID_AWARE)//设置核心与线程的关系,0:禁用线程相关性:NONE;1,将线程固定到核心，最适合静态基准测试:CODE;
     *               2,将线程固定到NUMA节点，最适合实际情况:NUMA;3,让运行时锁定内核类型，例如，对于延迟任务，更喜欢“大”内核:HYBRID_AWARE
     * ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)//设置性能模式为低延迟:LATENCY;默认:UNDEFINED
     * ov::hint::inference_precision(ov::element::f32)//设置推理精度，cpu只能是f32
     * ov::enable_profiling(false)//允许解析数据,耗时较长,不知道有什么用,设置为false关掉
     * ov::streams::num(12)//设置推理实例并发数为5个
     * ov::streams::num(ov::steams::AUTO)//创建最少的流以提高性能
     * ov::streams::num(ov::streams::NUMA)//推理实例数按计算资源平均分配
     * ov::inference_num_threads(16)//设置推理实例的线程并发数为16个,设置太满同时有大型程序允许会卡顿,会卡得厉害,
     * ov::intel_cpu::denormals_optimization(true)//非正规优化，耗时较长，有些情况能加速，大部分情况效果不明显
     */
  }

  if (this->gpu_mode == 2 || this->gpu_mode == 3) {
    // GPU设置
    auto gpu_properties = core.get_property("GPU", ov::supported_properties);
    for (auto &&property : gpu_properties) {
      if (property != ov::supported_properties.name()) {
        if (property.is_mutable()) {
          if (property == "INFERENCE_PRECISION_HINT") {
            core.set_property("GPU",
                              ov::hint::inference_precision(ov::element::f32));
            continue;
          }
          if (property == "PERFORMANCE_HINT") {
            core.set_property("GPU", ov::hint::performance_mode(
                                         ov::hint::PerformanceMode::LATENCY));
            continue;
          }
          if (property == "MODEL_PRIORITY") {
            core.set_property(
                "GPU", ov::hint::model_priority(ov::hint::Priority::HIGH));
            continue;
          }
          if (property == "CACHE_DIR") {
            core.set_property("GPU", ov::cache_dir("../../Cache_armor"));
            continue;
          }
          if (property == "GPU_HOST_TASK_PRIORITY") {
            core.set_property("GPU", ov::intel_gpu::hint::host_task_priority(
                                         ov::hint::Priority::HIGH));
            continue;
          }
          if (property == "GPU_QUEUE_PRIORITY") {
            core.set_property("GPU", ov::intel_gpu::hint::queue_priority(
                                         ov::hint::Priority::HIGH));
            continue;
          }
          if (property == "GPU_QUEUE_THROTTLE") {
            core.set_property("GPU",
                              ov::intel_gpu::hint::queue_throttle(
                                  ov::intel_gpu::hint::ThrottleLevel::HIGH));
            continue;
          }
          if (property == "PERF_COUNT") {
            core.set_property("GPU", ov::enable_profiling(false));
            continue;
          }
          if (property == "COMPILATION_NUM_THREADS") {
            core.set_property("GPU", ov::compilation_num_threads(16));
            continue;
          }
        }
      }
    }
    /*GPU模式设置
     * ,ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)//设置性能模式为低延迟:LATENCY;默认:UNDEFINED
     * ,ov::hint::model_priority(ov::hint::Priority::MEDIUM)//设置模型优先级为高:HIGH;中:MEDIUM(默认);低:LOW
     * ,ov::hint::inference_precision(ov::element::f32)//设置推理精度，cpu只能是f32
     * ,ov::enable_profiling(true))//允许解析数据,耗时较长,不知道有什么用
     * ,ov::compilation_num_threads(16)//设置编译实例的线并发数为16
     * ,ov::cache_dir("../../Cache")//设置缓存路径
     * ,ov::intel_gpu::hint::host_task_priority(ov::hint::Priority::HIGH)//设置GPU插件使用的TBB关联的cpu核心类型为大核(如果可用):HIGH;任意可用核心:MEDIUM(默认);小核(如果可用):LOW
     * ,ov::intel_gpu::hint::queue_priority(ov::hint::Priority::HIGH)//设置OpenCL队列优先级为高:HIGH;中:MEDIUM(默认);低:LOW
     * ,ov::intel_gpu::hint::queue_throttle(ov::intel_gpu::hint::ThrottleLevel::HIGH)//设置OpenCL节流阀，控制的能耗为高:HIGH;中:MEDIUM(默认);低:LOW
     */
  }
  /*
  cout<<"设备参数"<<endl;
  std::vector<std::string> availableDevices = core.get_available_devices();
  for (auto&& device : availableDevices) {
      cout << device << endl;

      // Query supported properties and print all of them
      cout<< "\tSUPPORTED_PROPERTIES: " << endl;
      auto supported_properties = core.get_property(device,
  ov::supported_properties); for (auto&& property : supported_properties) { if
  (property != ov::supported_properties.name()) { cout << "\t\t" <<
  (property.is_mutable() ? "Mutable: " : "Immutable: ") << property << " : "
                   << flush;

              if (core.get_property(device, property).empty()) {
                  cout << "EMPTY VALUE" << endl;
              } else {
                  std::string stringValue=core.get_property(device,
  property).as<std::string>(); cout << (stringValue.empty() ? "\"\"" :
  stringValue) << endl;
              }
          }
      }

      cout << endl;
  }
  */

  //     获取可执行网络
  if (this->gpu_mode == 0) {  // CPU模式
    compiled_model = core.compile_model(
        model, "CPU",
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  } else if (this->gpu_mode == 1) {  // GPU模式
    // compiled_model=core.compile_model(model,"GPU",ov::hint::inference_precision(ov::element::f16),ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

    compiled_model = core.compile_model(
        model, "CPU",
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  } else if (this->gpu_mode == 2) {
    compiled_model = core.compile_model(
        model, "AUTO:GPU,CPU",
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

  } else if (this->gpu_mode == 3) {  // 异构模式,待完善，优化耗时估计较长

    /*//可手动设置操作或子图使用的设备，例子如下
     * {
        // This example demonstrates how to perform default affinity
     initialization and then
        // correct affinity manually for some layers
        const std::string device = "HETERO:GPU,CPU";

        // query_model result contains mapping of supported operations to
     devices auto supported_ops = core.query_model(model, device);

        // update default affinities manually for specific operations
        supported_ops["operation_name"] = "CPU";

        // set affinities to a model
        for (auto &&node : model->get_ops()) {
         auto &affinity = supported_ops[node->get_friendly_name()];
         // Store affinity mapping using op runtime information
         node->get_rt_info()["affinity"] = affinity;
        }

        // load model with manually set affinities
        auto compiled_model = core.compile_model(model, device);
        }
     */
    compiled_model = core.compile_model(
        model, "HETERO:GPU,CPU",
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

    /*
     * 可以设置环境变量OPENVINO_HETERO_VISUALIZE=1获得操作和图的注释，来针对性设置子图和操作所用设备,
     * 注释文件位于cmake-build-debug/bin/中,命令行使用 xdot *.dot读取
     */
  }

  curr_request = compiled_model.create_infer_request();
  output_tensor1 = ov::Tensor(ov::element::f32,
                              curr_request.get_output_tensor().get_shape());
  output_tensor2 = ov::Tensor(ov::element::f32,
                              curr_request.get_output_tensor().get_shape());
}

int tdtml::openvino_number_detector::detect(
    vector<tdtml::YoloDetector::Object> &detected_armors, cv::Mat &src) {
  if (detected_armors.empty()) return 0;
  int ifAsyncInfer = 1;
  if (detected_armors.size() == 1 ||
      (ifAsyncInfer == 0 && detected_armors.size() > 1)) {
    for (tdtml::YoloDetector::Object &detected_armor : detected_armors) {
      cv::Mat image = detected_armor.number_image;
      // image = src(detected_armor.rect);

      resize(image, image, Size(img_width, img_height));

      ov::Tensor curr_input_tensor = curr_request.get_input_tensor();
      auto in_data = curr_input_tensor.data<uint8_t>();
      Mat data_mat(img_height, img_width, CV_8UC1, in_data);
      cv::cvtColor(image, data_mat, cv::COLOR_BGR2GRAY);
      // cv::imshow("image", data_mat);
      // cv::waitKey(1);
      curr_request.infer();

      int max = 0;
      float prob = 0.0;
      auto out_data = curr_request.get_output_tensor(0).data<float>();
      for (int i = 0; i < 9; i++) {
        if (out_data[i] > prob) {
          prob = out_data[i];
          max = i;
        }
      }
      if (max == 8) {
        detected_armor.armor_id = 0;
        continue;
      }

      cv::Point div_point(10, 10);
      for (int i = 0; i < 8; i++) {
        if (i % 2 == 0) {
          if (detected_armor.points[i] < 1) detected_armor.points[i] = 1;
          if (detected_armor.points[i] > 1439) detected_armor.points[i] = 1439;
        } else {
          if (detected_armor.points[i] > 1079) detected_armor.points[i] = 1079;
          if (detected_armor.points[i] < 1) detected_armor.points[i] = 1;
        }
      }

      cv::Scalar area_sum = sum(src(detected_armor.rect));

      if ((area_sum[2] >= area_sum[0])) {
        max += 8;
      }

      detected_armor.armor_id = max + 1;
    }
  } else if (detected_armors.size() > 1 && ifAsyncInfer == 1) {
    int last = detected_armors.size() - 1;

    cv::Mat image = detected_armors[0].number_image;
    // image = src(detected_armors[0].rect);
    resize(image, image, Size(img_width, img_height));
    ov::Tensor curr_input_tensor(
        ov::element::u8,
        ov::Shape{1, (unsigned long)img_height, (unsigned long)img_width, 1});
    auto in_data = curr_input_tensor.data<uint8_t>();
    Mat data_mat(img_height, img_width, CV_8UC1, in_data);
    cv::cvtColor(image, data_mat, cv::COLOR_BGR2GRAY);

    curr_request.set_input_tensor(curr_input_tensor);
    curr_request.set_output_tensor(output_tensor1);
    count_out = 1;
    curr_request.start_async();

    for (int i = 0; i < last; i++) {
      cv::Mat next_image = detected_armors[i + 1].number_image;
      // next_image = src(detected_armors[i + 1].rect);
      resize(next_image, next_image, Size(img_width, img_height));
      ov::Tensor next_input_tensor(
          ov::element::u8,
          ov::Shape{1, (unsigned long)img_height, (unsigned long)img_width, 1});
      auto next_in_data = next_input_tensor.data<uint8_t>();
      Mat next_data_mat(img_height, img_width, CV_8UC1, next_in_data);
      cv::cvtColor(next_image, next_data_mat, cv::COLOR_BGR2GRAY);
      curr_request.wait();
      curr_request.set_input_tensor(next_input_tensor);

      float *out_data;
      if (count_out == 1) {
        out_data = output_tensor1.data<float>();
        curr_request.set_output_tensor(output_tensor2);
        count_out = 2;
      } else if (count_out == 2) {
        out_data = output_tensor2.data<float>();
        curr_request.set_output_tensor(output_tensor1);
        count_out = 1;
      }
      curr_request.start_async();

      int max = 0;
      float prob = 0.0;

      for (int k = 0; k < 9; k++) {
        if (out_data[k] > prob) {
          prob = out_data[k];
          max = k;
        }
      }
      if (max == 8) {
        detected_armors[i].armor_id = 0;
        continue;
      }

      cv::Point div_point(10, 10);
      for (int k = 0; k < 8; k++) {
        if (k % 2 == 0) {
          if (detected_armors[i].points[k] < 1)
            detected_armors[i].points[k] = 1;
          if (detected_armors[i].points[k] > 1439)
            detected_armors[i].points[k] = 1439;
        } else {
          if (detected_armors[i].points[k] > 1079)
            detected_armors[i].points[k] = 1079;
          if (detected_armors[i].points[k] < 1)
            detected_armors[i].points[k] = 1;
        }
      }

      cv::Scalar area_sum = sum(src(detected_armors[i].rect));

      if ((area_sum[2] >= area_sum[0])) {
        max += 8;
      }
      detected_armors[i].armor_id = max + 1;
    }

    curr_request.wait();

    auto out_data = curr_request.get_output_tensor(0).data<float>();

    int max = 0;
    float prob = 0.0;

    for (int k = 0; k < 9; k++) {
      if (out_data[k] > prob) {
        prob = out_data[k];
        max = k;
      }
    }

    for (int k = 0; k < 8; k++) {
      if (k % 2 == 0) {
        if (detected_armors[last].points[k] < 1)
          detected_armors[last].points[k] = 1;
        if (detected_armors[last].points[k] > 1439)
          detected_armors[last].points[k] = 1439;
      } else {
        if (detected_armors[last].points[k] > 1079)
          detected_armors[last].points[k] = 1079;
        if (detected_armors[last].points[k] < 1)
          detected_armors[last].points[k] = 1;
      }
    }

    //

    cv::Scalar area_sum = sum(src(detected_armors[last].rect));
    if (max == 8) {
      detected_armors[last].armor_id = 0;
    } else {
      if ((area_sum[2] >= area_sum[0])) {
        max += 8;
      }

      detected_armors[last].armor_id = max + 1;
    }
  }
  return 1;
}
