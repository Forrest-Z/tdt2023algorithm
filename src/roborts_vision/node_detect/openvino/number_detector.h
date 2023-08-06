/**
* @discrption:
* @version: 2.0
* @author:李子健 qq2293253590
*
 */

#ifndef TDTVision_RM2021_NUMBERDETECTOR_H
#define TDTVision_RM2021_NUMBERDETECTOR_H



#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>


#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <openvino/opsets/opset1.hpp>
#include <openvino/runtime/intel_cpu/properties.hpp>
#include <openvino/runtime/intel_gpu/properties.hpp>

// #include "toolkit/base_class.h"
// #include "toolkit/base_toolkit.h"
namespace tdtml {

class NumberDetector {
   public:
    NumberDetector();

    void init(std::string xml_path , int use_gpu);

    void Predict(std::vector<cv::Mat> &samples, std::vector<int> &flags, std::vector<float> &confs);

    int img_width=28;
    int img_height=28;
    int gpu_mode=0;

    ov::Core core;
    ov::CompiledModel compiled_model;
    ov::InferRequest curr_request;

};

}

#endif // TDTVision_RM2021_NUMBERDETECTOR_H
