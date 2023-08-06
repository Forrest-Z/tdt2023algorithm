/**
* @discrption:
* @version: 2.0
* @author:李子健 qq2293253590
*
 */

#ifndef TDTVISION_RM2021_OPENVINO_NUMBER_DETECTOR_H
#define TDTVISION_RM2021_OPENVINO_NUMBER_DETECTOR_H
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

#include "Yolov5faceDetector.h"
// #include "tdtcommon.h"
namespace tdtml {

    class openvino_number_detector {
    public:
        openvino_number_detector();

        void init(std::string xml_path , int use_gpu);

        int detect(vector<tdtml::YoloDetector::Object> & detected_armors,cv::Mat &src);

        int img_width=20;
        int img_height=28;
        int gpu_mode=0;

        ov::Core core;
        ov::CompiledModel compiled_model;
        ov::InferRequest curr_request;
        ov::Tensor output_tensor1;
        ov::Tensor output_tensor2;
        int count_out;
    };

}


#endif //TDTVISION_RM2021_OPENVINO_NUMBER_DETECTOR_H
