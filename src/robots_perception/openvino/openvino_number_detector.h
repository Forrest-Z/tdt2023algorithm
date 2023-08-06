//
// Created by tdt on 22-7-13.
//

#ifndef _OPENVINO_NUMBER_DETECTOR_H_
#define _OPENVINO_NUMBER_DETECTOR_H_
#include <iostream>
#include <opencv2/opencv.hpp>
#include <inference_engine.hpp>
#include <iostream>
#include <chrono>
#include <opencv2/dnn/dnn.hpp>
#include <cmath>
#include <unistd.h>
namespace robots_perception {

    class openvino_number_detector {
    public:
        openvino_number_detector();

        void init(std::string xml_path , int use_gpu);

        int detect(cv::Mat image);

        int img_width=32;
        int img_height=32;


         InferenceEngine::CNNNetwork  network;
         InferenceEngine::ExecutableNetwork executable_network;
    };

}


#endif //TDTVISION_RM2021_OPENVINO_NUMBER_DETECTOR_H
