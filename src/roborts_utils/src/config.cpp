#include "config.h"
#include <opencv2/opencv.hpp>
#include <string>

// 默认全为false
int tdtconfig::CAMERA;
int tdtconfig::VIDEODEBUG;
int tdtconfig::CALIBRATE;
int tdtconfig::RECORDER;
int tdtconfig::LOG;
int tdtconfig::MANUAL;
int tdtconfig::INDEX;
int tdtconfig::BACKTREACK;
int tdtconfig::USART;
// bool tdtconfig::O3ENABLE=false;

void tdtconfig::Init() {
    cv::FileStorage fs;
    fs.open("./config/run_config.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "run_config.yaml不存在或者打不开" << std::endl;
        exit(-1);
    }
    Read(fs, "CAMERA", CAMERA);
    Read(fs, "VIDEODEBUG", VIDEODEBUG);
    //    Read(fs, "CALIBRATE", CALIBRATE);      //calibrate参数只有标定中用到，不用从文件中读取
    Read(fs, "RECORDER", RECORDER);
    Read(fs, "LOG", LOG);
    Read(fs, "INDEX", INDEX);
    Read(fs, "BACKTREACK", BACKTREACK);
    Read(fs, "USART", USART);
    Read(fs, "MANUAL", MANUAL);

    fs.release();
    // Read(fs,"O3ENABLE",O3ENABLE);
}

void tdtconfig::Read(const cv::FileStorage &fs, const std::string node, int &ok) {

    if (fs[node].empty()) {
        std::cout << "[TDT_FATAL] run_config.yaml中没有" << node << "的定义" << std::endl;
        exit(-1);
    } else {
        fs[node] >> ok;

        // std::cout<<"config: "<<node<<ok<<(ok?"   true":"  false")<<std::endl;
    }
}