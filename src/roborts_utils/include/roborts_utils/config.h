#ifndef TDT_CONFIG_H
#define TDT_CONFIG_H
#include <opencv2/opencv.hpp>
class tdtconfig {
public:
    static void Init();

public:
    //! 以下变量的名字全部是大写，这样一看就知道是宏
    // 按照run_config.yaml里面的顺序定义，这样就很容易修改了
    static int CAMERA;
    static int VIDEODEBUG;
    static int CALIBRATE;
    static int RECORDER;
    static int LOG;
    static int INDEX;
    static int BACKTREACK;
    static int USART;
    static int MANUAL;
    // static bool O3ENABLE;
private:
    static void Read(const cv::FileStorage &fs, const std::string node, int &ok);
};

#endif // TDT_CONFIG_H