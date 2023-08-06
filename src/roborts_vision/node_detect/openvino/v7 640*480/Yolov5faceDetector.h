/**
* @discrption:
* @version: 2.0
* @author:李子健 qq2293253590
*
*/
#ifndef TDTVISION_RM2021_YOLOV5FACEDETECTOR_H
#define TDTVISION_RM2021_YOLOV5FACEDETECTOR_H
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <openvino/opsets/opset1.hpp>

#include <openvino/runtime/intel_cpu/properties.hpp>
#include <openvino/runtime/intel_gpu/properties.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
using namespace std;
using namespace cv;

namespace tdtml{

    class YoloDetector
    {
    public:
        typedef struct {
            float prob;
            cv::Rect rect;
            std::vector<float> points;
            int armor_id;
        } Object;
        YoloDetector();
        ~YoloDetector();
        //初始化
        //TODO 还需要添加输出图像尺寸，是否使用gpu模式，以及使用的网络参数文件，后续再添加输入尺寸
        bool init(string xml_path,double cof_threshold,double nms_area_threshold,int output_width,int output_height,int use_gpu);
        //释放资源
        bool uninit();
        void detet2origin(const Rect& dete_rect,Rect &out_rect,const vector<float> &points,vector<float> &out_points,float rate_to);

            //处理图像获取结果
        bool process_frame(Mat& inframe,vector<Object> &detected_armors,vector<Object>& detected_robots);

        void to_final_objects(vector<Object> &final_objects,vector<int> &final_id, vector<Rect> &origin_rect,vector<float> &origin_rect_cof,vector<vector<float> > &origin_point);
    private:

        double sigmoid(double x);
        vector<int> get_anchors(int net_grid);
        bool parse_yolov5(float* output_blob,float cof_threshold,
                          vector<Rect>& armors_rect,
                          vector<float>& armors_rect_cof,
                          vector<vector<float>>& armors_points,
                          vector<Rect>& robots_rect,
                          vector<float>& robots_rect_cof,
                          vector<vector<float>>& robots_points
        );



        ov::Core core;
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;
        ov::Tensor input_tensor;
        ov::Tensor output_tensor;

        //参数区
        string _xml_path;                             //OpenVINO模型xml文件路径
        double _cof_threshold;                //置信度阈值,计算方法是框置信度乘以物品种类置信度
        double _nms_area_threshold;  //nms最小重叠面积阈值
        int gpu_mode=0;
        float f=114.0f/255.0f;
        float r = 640.0f/1440.0f;
    };

}


#endif //TDTVISION_RM2021_YOLOV5FACEDETECTOR_H
