/**
* @discrption:
* @version: 2.0
* @author:李子健 qq2293253590
*
 */





#include "Yolov5faceDetector.h"
#include <unistd.h>
#define IMG_LEN 480
#define IMG_LEN_F  640.0f

using namespace tdtml;

YoloDetector::YoloDetector(){}

YoloDetector::~YoloDetector(){}

//注意此处的阈值是框和物体prob乘积的阈值


bool YoloDetector::parse_yolov5(float* output_blob,float cof_threshold,
                  vector<Rect>& armors_rect,
                  vector<float>& armors_rect_cof,
                  vector<vector<float>>& armors_points,
                  vector<Rect>& robots_rect,
                  vector<float>& robots_rect_cof,
                  vector<vector<float>>& robots_points
                  ){



    //25200 if 640
    //14175 if 480
    //0~3是xywh
    //4是置信度
    //5~6是各类置信度
    //7~14是特征点xy

    for (int i =0 ;i<14175*15;i+=15){


        double box_prob = output_blob[i+ 4];

        //框置信度不满足则整体置信度不满足
        if(box_prob < cof_threshold) {
            continue;
        }


        //注意此处输出为中心点坐标,需要转化为角点坐标


        double x = output_blob[i+ 0];
        double y = output_blob[i + 1];
        double w = output_blob[i + 2];
        double h = output_blob[i + 3];



        double max_prob = 0;
        int idx=0;
        for(int t=13;t<15;++t){
            double tp= output_blob[i+ t];
            if(tp > max_prob){
                max_prob = tp;
                idx = t;
            }
        }
        idx-=8;
        float cof = box_prob * max_prob;
        //对于边框置信度小于阈值的边框,不关心其他数值,不进行计算减少计算量
        //出于精度考虑还是增加上去
        Rect rect = Rect(round(x - w/2),round(y - h/2),round(w),round(h));
        vector<float> point;
        for (int j = 5; j < 13; ++j) {
            point.push_back(output_blob[i + j]);
        }

        if(idx==5){//装甲板
            armors_points.push_back(point);
            armors_rect.push_back(rect);
            armors_rect_cof.push_back(cof);
        }
        else{//车
            robots_points.push_back(point);
            robots_rect.push_back(rect);
            robots_rect_cof.push_back(cof);
        }
    }
    return true;
}



//初始化
bool YoloDetector::init(string xml_path,double cof_threshold,double nms_area_threshold,int output_width,int output_height,int use_gpu) {
    this->gpu_mode      = use_gpu;

    _xml_path           = xml_path;
    _cof_threshold      = cof_threshold;
    _nms_area_threshold = nms_area_threshold;





    auto model = core.read_model(_xml_path);

    ov::preprocess::PrePostProcessor ppp(model);
    // 输入设置
    ov::preprocess::InputInfo       &input = ppp.input("input");
    input.tensor().set_layout("NHWC").set_element_type(ov::element::u8).set_shape({1,360,480,3}).set_color_format(ov::preprocess::ColorFormat::BGR);
    // 模型设置
    input.model().set_layout("NCHW");
    // 预处理设置
    uint16_t t[4]={0,0,60,0};
    input.preprocess().convert_layout("NCHW").convert_color(ov::preprocess::ColorFormat::RGB);

    if(this->gpu_mode==1) {
        input.preprocess().convert_element_type(ov::element::f16);
    }else{
        input.preprocess().convert_element_type(ov::element::f32);
    }

if(this->gpu_mode==1) {
        input.preprocess().scale({255.0f,255.0f,255.0f})
        .custom([=](const ov::Output<ov::Node>& node){
          return std::make_shared<ov::opset1::Pad>(node,std::make_shared<ov::opset1::Constant>(ov::element::u16,ov::Shape{4},t)->output(0),
                                                   std::make_shared<ov::opset1::Constant>(ov::element::u16,ov::Shape{4},t)->output(0),
                                                   std::make_shared<ov::opset1::Constant>(ov::element::f16,ov::Shape{},float(114.0/255.0))->output(0),
                                                   ov::op::PadMode::CONSTANT);})
        ;
    }else{
        input.preprocess().scale({255.0f,255.0f,255.0f})
        .custom([=](const ov::Output<ov::Node>& node){
          return std::make_shared<ov::opset1::Pad>(node,std::make_shared<ov::opset1::Constant>(ov::element::u16,ov::Shape{4},t)->output(0),
                                                   std::make_shared<ov::opset1::Constant>(ov::element::u16,ov::Shape{4},t)->output(0),
                                                   std::make_shared<ov::opset1::Constant>(ov::element::f32,ov::Shape{},float(114.0/255.0))->output(0),
                                                   ov::op::PadMode::CONSTANT);})
        ;
    }


    //输出设置
    ov::preprocess::OutputInfo &output = ppp.output(0);
    output.postprocess().convert_element_type(ov::element::f32);

    //
    model=ppp.build();


    if(this->gpu_mode==0||this->gpu_mode==2||this->gpu_mode==3){
        //CPU设置
        auto cpu_properties = core.get_property("CPU", ov::supported_properties);
        for (auto&& property : cpu_properties) {
            if (property != ov::supported_properties.name()) {
                if(property.is_mutable()){
                    if(property=="AFFINITY"){core.set_property("CPU",ov::affinity(ov::Affinity::HYBRID_AWARE));continue;}
                    if(property=="INFERENCE_NUM_THREADS"){core.set_property("CPU",ov::inference_num_threads(16));continue;}
                    if(property=="INFERENCE_PRECISION_HINT"){core.set_property("CPU",ov::hint::inference_precision(ov::element::f32));continue;}
                    if(property=="PERFORMANCE_HINT"){core.set_property("CPU",ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));continue;}
                    if(property=="NUM_STREAMS"){core.set_property("CPU",ov::streams::num(ov::streams::NUMA));continue;}
                    if(property=="PERF_COUNT"){core.set_property("CPU",ov::enable_profiling(false));continue;}
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



    if(this->gpu_mode==1||this->gpu_mode==2||this->gpu_mode==3) {
        // GPU设置
        auto gpu_properties = core.get_property("GPU", ov::supported_properties);
        for (auto &&property : gpu_properties) {
            if (property != ov::supported_properties.name()) {
                if (property.is_mutable()) {
                    if(property == "INFERENCE_PRECISION_HINT") {core.set_property("GPU", ov::hint::inference_precision(ov::element::f32));continue;}
                    if(property == "PERFORMANCE_HINT") {core.set_property("GPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));continue;}
                    if(property=="MODEL_PRIORITY"){core.set_property("GPU",ov::hint::model_priority(ov::hint::Priority::HIGH));continue;}
                    if (property == "CACHE_DIR") {core.set_property("GPU", ov::cache_dir("../../Cache_armor"));continue;}
                    if(property=="GPU_HOST_TASK_PRIORITY"){core.set_property("GPU" ,ov::intel_gpu::hint::host_task_priority(ov::hint::Priority::HIGH));continue;}
                    if(property=="GPU_QUEUE_PRIORITY"){core.set_property("GPU",ov::intel_gpu::hint::queue_priority(ov::hint::Priority::HIGH));continue;}
                    if(property=="GPU_QUEUE_THROTTLE"){core.set_property("GPU",ov::intel_gpu::hint::queue_throttle(ov::intel_gpu::hint::ThrottleLevel::HIGH));continue;}
                    if(property=="PERF_COUNT"){core.set_property("GPU",ov::enable_profiling(false));continue;}
                    if(property=="COMPILATION_NUM_THREADS"){core.set_property("GPU",ov::compilation_num_threads(16));continue;}
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
        auto supported_properties = core.get_property(device, ov::supported_properties);
        for (auto&& property : supported_properties) {
            if (property != ov::supported_properties.name()) {
                cout << "\t\t" << (property.is_mutable() ? "Mutable: " : "Immutable: ") << property << " : "
                     << flush;

                if (core.get_property(device, property).empty()) {
                    cout << "EMPTY VALUE" << endl;
                } else {
                    std::string stringValue=core.get_property(device, property).as<std::string>();
                    cout << (stringValue.empty() ? "\"\"" : stringValue) << endl;
                }
            }
        }

        cout << endl;
    }
    */


    //     获取可执行网络
    if (this->gpu_mode == 0) {//CPU模式
        compiled_model=core.compile_model(model,"CPU");
    }
    else if(this->gpu_mode==1){//GPU模式
        compiled_model=core.compile_model(model,"GPU",ov::hint::inference_precision(ov::element::f16));
    }
    else if (this->gpu_mode == 2) {
        compiled_model=core.compile_model(model,"AUTO:GPU,CPU",ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
        auto supported_properties = compiled_model.get_property( ov::hint::performance_mode);
        cout << supported_properties<< endl;
    }
    else if (this->gpu_mode == 3) {//异构模式,待完善，优化耗时估计较长

        /*//可手动设置操作或子图使用的设备，例子如下
         * {
            // This example demonstrates how to perform default affinity initialization and then
            // correct affinity manually for some layers
            const std::string device = "HETERO:GPU,CPU";

            // query_model result contains mapping of supported operations to devices
            auto supported_ops = core.query_model(model, device);

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
        compiled_model=core.compile_model(model,"HETERO:GPU,CPU",ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
        auto supported_properties = compiled_model.get_property( ov::hint::performance_mode);
        cout << supported_properties<< endl;
        /*
         * 可以设置环境变量OPENVINO_HETERO_VISUALIZE=1获得操作和图的注释，来针对性设置子图和操作所用设备,
         * 注释文件位于cmake-build-debug/bin/中,命令行使用 xdot *.dot读取
         */
    }


    infer_request          = compiled_model.create_infer_request();
    input_tensor           = infer_request.get_input_tensor(0);
    output_tensor          = infer_request.get_output_tensor(0);
    return true;
}
//释放资源
bool YoloDetector::uninit(){
    return true;
}

void YoloDetector::detet2origin(const Rect& dete_rect,Rect &out_rect,const vector<float> &points,vector<float> &out_points,float rate_to,int top,int left){
    //detect坐标转换到内部纯图坐标
    int inside_x = dete_rect.x - left;
    int inside_y = dete_rect.y - top;
    int ox = round(float(inside_x)/rate_to);
    int oy = round(float(inside_y)/rate_to);
    int ow = round(float(dete_rect.width)/rate_to);
    int oh =  round(float(dete_rect.height)/rate_to);
    out_rect = Rect (ox,oy,ow,oh);
    for (int i = 0; i < points.size(); i=i+2) {
        float  x=(points[i]-left)/rate_to;
        float  y=(points[i+1]-top)/rate_to;
        out_points.push_back(x);
        out_points.push_back(y);

    }
}

//处理图像获取结果
bool YoloDetector::process_frame(Mat& inframe,vector<Object>& detected_armors,vector<Object>& detected_robots){

    if(inframe.empty()){
        cout << "无效图片输入" << endl;
        return false;
    }


    //变换尺寸兼载入数据

    auto in_data = input_tensor.data<uint8_t>();
    Mat  data_mat(360, 480, CV_8UC3, in_data);
    resize(inframe, data_mat, Size(480, 360));


    infer_request.infer();




    output_tensor          = infer_request.get_output_tensor(0);
    auto out_data = output_tensor.data<float>();


    vector<Rect> armors_rect;
    vector<float> armors_rect_cof;
    vector<vector<float> > armors_points;

    vector<Rect> robots_rect;
    vector<float> robots_rect_cof;
    vector<vector<float> > robots_points;


    parse_yolov5(out_data, _cof_threshold, armors_rect, armors_rect_cof,armors_points,
                 robots_rect,robots_rect_cof,robots_points);



    vector<int> armors_final_id;
    vector<int> robots_final_id;

    dnn::NMSBoxes(armors_rect, armors_rect_cof, _cof_threshold, 0, armors_final_id);
    dnn::NMSBoxes(robots_rect, robots_rect_cof, _cof_threshold, _nms_area_threshold, robots_final_id);



    // 根据final_id获取最终结果
    to_final_objects(detected_armors,armors_final_id,armors_rect,armors_rect_cof,armors_points);
    to_final_objects(detected_robots,robots_final_id,robots_rect,robots_rect_cof,robots_points);

    return true;
}


void YoloDetector::to_final_objects(vector<Object> & final_objects,vector<int> & final_id, vector<Rect> & origin_rect,vector<float> & origin_rect_cof,vector<vector<float> > &origin_point){
    for (int i = 0; i < final_id.size(); ++i) {

        Rect          resize_rect = origin_rect[final_id[i]];
        Rect          rawrect;
        vector<float> points = origin_point[final_id[i]];
        vector<float> output_points;
        detet2origin(resize_rect, rawrect, points, output_points, r, top, left);
        final_objects.push_back(Object{origin_rect_cof[final_id[i]], rawrect, output_points});
    }
}

//以下为工具函数
double YoloDetector::sigmoid(double x){
    return (1 / (1 + exp(-x)));
}



