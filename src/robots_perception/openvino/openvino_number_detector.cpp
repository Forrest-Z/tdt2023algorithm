

#include "robots_perception/openvino_number_detector.h"
using namespace cv;
using namespace InferenceEngine;

namespace robots_perception {
    openvino_number_detector::openvino_number_detector() {}

    void openvino_number_detector::init(String xml_path, int use_gpu) {

        std::cout << "openvino numdetector init " << xml_path << std::endl;

        InferenceEngine::Core core;

        network = core.ReadNetwork(xml_path);
        //    executable_network = core.LoadNetwork("/home/iiap/PycharmProjects/再次开始的deeplearning/train/test.xml", "CPU");
        if (use_gpu == 0) {
            executable_network = core.LoadNetwork(network, "CPU");
        } else {
            executable_network = core.LoadNetwork(network, "CPU");
        }
    }

    int openvino_number_detector::detect(cv::Mat image) {

        cv::resize(image, image, cv::Size(img_width, img_height));
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        InferenceEngine::InputsDataMap  input_info    = network.getInputsInfo();
        InferenceEngine::OutputsDataMap output_info   = network.getOutputsInfo();
        auto                            infer_request = executable_network.CreateInferRequest();

        /** Iterate over all input blobs **/
        for (auto &item : input_info) {

            auto input_name = item.first;
            /** Get input blob **/
            auto input = infer_request.GetBlob(input_name);
            /** Fill input tensor with planes. First b channel, then g and r channels **/
            Blob::Ptr                           frameBlob  = infer_request.GetBlob(input_name);
            InferenceEngine::LockedMemory<void> blobMapped = InferenceEngine::as<InferenceEngine::MemoryBlob>(frameBlob)->wmap();
            float                              *blob_data  = blobMapped.as<float *>();

            for (size_t row = 0; row < img_height; row++) {
                for (size_t col = 0; col < img_width; col++) {
                    for (size_t ch = 0; ch < 1; ch++) {
                        //在使用灰度图片的时候需要有特殊形式。
                        blob_data[img_height * img_width * ch + row * img_height + col] = float(image.at<uchar>(row, col)) / 255.0f;
                    }
                }
            }
        }

        auto start = std::chrono::high_resolution_clock::now();
        infer_request.Infer();
        auto                          end  = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;
        std::cout << "armor infer " << diff.count() << " s" << std::endl;

        int   max  = 0;
        float prob = 0.0;

        for (auto &output : output_info) {
            auto      output_name = output.first;
            Blob::Ptr blob        = infer_request.GetBlob(output_name);

            LockedMemory<const void> blobMapped  = as<MemoryBlob>(blob)->rmap();
            const float             *output_blob = blobMapped.as<float *>();

            for (int i = 0; i < 8; i++) {

                if (output_blob[i] > prob) {

                    prob = output_blob[i];
                    max  = i;
                }
            }
        }
        return max;
    }
} // namespace robots_perception