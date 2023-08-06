#ifndef REGISTRATION_NDT_CUDA_REGISTRATION_HPP_
#define REGISTRATION_NDT_CUDA_REGISTRATION_HPP_

#include "../registration_interface.hpp"
#include "cudaNDT.h"
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <fstream>
#include <chrono>

namespace robot_localization
{

  class NDTCUDARegistration : public RegistrationInterface// 继承点云配准的基类
  { 
  public:
    NDTCUDARegistration(const YAML::Node &node) :input_target_pc( new pcl::PointCloud<pcl::PointXYZ>())
    {   
        input_target_pc.reset( new pcl::PointCloud<pcl::PointXYZ>());
        NDTCUDARegistration::resolution = node["res"].as<float>();
        NDTCUDARegistration::step_size = node["step_size"].as<float>();
        NDTCUDARegistration::transformation_epsilon = node["trans_eps"].as<float>();
        NDTCUDARegistration::maximum_iterations = node["max_iter"].as<int>();
    }

    inline bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override;
    

    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override
    {
      ;
    }
    float GetFitnessScore() override
    {
      ;
    }
    inline bool ScanMatch(
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                    const Eigen::Matrix4f &guess_pose,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                    Eigen::Matrix4f &result_pose,
                    float &fitness_score
                   ) override;  
    inline double calculateFitneeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr P,
        pcl::PointCloud<pcl::PointXYZ>::Ptr Q,
        Eigen::Matrix4f transformation_matrix);
        
  public:
    float resolution = 1.0;//设置NDT算法中用于计算高斯核的分辨率
    int maximum_iterations = 72;//设置NDT算法的最大迭代次数
    float transformation_epsilon = 0.01;//NDT算法中的变换矩阵阈值 (迭代过程中，当变换矩阵的变化量小于该阈值时，算法停止迭代）
    float step_size = 0.1;//设置NDT算法中的步长大小
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_target_pc;
    // inline bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);
    
  };

  bool NDTCUDARegistration::SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target)
  { 
    // input_target_pc.reset (new pcl::PointCloud<pcl::PointXYZ>(*input_target));
    pcl::copyPointCloud(*input_target, *input_target_pc);
    // input_target_pc = input_target;

    return true;
  }

  bool NDTCUDARegistration::ScanMatch(
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                                      const Eigen::Matrix4f &guess_pose,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                                      Eigen::Matrix4f &result_pose,
                                      float &fitness_score
                                     )
  {

    int nP = input_source->size();
    int nQ = input_target_pc->size();

    float *nPdata = (float *)input_source->points.data();
    float *nQdata = (float *)input_target_pc->points.data();

    Eigen::Matrix4f matrix_trans = Eigen::Matrix4f::Identity();

    // Eigen::Matrix4d guess_pose_double = guess_pose;
    // Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = guess_pose; 
    // init_guess = matrix_trans;//guess_pose.cast<float>();
    printf("guess_pose Rotation matrix :\n");
    printf("    | %f %f %f | \n", init_guess(0, 0), guess_pose(0, 1), guess_pose(0, 2));
    printf("R = | %f %f %f | \n", init_guess(1, 0), guess_pose(1, 1), guess_pose(1, 2));
    printf("    | %f %f %f | \n", guess_pose(2, 0), guess_pose(2, 1), guess_pose(2, 2));
    printf("guess_pose Translation vector :\n");
    printf("t = < %f, %f, %f >\n\n", guess_pose(0, 3), guess_pose(1, 3), guess_pose(2, 3));
    std::cout << "------------checking CUDA NDT(GPU)---------------- "<< std::endl;
    /************************************************/
    cudaStream_t stream = NULL;
    cudaStreamCreate ( &stream );

    float *PUVM = NULL;
    cudaMallocManaged(&PUVM, sizeof(float) * 4 * nP, cudaMemAttachHost);
    cudaStreamAttachMemAsync (stream, PUVM );
    cudaMemcpyAsync(PUVM, nPdata, sizeof(float) * 4 * nP, cudaMemcpyHostToDevice, stream);

    float *QUVM = NULL;
    cudaMallocManaged(&QUVM, sizeof(float) * 4 * nQ, cudaMemAttachHost);
    cudaStreamAttachMemAsync (stream, QUVM );
    cudaMemcpyAsync(QUVM, nQdata, sizeof(float) * 4 * nQ, cudaMemcpyHostToDevice, stream);

    float *guess = NULL;
    cudaMallocManaged(&guess, sizeof(float) * 4 * 4, cudaMemAttachHost);
    cudaStreamAttachMemAsync (stream, guess);
    cudaMemcpyAsync(guess, init_guess.data(), sizeof(float) * 4 * 4, cudaMemcpyHostToDevice, stream);

    float *cudaMatrix = NULL;
    cudaMallocManaged(&cudaMatrix, sizeof(float) * 4 * 4, cudaMemAttachHost);
    cudaStreamAttachMemAsync (stream, cudaMatrix);
    cudaMemsetAsync(cudaMatrix, 0, sizeof(float) * 4 * 4, stream);

    cudaStreamSynchronize(stream);

    cudaNDT ndtTest(nP, nQ, stream);

    ndtTest.setInputSource ((void*)PUVM);
    ndtTest.setInputTarget ((void*)QUVM);
    ndtTest.setResolution (1.0);  //设置NDT算法中用于计算高斯核的分辨率
    ndtTest.setMaximumIterations (35);  //设置NDT算法的最大迭代次数
    ndtTest.setTransformationEpsilon (0.01);  //设置NDT算法中的变换矩阵阈值 (即在迭代过程中，当变换矩阵的变化量小于该阈值时，算法停止迭代）
    ndtTest.setStepSize (0.1);  //设置NDT算法中的步长大小


    ndtTest.ndt((float*)PUVM, nP, (float*)QUVM, nQ, guess, cudaMatrix, stream);
    //调用ndtTest.ndt()函数时，将初始猜测矩阵init_guess作为输入参数传递，然后在函数内部计算出匹配的变换矩阵，将其保存在cudaMatrix



    // std::cout << "CUDA NDT by Time: " << time_span.count() << " ms."<< std::endl;
    cudaStreamDestroy(stream);
    /************************************************/
    memcpy(matrix_trans.data(), cudaMatrix, sizeof(float)*4*4);//通过memcpy函数将cudaMatrix中的值复制到matrix_trans
    // transformation_matrix = matrix_trans;//最后将matrix_trans赋值给传递进来的transformation_matrix
    result_pose = matrix_trans;//.cast<double>();


    // std::cout << "CUDA NDT fitness_score: " << calculateFitneeScore (input_source, input_target_pc, matrix_trans) << std::endl;

    printf("Rotation matrix :\n");
    printf("    | %f %f %f | \n", matrix_trans(0, 0), matrix_trans(0, 1), matrix_trans(0, 2));
    printf("R = | %f %f %f | \n", matrix_trans(1, 0), matrix_trans(1, 1), matrix_trans(1, 2));
    printf("    | %f %f %f | \n", matrix_trans(2, 0), matrix_trans(2, 1), matrix_trans(2, 2));
    printf("Translation vector :\n");
    printf("t = < %f, %f, %f >\n\n", matrix_trans(0, 3), matrix_trans(1, 3), matrix_trans(2, 3));
    // printf("guess_pose Rotation matrix :\n");
    // printf("    | %f %f %f | \n", init_guess(0, 0), guess_pose(0, 1), guess_pose(0, 2));
    // printf("R = | %f %f %f | \n", init_guess(1, 0), guess_pose(1, 1), guess_pose(1, 2));
    // printf("    | %f %f %f | \n", guess_pose(2, 0), guess_pose(2, 1), guess_pose(2, 2));
    // printf("guess_pose Translation vector :\n");
    // printf("t = < %f, %f, %f >\n\n", guess_pose(0, 3), guess_pose(1, 3), guess_pose(2, 3));

    cudaFree(PUVM);
    cudaFree(QUVM);
    cudaFree(cudaMatrix);
    cudaFree(guess);

    
    
    return true;
  }

  double NDTCUDARegistration::calculateFitneeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr P,
        pcl::PointCloud<pcl::PointXYZ>::Ptr Q,
        Eigen::Matrix4f transformation_matrix)
  {
    double fitness_score = 0.0;
    pcl::PointCloud<pcl::PointXYZ> input_transformed;
    pcl::transformPointCloud (*P, input_transformed, transformation_matrix);

    pcl::search::KdTree<pcl::PointXYZ> tree_;
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    tree_.setInputCloud(Q);
    int nr = 0;
    for (std::size_t i = 0; i < input_transformed.points.size (); ++i)
    {
      // Find its nearest neighbor in the target
      tree_.nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
      if (nn_dists[0] <=  std::numeric_limits<double>::max ())
      {
        // Add to the fitness score
        fitness_score += nn_dists[0];
        nr++;
      }
    }
    if (nr > 0)
      return (fitness_score / nr);
    return (std::numeric_limits<double>::max ());
  }

} // namespace  robot_localization

#endif