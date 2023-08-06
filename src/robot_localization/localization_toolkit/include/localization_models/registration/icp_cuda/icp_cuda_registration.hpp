#ifndef REGISTRATION_ICP_CUDA_REGISTRATION_HPP_
#define REGISTRATION_ICP_CUDA_REGISTRATION_HPP_

#include "../registration_interface.hpp"
#include "cudaICP.h"
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <fstream>
#include <chrono>

namespace robot_localization
{

  class ICPCUDARegistration : public RegistrationInterface// 继承点云配准的基类
  { 
  public:
    ICPCUDARegistration(const YAML::Node &node) :input_target_pc( new pcl::PointCloud<pcl::PointXYZ>())
    {   
        input_target_pc.reset( new pcl::PointCloud<pcl::PointXYZ>());
        ICPCUDARegistration::relative_mse = node["euc_fitness_eps"].as<float>();
        ICPCUDARegistration::threshold = node["trans_eps"].as<double>();
        ICPCUDARegistration::distance_threshold = node["max_corr_dist"].as<float>();
        ICPCUDARegistration::Maxiterate = node["max_iter"].as<int>();
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

    double threshold=1e-6;//threshold for distance Error，is TransformationEpsilon
    int Maxiterate=50;//Maximum iteration count
    double acceptrate=1.0;//accept rate
    float distance_threshold=0.5;  // max distance between source point and its closest target point
    float relative_mse=0.0001;      // icp.setEuclideanFitnessEpsilon



    pcl::PointCloud<pcl::PointXYZ>::Ptr input_target_pc;
    // inline bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);
    
  };

  bool ICPCUDARegistration::SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target)
  { 
    // input_target_pc.reset (new pcl::PointCloud<pcl::PointXYZ>(*input_target));
    pcl::copyPointCloud(*input_target, *input_target_pc);
    // input_target_pc = input_target;

    return true;
  }

  bool ICPCUDARegistration::ScanMatch(
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
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::ratio<1, 1000>> time_span =
        std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

    
    
    void *cudaMatrix = NULL;
    cudaMatrix = malloc(sizeof(float)*4*4);
    memset(cudaMatrix, 0 , sizeof(float)*4*4);
    std::cout << "------------checking CUDA ICP(GPU)---------------- "<< std::endl;
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

    cudaStreamSynchronize(stream);

    cudaICP icpTest(nP, nQ, stream);

    t1 = std::chrono::steady_clock::now();
    icpTest.icp(
                 (float*)PUVM, nP, (float*)QUVM, nQ, ICPCUDARegistration::relative_mse ,
                 ICPCUDARegistration::Maxiterate, ICPCUDARegistration::threshold, 
                 ICPCUDARegistration::distance_threshold,
		             cudaMatrix, stream
              );
    t2 = std::chrono::steady_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
    std::cout << "CUDA ICP by Time: " << time_span.count() << " ms."<< std::endl;
    cudaStreamDestroy(stream);
    /************************************************/
    Eigen::Matrix4f matrix_icp = Eigen::Matrix4f::Identity();
    memcpy(matrix_icp.data(), cudaMatrix, sizeof(float)*4*4);
    // memcpy(result_pose.data(), cudaMatrix, sizeof(double)*4*4);
    result_pose = matrix_icp;//big problem

    std::cout << "CUDA ICP fitness_score: " << calculateFitneeScore( input_source, input_target_pc, matrix_icp) << std::endl;
    std::cout << "matrix_icp calculated Matrix by Class ICP "<< std::endl;


    cudaFree(PUVM);
    cudaFree(QUVM);
    free(cudaMatrix);
    auto cloudSrc = input_source;
    auto cloudDst = input_target_pc;
    pcl::PointCloud<pcl::PointXYZ> input_transformed;
    pcl::transformPointCloud (*cloudSrc, input_transformed, matrix_icp);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSrcRGB (new pcl::PointCloud<pcl::PointXYZRGB>(cloudSrc->size(),1));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDstRGB (new pcl::PointCloud<pcl::PointXYZRGB>(cloudDst->size(),1));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudALL (new pcl::PointCloud<pcl::PointXYZRGB>(cloudSrc->size() + cloudDst->size(),1));
    // Fill in the CloudIn data
    for (int i = 0; i < cloudSrc->size(); i++)
    {
        pcl::PointXYZRGB &pointin = (*cloudSrcRGB)[i];
        pointin.x = (input_transformed)[i].x;
        pointin.y = (input_transformed)[i].y;
        pointin.z = (input_transformed)[i].z;
        pointin.r = 255;
        pointin.g = 0;
        pointin.b = 0;
        (*cloudALL)[i] = pointin;
    }
    for (int i = 0; i < cloudDst->size(); i++)
    {
        pcl::PointXYZRGB &pointout = (*cloudDstRGB)[i];
        pointout.x = (*cloudDst)[i].x;
        pointout.y = (*cloudDst)[i].y;
        pointout.z = (*cloudDst)[i].z;
        pointout.r = 0;
        pointout.g = 255;
        pointout.b = 255;
        (*cloudALL)[i+cloudSrc->size()] = pointout;
    }
    
    
    return true;
  }

  double ICPCUDARegistration::calculateFitneeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr P,
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