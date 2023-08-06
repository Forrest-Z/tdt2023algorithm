/*
 * @Description: ICP SVD lidar odometry
 */
#ifndef ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_
#define ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_

#include "registration_interface.hpp"
// pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <vector>
#include "glog/logging.h"
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace robot_localization
{

  class ICPSVDRegistration : public RegistrationInterface
  {
  public:
    ICPSVDRegistration(const YAML::Node &node): input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
    {
        // parse params:
        float max_corr_dist = node["max_corr_dist"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    ICPSVDRegistration(float max_corr_dist,
                       float trans_eps,
                       float euc_fitness_eps,
                       int max_iter): input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
    {
        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override
    {
        input_target_ = input_target;
        input_target_kdtree_->setInputCloud(input_target_);

        return true;
    }

    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override
    {
        input_source_ = input_source;

        // pre-process input source:
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_input_source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

        // init estimation:
        transformation_.setIdentity();

        //
        // first option -- implement all computing logic on your own
        //
        // do estimation:
        int curr_iter = 0;
        while (curr_iter < max_iter_)
        { //  最大迭代次数
            
            // apply current estimation:
            pcl::PointCloud<pcl::PointXYZ>::Ptr curr_input_source(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*transformed_input_source, *curr_input_source, transformation_);

            // get correspondence:
            std::vector<Eigen::Vector3d> xs;
            std::vector<Eigen::Vector3d> ys;

            // do not have enough correspondence -- break:
            if (GetCorrespondence(curr_input_source, xs, ys) < 3) //  寻找最邻近点的点对，当匹配点少于3个退出
                break;

            // update current transform:
            Eigen::Matrix4d delta_transformation;
            GetTransform(xs, ys, delta_transformation);

            // whether the transformation update is significant:
            if (!IsSignificant(delta_transformation, trans_eps_)) // 最大旋转矩阵
                break;
            // update transformation:
            transformation_ = delta_transformation * transformation_;

            ++curr_iter;
        }

        // set output:
        result_pose = transformation_ * predict_pose;

        // 归一化
        Eigen::Quaterniond qr(result_pose.block<3, 3>(0, 0));
        qr.normalize();
        Eigen::Vector3d t = result_pose.block<3, 1>(0, 3);
        result_pose.setIdentity();
        result_pose.block<3, 3>(0, 0) = qr.toRotationMatrix();
        result_pose.block<3, 1>(0, 3) = t;
        pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

        return true;
    }

    bool ScanMatch(
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                                const Eigen::Matrix4f &guess_pose,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                                Eigen::Matrix4f &result_pose,
                                float &fitness_score
                              ) override
    {
      ;
    } 
  private:
    bool SetRegistrationParam(float max_corr_dist,
                              float trans_eps,
                              float euc_fitness_eps,
                              int max_iter)
    {
        // set params:
        max_corr_dist_ = max_corr_dist;
        trans_eps_ = trans_eps;
        euc_fitness_eps_ = euc_fitness_eps;
        max_iter_ = max_iter;

        LOG(INFO) << "ICP SVD params:" << std::endl
                  << "max_corr_dist: " << max_corr_dist_ << ", "
                  << "trans_eps: " << trans_eps_ << ", "
                  << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
                  << "max_iter: " << max_iter_
                  << std::endl
                  << std::endl;

        return true;
    }

  private:
    size_t GetCorrespondence(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                             std::vector<Eigen::Vector3d> &xs,
                             std::vector<Eigen::Vector3d> &ys)
    {
        const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

        size_t num_corr = 0;

        //  set up point correspondence
        for (size_t i = 0; i < input_source->points.size(); ++i)
        {
            std::vector<int> corr_ind;      // index
            std::vector<float> corr_sq_dis; // correspondence_square_dis
            input_target_kdtree_->nearestKSearch(
                input_source->at(i),
                1,
                corr_ind, corr_sq_dis); // kdtree  搜索

            if (corr_sq_dis.at(0) > MAX_CORR_DIST_SQR)
                continue;

            // add  correspondence:
            Eigen::Vector3d x(
                input_target_->at(corr_ind.at(0)).x,
                input_target_->at(corr_ind.at(0)).y,
                input_target_->at(corr_ind.at(0)).z);
            Eigen::Vector3d y(
                input_source->at(i).x,
                input_source->at(i).y,
                input_source->at(i).z);
            xs.push_back(x);
            ys.push_back(y);

            ++num_corr;
        }

        return num_corr;
    }

    void GetTransform(const std::vector<Eigen::Vector3d> &xs,
                      const std::vector<Eigen::Vector3d> &ys,
                      Eigen::Matrix4d &transformation_)
    {
        const size_t N = xs.size();

        // find centroids of mu_x and mu_y:
        Eigen::Vector3d mu_x = Eigen::Vector3d::Zero();
        Eigen::Vector3d mu_y = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < N; ++i)
        {
            mu_x += xs.at(i);
            mu_y += ys.at(i);
        }
        mu_x /= N;
        mu_y /= N;

        // build H:
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < N; ++i)
        {
            H += (ys.at(i) - mu_y) * (xs.at(i) - mu_x).transpose();
        }

        // solve R:
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

        // solve t:
        Eigen::Vector3d t = mu_x - R * mu_y;

        // set output:
        transformation_.setIdentity();
        transformation_.block<3, 3>(0, 0) = R;
        transformation_.block<3, 1>(0, 3) = t;
    }

    bool IsSignificant(const Eigen::Matrix4d &transformation,
                       const float trans_eps)
    {
        // a. translation magnitude -- norm:
        float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
        // b. rotation magnitude -- angle:
        float rotation_magnitude = fabs(
            acos(
                (transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f));

        return (
            (translation_magnitude > trans_eps) ||
            (rotation_magnitude > trans_eps));
    }

    float max_corr_dist_;
    float trans_eps_;
    float euc_fitness_eps_;
    int max_iter_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_target_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr input_target_kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_source_;

    Eigen::Matrix4d transformation_;
  };

}

#endif