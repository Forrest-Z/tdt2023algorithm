/*
 * @Description: 把加载的pcd文件稀疏后实时显示于rviz当中，提供调试的客观性
 */
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/global_path_defination/global_path.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <pcl/io/pcd_io.h>
#include "robot_localization/publisher/cloud_publisher.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/impl/filter_indices.hpp>
#include "localization_models/cloud_filter/voxel_filter.hpp"
#include <boost/iostreams/device/mapped_file.hpp>
#include <fstream>
#include <vector>
#include <thread>
#include <pcl/point_types.h>
#include "localization_tools/load_pcd_file.hpp"
#include <boost/filesystem.hpp>
#include <pcl/filters/passthrough.h>

using namespace robot_localization;

void read_points_from_file(const std::string& filename, std::vector<pcl::PointXYZI>& points, size_t start, size_t end) {
    std::ifstream input(filename, std::ios::binary | std::ios::in);
    if (!input) {
        std::cerr << "Error: Could not open " << filename << " for reading." << std::endl;
        return;
    }

    input.seekg(start * sizeof(pcl::PointXYZI));

    for (size_t i = start; i < end; ++i) {
        pcl::PointXYZI point;
        input.read(reinterpret_cast<char*>(&point.x), sizeof(float));
        input.read(reinterpret_cast<char*>(&point.y), sizeof(float));
        input.read(reinterpret_cast<char*>(&point.z), sizeof(float));
        input.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
        points.push_back(point);
    }
    input.close();
}
int main(int argc, char *argv[]) 
{   
    setlocale(LC_ALL,"");
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr ros2_node = nullptr;
    ros2_node = rclcpp::Node::make_shared("the_pcd_process_end");
    google::InitGoogleLogging(argv[0]);
    

    std::string yaml_config_path = ros2_node->declare_parameter<std::string>("yaml_config_path", WORK_PACKAGE_PATH + "/config/livox_lidar.yaml");
    short int pcd_process_symbol = ros2_node->declare_parameter<short int>("pcd_process_action", 1);

    YAML::Node user_config = YAML::LoadFile(yaml_config_path);
    std::string global_frame_id;

    global_frame_id = user_config["global_frame_id"].as<std::string>();
    // std::string pcd_map_path = WORK_PACKAGE_PATH + user_config["map_path"].as<std::string>();
    
    std::string pcd_map_path_1 = WORK_PACKAGE_PATH + "/../pcd_maps/slam_scans.pcd";
    std::string pcd_map_path_2 = WORK_PACKAGE_PATH + "/../pcd_maps/input_matching_cloud_2.pcd";

    if(pcd_process_symbol==1)
    {
        std::cout<<"pcd_process_symbol=1,执行pcd坐标对称转换分割任务\n";
        double x_transform = 0.00;///x转换
        double y_transform = 0.00;//y转换
        std::cout<<"use pcd map path:"<<pcd_map_path_1<<"\n";


    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>);
    // 创建一个输出点云
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZINormal>);

    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_map_path_1, *cloud_in) == -1)
    {
        PCL_ERROR("无法读取点云文件！\n");
        return -1;
    }

    // 定义转换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // 沿x和y方向平移
    transform.translation() << x_transform, y_transform, 0;

    // 绕yaw轴旋转180度
    float yaw = 0;// M_PI/180.00*9.58;
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

    float pitch = 0/180.00*(2.1);// M_PI/180.00*9.58;
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));

    // 执行变换
    pcl::transformPointCloudWithNormals(*cloud_in, *cloud_out, transform);

    // // 遍历点云数据，取反每个点的x和y坐标
    // for (size_t i = 0; i < cloud_out->points.size(); ++i)
    // {
    //     cloud_out->points[i].x = -cloud_out->points[i].x;
    //     cloud_out->points[i].y = -cloud_out->points[i].y;
    // }

    pcl::PassThrough<pcl::PointXYZINormal> pass;
    pass.setInputCloud (cloud_out);

    pass.setFilterFieldName ("z");
    pass.setFilterLimitsNegative (true);
    pass.setFilterLimits (-100.0,0.8 );
    pass.filter (*cloud_out);

    pass.setFilterFieldName ("x");
    pass.setFilterLimitsNegative (true);
    pass.setFilterLimits (-100.0,-3.80 );
    pass.filter (*cloud_out);

    

    // 滤波
        std::shared_ptr<CloudFilterInterface> pcd_filter_ptr_;
        pcd_filter_ptr_ = std::make_shared<VoxelFilter>(0.13, 0.13, 0.13);
        pcd_filter_ptr_->Filter(cloud_out  , cloud_out  );

    // 保存变换后的点云
    pcl::PCDWriter pcd_writer;
    std::cout << "current scan saved to /../pcd_maps/input_matching_cloud_2" <<".pcd" << std::endl;
    pcd_writer.writeBinary(pcd_map_path_2, *cloud_out);//将点云数据写入指定的文件。如果目标路径下存在同名文件，它将覆盖现有文件
    std::cout << "Saved " << pcd_map_path_2 << std::endl;

    // 发布
        rclcpp::Rate loop_rate(1);
        sensor_msgs::msg::PointCloud2 cloud_ptr_in;
        // std::string frame_id = global_frame_id;
        cloud_ptr_in.header.stamp = ros2_node->now();
        cloud_ptr_in.header.frame_id = global_frame_id;
        pcl::toROSMsg(*cloud_in, cloud_ptr_in);
        sensor_msgs::msg::PointCloud2 cloud_ptr_out;
        cloud_ptr_out.header.stamp = ros2_node->now();
        cloud_ptr_out.header.frame_id = global_frame_id;
        pcl::toROSMsg(*cloud_out, cloud_ptr_out);
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr in_pcd_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr out_pcd_pub;
        in_pcd_pub = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>("in_pcd_pub",1);
        out_pcd_pub = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>("out_pcd_pub",1);

        while (rclcpp::ok()) 
        {   
            
            in_pcd_pub->publish(cloud_ptr_in);
            out_pcd_pub->publish(cloud_ptr_out);
            rclcpp::spin_some(ros2_node);
            loop_rate.sleep();
        }

    }else 
    if(pcd_process_symbol==2)
    {   
        std::cout<<"pcd_process_symbol=2,执行大型pcd分割任务\n";
        //////////////////////////////////////////////////////////删除目录下所有 input_matching_cloud_x.pcd
        const boost::filesystem::path directory_path = WORK_PACKAGE_PATH + "/../pcd_maps";
        // 设置待删除的文件名前缀
        const std::string prefix_1 = "input_matching_cloud_1_";
        const std::string prefix_2 = "input_matching_cloud_2_";

        ///////////删除目录下开头为 input_matching_cloud_1_ 和 input_matching_cloud_2_的pcd文件/////////
        for (auto&& file : boost::filesystem::directory_iterator(directory_path))
        {
            // 检查文件名是否以指定的前缀开头，以及是否是PCD文件
            if (boost::starts_with(file.path().filename().string(), prefix_1) &&
                boost::iends_with(file.path().filename().string(), ".pcd"))
            {
                // 删除文件
                boost::filesystem::remove(file.path());
                std::cout << "Deleted file: " << file.path().filename().string() << std::endl;
            }
            if (boost::starts_with(file.path().filename().string(), prefix_2) &&
                boost::iends_with(file.path().filename().string(), ".pcd"))
            {
                // 删除文件
                boost::filesystem::remove(file.path());
                std::cout << "Deleted file: " << file.path().filename().string() << std::endl;
            }
        }


        ///////////删除目录下开头为 input_matching_cloud_1_ 和 input_matching_cloud_2_的pcd文件/////////
        //////////////////////////////////////////////////////////
        std::cout<<"use pcd map path1:"<<pcd_map_path_1<<"\n";
        std::cout<<"use pcd map path2:"<<pcd_map_path_2<<"\n";


        // pcl::PointCloud<pcl::PointXYZI> raw_cloud;
        // pcl::io::loadPCDFile (pcd_map_path, raw_cloud);
        /////////////////////////分割pcd1
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr map_pcd_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::io::loadPCDFile (pcd_map_path_1, *map_pcd_cloud);
        // 去除无效点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*map_pcd_cloud, *map_pcd_cloud, indices);

        int how_many_pcds = 10;
        int total_points_num = map_pcd_cloud->points.size();
        int points_num_for_one = total_points_num / how_many_pcds;
        std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> output_clouds(how_many_pcds);
        for (int i = 0; i < how_many_pcds; ++i) 
        {
            output_clouds[i] = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
        }
        for (int i = 0; i < total_points_num; ++i) 
        {
            int cloud_idx = (int)(i / points_num_for_one);
            if (cloud_idx > how_many_pcds-1) 
            {
                cloud_idx = how_many_pcds-1;
            }
            output_clouds[cloud_idx]->points.push_back(map_pcd_cloud->points[i]);
        }
        for (int i = 0; i < how_many_pcds; ++i) 
        {   
            std::string output_filename = WORK_PACKAGE_PATH + "/../pcd_maps/input_matching_cloud_1_" + std::to_string(i+1) + ".pcd";
            pcl::PCDWriter pcd_writer;
            std::cout << "current scan saved to /../pcd_maps/input_matching_cloud_1_" << std::to_string(i+1)<<".pcd" << std::endl;
            pcd_writer.writeBinary(output_filename, *output_clouds[i]);
            // pcl::io::savePCDFileASCII(output_filename, *output_clouds[i]);
            std::cout << "Saved " << output_filename << std::endl;
        }
        /////////////////////////分割pcd1

        /////////////////////////分割pcd2
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr map_pcd_cloud_2(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::io::loadPCDFile (pcd_map_path_2, *map_pcd_cloud_2);
        // 去除无效点
        std::vector<int> indices_2;
        pcl::removeNaNFromPointCloud(*map_pcd_cloud_2, *map_pcd_cloud_2, indices_2);

        int how_many_pcds_2 = 10;
        int total_points_num_2 = map_pcd_cloud_2->points.size();
        int points_num_for_one_2 = total_points_num_2 / how_many_pcds_2;
        std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> output_clouds_2(how_many_pcds_2);
        for (int i = 0; i < how_many_pcds_2; ++i) 
        {
            output_clouds_2[i] = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
        }
        for (int i = 0; i < total_points_num_2; ++i) 
        {
            int cloud_idx = (int)(i / points_num_for_one_2);
            if (cloud_idx > how_many_pcds_2-1) 
            {
                cloud_idx = how_many_pcds_2-1;
            }
            output_clouds_2[cloud_idx]->points.push_back(map_pcd_cloud_2->points[i]);
        }
        for (int i = 0; i < how_many_pcds_2; ++i) 
        {   
            std::string output_filename = WORK_PACKAGE_PATH + "/../pcd_maps/input_matching_cloud_2_" + std::to_string(i+1) + ".pcd";
            pcl::PCDWriter pcd_writer;
            std::cout << "current scan saved to /../pcd_maps/input_matching_cloud_2_" << std::to_string(i+1)<<".pcd" << std::endl;
            pcd_writer.writeBinary(output_filename, *output_clouds_2[i]);
            // pcl::io::savePCDFileASCII(output_filename, *output_clouds_2[i]);
            std::cout << "Saved " << output_filename << std::endl;
        }
        /////////////////////////分割pcd2
        std::cout<<"pcd 分割工作完成，话题发布\n";

        // 滤波
        std::shared_ptr<CloudFilterInterface> pcd_filter_ptr_;
        pcd_filter_ptr_ = std::make_shared<VoxelFilter>(0.11, 0.11, 0.11);
        pcd_filter_ptr_->Filter(map_pcd_cloud  , map_pcd_cloud  );
        pcd_filter_ptr_->Filter(map_pcd_cloud_2, map_pcd_cloud_2);
        rclcpp::Rate loop_rate(1);
        sensor_msgs::msg::PointCloud2 cloud_ptr_output_1;
        std::string frame_id = global_frame_id;
        cloud_ptr_output_1.header.stamp = ros2_node->now();
        cloud_ptr_output_1.header.frame_id = frame_id;
        pcl::toROSMsg(*map_pcd_cloud, cloud_ptr_output_1);
        sensor_msgs::msg::PointCloud2 cloud_ptr_output_2;
        cloud_ptr_output_2.header.stamp = ros2_node->now();
        cloud_ptr_output_2.header.frame_id = frame_id;
        pcl::toROSMsg(*map_pcd_cloud_2, cloud_ptr_output_2);
        // // 发布
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pcd_pub_1;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pcd_pub_2;
        // global_pcd_pub_1 = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>("global_pcd_map_file_1",1);
        // global_pcd_pub_2 = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>("global_pcd_map_file_2",1);

        // while (rclcpp::ok()) 
        // {   
            
        //     global_pcd_pub_1->publish(cloud_ptr_output_1);
        //     global_pcd_pub_2->publish(cloud_ptr_output_2);
        //     rclcpp::spin_some(ros2_node);
        //     loop_rate.sleep();
        // }
    }
    

    






    // {
    //     for (size_t i = 0; i < raw_cloud.points.size(); i++)
    //     {
    //     // if (raw_cloud.points[i].z<0.9&&raw_cloud.points[i].x<4.2&&raw_cloud.points[i].y>-0.8&&raw_cloud.points[i].x>-1.1&&raw_cloud.points[i].y<7.65)
    //     // {   
    //     //     if(raw_cloud.points[i].x>0.64)      //处理测枪平台
    //     //     {
    //     //         if (raw_cloud.points[i].z<0.5)
    //     //             map_pcd_cloud.push_back(raw_cloud.points[i]);

    //     //     }
    //     //     else
    //     //     {
    //     //         if (raw_cloud.points[i].y>5)
    //     //         {
    //     //             if (raw_cloud.points[i].z<0.5)
    //     //                 map_pcd_cloud.push_back(raw_cloud.points[i]);
    //     //         }

    //     //         else
    //     //             map_pcd_cloud.push_back(raw_cloud.points[i]);
    //     //     }
                
    //     // }
    //     map_pcd_cloud.push_back(raw_cloud.points[i]);
        
    //     }
    // }
    
    


    // // 去除无效点
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*map_pcd_cloud, *map_pcd_cloud, indices);

    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr filter_cloud_ptr(new pcl::PointCloud<pcl::PointXYZINormal>());
    // *filter_cloud_ptr = *map_pcd_cloud;
    // // 滤波
    // std::shared_ptr<CloudFilterInterface> pcd_filter_ptr_;
    // pcd_filter_ptr_ = std::make_shared<VoxelFilter>(0.1, 0.1, 0.1);
    // pcd_filter_ptr_->Filter(filter_cloud_ptr, filter_cloud_ptr);
    // *map_pcd_cloud = *filter_cloud_ptr;


    // sensor_msgs::msg::PointCloud2 cloud_ptr_output;
    // pcl::toROSMsg(*map_pcd_cloud, cloud_ptr_output);
    // std::string frame_id = global_frame_id;
    // cloud_ptr_output.header.stamp = ros2_node->now();
    // cloud_ptr_output.header.frame_id = frame_id;
    // // 发布
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pcd_pub;
    // global_pcd_pub = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>("global_pcd_map_file",1);

    
    // rclcpp::Rate loop_rate(1);
    // while (rclcpp::ok()) 
    // {
    //     global_pcd_pub->publish(cloud_ptr_output);
    //     rclcpp::spin_some(ros2_node);
    //     loop_rate.sleep();
    // }
    rclcpp::shutdown();
  
    return 0;
}

