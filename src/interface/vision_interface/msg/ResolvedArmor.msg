
##坐标系命名参考《视觉SLAM14讲/第二版》  P45，P46
#r12：描述坐标系2到坐标系1的旋转向量 r:旋转向量 R：旋转矩阵
#t12：从坐标系1到坐标系2的向量 , 坐标系1原点指向坐标系2原点
#TDT_VISION 旋转均为外旋
#当描述两坐标系a1，a2坐标转换时，约定如下： a1 = R_1_2 * a2 + t_1_2
#对理解有歧义的请联系 郑乐 qq:2360838113
uint8 armortype #装甲板类型
geometry_msgs/Point32 tvec_world_armor #世界坐标系原点指向装甲板（物体）坐标系的向量
geometry_msgs/Point32 rvec_world_armor #装甲板（物体）坐标系到世界坐标系的旋转矩阵
geometry_msgs/Point32 tvec_camera_armor #相机坐标系原点指向装甲板（物体）坐标系的向量
geometry_msgs/Point32 rvec_camera_armor #装甲板（物体）坐标系到相机坐标系的旋转矩阵

geometry_msgs/Point32[6] armor_point_image #灯条的6个顶点坐标 顺序为左上，左下，左中，右上，右中，右下 
geometry_msgs/Point32   robot_center_image
