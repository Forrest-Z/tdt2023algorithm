
##坐标系命名参考《视觉SLAM14讲/第二版》  P45，P46
#r12：描述坐标系2到坐标系1的旋转向量 r:旋转向量 R：旋转矩阵
#t12：从坐标系1到坐标系2的向量 | 坐标系 2 原点指向坐标系 1 原点的向量
#当描述两坐标系a1，a2坐标转换时，约定如下： a1 = R_1_2 * a2 + t_1_2
uint8 armortype #装甲板类型
geometry_msgs/Point32 tvec_world_armor #世界坐标系原点指向装甲板（物体）坐标系的向量
geometry_msgs/Point32 rvec_world_armor #装甲板（物体）坐标系到世界坐标系的旋转矩阵
geometry_msgs/Point32[4] buffarmor_point_image #大符的四个角点坐标 顺序为左上，左下，右上，右下 