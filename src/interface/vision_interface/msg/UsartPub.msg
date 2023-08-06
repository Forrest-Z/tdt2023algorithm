std_msgs/Header recv_header #与时间有关的单位 全部为s ， 距离有关的单位全部为cm , 角度有关的单位全部为弧度 。 使用时自行进行数据转换
float32 yaw         #单位：弧度
float32 pitch       #单位：弧度
uint8 enemycolor    #0：红色，1：error 2：蓝色
uint8 lockcommand   #锁定命令，0：未锁定，1：锁定
float32 nominal_bulletspeed 3000. #标称弹速，即裁判系统限制的最高弹速  单位：cm/s 
float32 bullet_speed 2600.        #实际弹速，即上一次发射的弹速       单位：cm/s
uint8 beat_mode #击打类型  包含平移，小陀螺，能量机关，吊射等击打类型 （
uint8 buff_type #大符类型  大符，小符 （视觉判断可能会误判，因此选择指令操作）
uint8 colorvincible
uint8 ifbeatsentry 0