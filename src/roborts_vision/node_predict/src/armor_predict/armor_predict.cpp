
#include "armor_predict/armor_predict.h"

#include <limits.h>
#include <roborts_utils/base_toolkit.h>

#include <cstddef>
#include <opencv2/core/types.hpp>
#include <utility>
#include <vector>

#define Radian_to_Angle 57.29578049
#define Angle_to_Radian 0.017453292
namespace armor_predict {

armor_predict::armor_predict(rclcpp::Node::SharedPtr &predict_node) {
  predict_node_ = predict_node;
  RCLCPP_INFO(predict_node_->get_logger(),
              "VISION_ARMOR_PREDICT has created !");
  options.minimizer_progress_to_stdout = false;  //禁用ceres输出

  LoadParam::InitParam("predict", "./config/vision_param.jsonc");
  LoadParam::InitParam("Camera_param", "./config/camera_param.jsonc");
  init();
}

void armor_predict::init() {
  std::cout << "armor_predict init" << std::endl;

  for (int i = 0; i < 9; i++) {  //装甲板信息，预测器信息全部初始化
    clearHis_info(i);
  }

  armorResolveSub = predict_node_->create_subscription<
      vision_interface::msg::ResolvePub>(  //接受发布消息重置
      "armor_resolve", 1,
      std::bind(&armor_predict::resolve_callback, this, std::placeholders::_1));

  usartSub =
      predict_node_->create_subscription<vision_interface::msg::UsartPub>(
          "visionUsartPub", 1,
          std::bind(&armor_predict::usart_sub, this,
                    std::placeholders::_1));  //订阅串口消息

  preceptionSub =
      predict_node_
          ->create_subscription<perception_interface::msg::Perception2Nav>(
              "Percep2Vision", 1,
              std::bind(&armor_predict::perception_sub, this,
                        std::placeholders::_1));  //订阅串口消息

  predictPub =
      predict_node_->create_publisher<vision_interface::msg::PredictPub>(
          "predictPub", 1);  //发布预测信息

  LoadParam::ReadParam("predict", "Follow_Yaw_offset",
                       yawOffset);  //读取消息重置
  LoadParam::ReadParam("predict", "Follow_Pitch_offset", pitchOffset);

  LoadParam::ReadParam("predict", "spinModeByDeepLearning",
                       spinModeByDeepLearning);

  LoadParam::ReadParam("predict", "calcVelthreshold", calcVelthreshold);
  LoadParam::ReadParam("predict", "centerTagthreshold", centerTagthreshold);
  LoadParam::ReadParam("predict", "spinPreScale", spinPreScale);
  LoadParam::ReadParam("predict", "spinBeatMinAngle", spinBeatMinAngle);
  LoadParam::ReadParam("Camera_param", "camera_matrix", cameraMatrix);
  LoadParam::ReadParam("Camera_param", "tvec_camera_in_world",
                       TvecCameraInWorld_);  //相机在世界坐标系下的平移向量

  LoadParam::ReadParam("predict", "sameArmorScore", sameArmorScore);
  LoadParam::ReadParam("predict", "continusTime", continusTime);
  LoadParam::ReadParam("predict", "armorDecisionAttenuationCoefficient",
                       armorDecisionAttenuationCoefficient);

  LoadParam::ReadParam("predict", "radiusFilter_Q", radiusFilter_Q);
  LoadParam::ReadParam("predict", "radiusFilter_R", radiusFilter_R);
  LoadParam::ReadParam("predict", "fireLength", fireLength);

  LoadParam::ReadParam("predict", "ifPredict", ifPredict);
  LoadParam::ReadParam("predict", "init_pitch", init_pitch);

  

  TvecCameraInWorld_.convertTo(TvecCameraInWorld_, CV_32FC1, 1, 0);
}

void armor_predict::resolve_callback(
    const vision_interface::msg::ResolvePub &msg) {
  // static vision_interface::msg::ResolvePub demo;
  // if (demo.header == msg.header) {
  //   int a = 2;
  // }
  // demo = msg;
  // std::cout<<"inhao"<<std::endl;

  std::vector<Armor> Armors[9];
  bool armorSame[9] = {0};  //是否出现相同类型装甲板
  platForm.yaw = msg.plat_form_yaw;
  platForm.pitch = msg.plat_form_pitch;
  platForm.timeStamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  setBeatMode((int)msg.mode);

  //分类
  for (int i = 0; i < msg.num; i++) {
    Armor armor;
    cv::Point2f centerPoint = cv::Point2f(0, 0);

    armor.armorType = msg.resolved_armors[i].armortype;  //数字类型

    roborts[armor.armorType].robotCenterInImage =
        cv::Point2f(msg.resolved_armors[i].robot_center_image.x,
                    msg.resolved_armors[i].robot_center_image.y);

    armor.tvec_world_armor =
        (cv::Mat_<float>(3, 1) << msg.resolved_armors[i].tvec_world_armor.x,
         msg.resolved_armors[i].tvec_world_armor.y,
         msg.resolved_armors[i].tvec_world_armor.z);

    armor.rvec_world_armor =
        (cv::Mat_<float>(3, 1) << msg.resolved_armors[i].rvec_world_armor.x,
         msg.resolved_armors[i].rvec_world_armor.y,
         msg.resolved_armors[i].rvec_world_armor.z);

    armor.tvec_camera_armor =
        (cv::Mat_<float>(3, 1) << msg.resolved_armors[i].tvec_camera_armor.x,
         msg.resolved_armors[i].tvec_camera_armor.y,
         msg.resolved_armors[i].tvec_camera_armor.z);

    armor.rvec_camera_armor =
        (cv::Mat_<float>(3, 1) << msg.resolved_armors[i].rvec_camera_armor.x,
         msg.resolved_armors[i].rvec_camera_armor.y,
         msg.resolved_armors[i].rvec_camera_armor.z);

    for (int j = 0; j < 6; j++) {  //灯条六点的像素坐标
      armor.armor_point_image[j] =
          cv::Point2f(msg.resolved_armors[i].armor_point_image[j].x,
                      msg.resolved_armors[i].armor_point_image[j].y);

      centerPoint += armor.armor_point_image[j];
    }
    armor.armor_center_point = centerPoint / 4;  //中心店

    armor.timeStap =
        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;  //时间戳 单位s

    armorSame[armor.armorType] = true;

    Armors[armor.armorType].push_back(armor);
  }

  //根据类别标记tag ， 开启openMP 优化

  if (msg.num) {  // 没有机器人时，则不进行位置更新
                  // #pragma omp parallel for
    for (int i = 0; i < 9; i++) {
      if (armorSame[i]) {
        CalcArmorTag(Armors[i]);  //标tag
        MeasureAxis(Armors[i]);   //计算轴心
        entryPoint(Armors[i]);
        EKF(Armors[i]);  //获得该类型机器人的后验估计
      }
    }

    lastDetectTime = getTimeNow();
  }


      predict();

  // if (getTimeNow() - lastDetectTime < continusTime) {  // 信息过期，则不进行预测
  //   predict();
  // } else {


  //   vision_interface::msg::PredictPub sendMsg;
  //   sendMsg.yaw = recvMsg.yaw;
  //   sendMsg.pitch = recvMsg.pitch;

  //   // std::cout<<"sendMsg.yaw= "<<sendMsg.yaw<<"sendMsg.pitch=
  //   // "<<sendMsg.pitch<<std::endl;

  //   predictPub->publish(sendMsg);  // no_obj 默认为 1
  // }

  // std::cout<<"num= "<<(int)msg.num<<"getTimeNow()=
  // "<<getTimeNow()-lastDetectTime<<std::endl;
}

void armor_predict::usart_sub(const vision_interface::msg::UsartPub &msg) {
  recvMsg = msg;  //保证预测使用最新串口信息
}

std::pair<int, int> armor_predict::roborts_decision() {
  // int score[9][4];  //负无穷
  // memset(score, INT_MIN, sizeof(score));
  std::vector<std::pair<int, std::pair<int, int> > > scores;

  for (int i = 1; i < 9; i++) {
    for (auto &armors : roborts[i].armors) {
      if (armors.empty()) {
        continue;
      }
      auto armor = armors.back();

      std::cout << "armorType= " << armor.armorType << std::endl;
      if (armor.armorType == 8 || armor.armorType == 6) {
        continue;
      }

      // if (armor.armorType == 7 && (!recvMsg.ifbeatsentry)) {
      //   continue;
      // }

      if (getTimeNow() - armor.timeStap > continusTime) {
        clearHis_info(i, armor.armorTag);
        continue;
      }
      // score[i][j] = 0 ;
      int score = 0;

      switch (armor.armorType) {
        case tdttoolkit::RobotType::HERO:
          score += 500;
          break;
        case tdttoolkit::RobotType::ENGINEER:
          score += 100;
          break;
        case tdttoolkit::RobotType::SENTRY:
          score += 50;
          break;
        default:
          score += 300;
          break;
      }

      score += (int)(armor.getArea() / 17.);
      // std::cout << "area= " << armor.getArea() << std::endl;
      score /= cv::norm(armor.tvec_world_armor * 0.06);

      score *= std::exp((armor.timeStap - getTimeNow()) /
                        armorDecisionAttenuationCoefficient);

      int lastTag = roborts[armor.armorType].lastChosenTag;
      if (lastTag != -1) {
        score *= 1.85; /* code */
      }

      scores.push_back(
          std::make_pair(score, std::make_pair(i, armor.armorTag)));

      // score /=
    }
  }

  //降序，将得分按照从高到低进行排序
  if (scores.empty()) {
    return std::make_pair(-1, -1);
  }

  std::sort(
      scores.begin(), scores.end(),
      [](std::pair<int, std::pair<int, int> > a,
         std::pair<int, std::pair<int, int> > b) { return a.first > b.first; });

  // scores[0].second=std::make_pair(5, 0);
  roborts[scores[0].second.first].lastChosenTag = scores[0].second.second;
  return scores[0].second;
}

float armor_predict::CalcGIou(Armor cntArmor, Armor lastArmor) {
  cv::Point2f cntBr = cntArmor.armor_point_image[4],  // cnt右下
      cntTl = cntArmor.armor_point_image[0],          // cnt左上
      lastBr = lastArmor.armor_point_image[4],        // last右下
      lastTl = lastArmor.armor_point_image[0];        // last左上

  float inter_h = fmin(cntBr.y, lastBr.y) -
                  fmax(cntTl.y, lastTl.y),  // 交集矩形区域的长宽
      inter_w = fmin(cntBr.x, lastBr.x) - fmax(cntTl.x, lastTl.x);

  inter_h = fmax(0, inter_h);
  inter_w = fmax(0, inter_w);
  float intersection = inter_h * inter_w,  // 面积交集
      unit = (cntBr.x - cntTl.x) * (cntBr.y - cntTl.y) +
             (lastBr.x - lastTl.x) * (lastBr.y - lastTl.y) -
             intersection;  // 面积并集

  float convex_h = fmax(cntBr.y, lastBr.y) - fmin(cntTl.y, lastTl.y),
        convex_w = fmax(cntBr.x, lastBr.x) - fmin(cntTl.x, lastTl.x),
        convex = convex_h * convex_w;  //
  float tool = intersection / unit - (convex - unit) / convex;
  //   std::cout<<"Iou= "<<intersection/unit<<std::endl;
  return tool;
}

void armor_predict::CalcArmorTag(
    std::vector<Armor> &armors) {  //同一类型装甲板计算tag

  int armorType = armors[0].armorType;

  if (roborts[armorType].lastArmors.size() == 0) {  //首次看到装甲板
    for (int i = 0; i < armors.size(); i++) {
      armors[i].armorTag = (i) % 4;  //规定tag从0 ->1 ->2 ->3 ->0算起
    }
  }

  else if (armors[0].timeStap - roborts[armorType].lastArmors[0].timeStap >
           continusTime) {  //超时

    clearHis_info(armorType);

    for (int i = 0; i < armors.size(); i++) {
      armors[i].armorTag = i;  //规定tag从0 ->1 ->2 ->3 ->0算起
    }
  }

  else {  //有效信息
          //注意：平衡在判断是只存在一块装甲板，根据2022赛季，平衡小陀螺时的装甲板tag存在问题。
          //猜测原因在于平衡tag分配只存在1对1情况，过于依赖GIOU判定（像素相近）。

    std::vector<std::pair<float, Armor> > resultTag[2];  // GIOU得分，tag
    int i = 0;

    for (auto &cntArmor : armors) {
      for (auto &lastArmor : roborts[armorType].lastArmors) {  // at least j >0
        resultTag[i].push_back(
            std::make_pair(CalcGIou(cntArmor, lastArmor), lastArmor));
        // std::cout<<"score= "<<resultTag[i].back().first<<std::endl;
      }
      ++i;
    }

    // std::cout<<"i= "<<i<<std::endl;
    if (i == 1) {  // 当前帧只有单块装甲板进行匹配
      std::sort(resultTag[0].begin(), resultTag[0].end(),
                [](std::pair<float, Armor> armor1,
                   std::pair<float, Armor> armor2) -> bool {
                  return (armor1.first > armor2.first);
                });  //按照相似度从大至小进行排序

      if (resultTag[0][0].first > sameArmorScore ||
          roborts[armorType].lastArmors.size() == 2) {  //满足匹配
        armors[0].armorTag = resultTag[0][0].second.armorTag;

      } else {
        if (armors[0].armor_center_point.x <=
            resultTag[0][0].second.armor_center_point.x) {  //当前帧装甲板在左侧
          armors[0].armorTag = (resultTag[0][0].second.armorTag + 1) % 4;
        } else {
          armors[0].armorTag =
              (resultTag[0][0].second.armorTag + 3) % 4;  //当前帧装甲板在右侧
        }
      }

      // std::cout << "armortag= " << armors[0].armorTag << std::endl;
    }

    else {  //当前帧有两块装甲板进行匹配
      // std::cout << "in" << std::endl;
      int max_id = 0;
      // max_id =
      //     resultTag[0][0].first > resultTag[1][0].first
      //         ? 0
      //         : 1;
      //         //思路：找到当前帧匹配度最高装甲板去继承tag，第二块当前帧装甲板跟去第一块当前帧装甲板相对位置计算tag

      //最大评分装甲板满足匹配，则继承tag。
      // std::cout << "intag= " << armors[max_id].armorTag << "  "
      //           << resultTag[max_id][0].second.armorTag << " "
      //           << resultTag[1][0].second.armorTag << std::endl;

      armors[max_id].armorTag = resultTag[max_id][0].second.armorTag;

      //根据相对位置计算第二块装甲板tag
      int other_id = (max_id + 1) % 2;

      if (armors[other_id].armor_center_point.x <
          armors[max_id].armor_center_point.x) {
        armors[other_id].armorTag =
            (armors[max_id].armorTag + 3) % 4;  //第二块在第一块左侧
      } else {
        armors[other_id].armorTag = (armors[max_id].armorTag + 1) % 4;
      }
    }
  }

  roborts[armorType].lastArmors.clear();

  //将当前帧装甲板信息存入roborts,储存上一帧历史信息
  for (auto &armor : armors) {
    // std::cout << "tag== " << armor.armorTag << "type= " << armor.armorType
    //           << std::endl;
    roborts[armorType].armors[armor.armorTag].push_back(armor);
    roborts[armorType].lastArmors.push_back(armor);

    // std::cout << "size= " << roborts[armorType].armors[armor.armorTag].size()
    //           << std::endl;

    if (roborts[armorType].armors[armor.armorTag].size() > dequeMaxNum) {
      roborts[armorType].armors[armor.armorTag].pop_front();
    }
  }

  //计算装甲板tag  1-1,2-2 根据逻辑筛掉 . 1-2 , 2-1 暂时采取giou进行计算
}

void armor_predict::predict() {
  // armordeciocn 开火决策，返回击打的机器人类型
  std::pair<int, int> last = beatRobotype;
  beatRobotype = roborts_decision();


  if (beatRobotype.first == -1  ) {
    
    if(getTimeNow() - lastDetectTime < continusTime){  //历史信息
      beatRobotype = last ;
      firecommand();
    }
    else{ //感知决策信息or上次识别角度

    vision_interface::msg::PredictPub sendMsg;
    sendMsg.yaw =  (decision_yaw==0?recvMsg.yaw :decision_yaw);
    sendMsg.pitch = init_pitch;
    overCircle(sendMsg);
    predictPub->publish(sendMsg);  // no_obj 默认为 1
    return;
    }

  }

  firecommand();
}

void armor_predict::clearHis_info(int roboType, int armorTag) {
  roborts[roboType].lastChosenTag = -1;
  if (armorTag != -1) {
    roborts[roboType].armors[armorTag].clear();
    roborts[roboType].ekfFilter[armorTag].init(cv::Point3f(-1., -1., -1.));
    // ekfFilter[armorTag].init(cv::Point3f(-1., -1., -1.));
    return;
  }

  roborts[roboType].lastArmors.clear();
  for (int i = 0; i < 4; i++) {
    roborts[roboType].armors[i].clear();
  }

  roborts[roboType].InitializationKalmanFilter();

  roborts[roboType].robotsAxis_.clear();

  roborts[roboType].changeTag = true;
  roborts[roboType].robotCenterInImage = cv::Point2f(0, 0);
}

void armor_predict::firecommand() {
  if (roborts[beatRobotype.first].lastArmors.empty()) {  //未有有效信息
    return;
  }

  //单位:s ，选取串口时间作为cntTime

  if (getTimeNow() - roborts[beatRobotype.first].lastArmors[0].timeStap >
      continusTime) {  //识别信息超时。原因： 标tag是与识别挂钩的 ，
                       //预测则是单独执行 。 因此超时时间的判定要分别进行
    clearHis_info(beatRobotype.first);
    return;
  }

  // ZLL-TODO: 是否要加入有效信息数目的判断？只有一帧有效信息能否进入准确预测？
  //目前进入预测最低标准为 至少1次有效历史信息
  vision_interface::msg::PredictPub sendMsg;

  sendMsg.yaw = recvMsg.yaw;
  sendMsg.pitch = recvMsg.pitch;

  int status = GetStatus();  // ZLL-TODO: 选取合适的运动学模型

  std::cout << "status= " << status << std::endl;
  switch (status) {
    case 0:  //平移击打模式
      /* code */
      FollowModeShoot(sendMsg);
      break;
    case 1:  //小陀螺击打模式
      spinModeShoot_2(sendMsg);

      // FollowModeShoot(sendMsg);

      break;

    case 2:  //建筑击打模式
      break;
      FollowModeShoot(sendMsg);
  }
  decision_yaw = sendMsg.yaw ; 
  overCircle(sendMsg);
  fireGunOffset(sendMsg);



  predictPub->publish(sendMsg);
}

int armor_predict::GetStatus() {
  int status = 0;

  float linerVel = 0;
  for (auto &armor : roborts[beatRobotype.first].lastArmors) {
    linerVel += armor.getLinerVel();
  }

  // cv::Point3f cntAxis = roborts[beatRobotype.first].axis.getAxis(1),
  //             lastAxis = roborts[beatRobotype.first].axis.getAxis(0);

  cv::Point3f cntAxis = roborts[beatRobotype.first].robotsAxis_.getAxis(false),
              lastAxis = roborts[beatRobotype.first].robotsAxis_.getAxis(true);
  if (roborts[beatRobotype.first]
          .lastArmors.size()) {  //基本不可能发生，但还是做个数据防护，防止除以0
    linerVel /= roborts[beatRobotype.first].lastArmors.size();
  }

  if (linerVel > minLinerVel &&
      getTimeNow() - lastEnterCenterTime < spinTimeOut) {
    if (cntAxis == cv::Point3f(-1, -1, -1) ||
        lastAxis == cv::Point3f(-1, -1, -1)) {
      status = 0;
    } else if (fabs(tdttoolkit::RectangularToPolar(cntAxis).yaw -
                    tdttoolkit::RectangularToPolar(lastAxis).yaw) >
               spinMoveMinDelatYaw) {
      status = 0;
    } else {
      status = 1;
    }
  } else {
    status = 0;
  }
  //下面是状态判断 ，规定： 0：平移  1：小陀螺  2：建筑击打模式
  return status;
}

void armor_predict::FollowModeShoot(vision_interface::msg::PredictPub &msg) {
  // int t = armorDecision();   // ZLL-TODO: 击打装甲板tag
  int t = beatRobotype.second;
  cv::Point3f predictPoint;  //装甲板中心预测点

  cv::Point3f cntPoint =
      tdttoolkit::MatToPoint3f(roborts[beatRobotype.first]
                                   .armors[t]
                                   .back()
                                   .tvec_world_armor);  //最新点的直角坐标

  tdttoolkit::Polar3f cntPolar =
      tdttoolkit::RectangularToPolar(cntPoint);  //最新点的极坐标

  float beatHeight = cntPoint.y;
  float disHorizontal = abs(cntPolar.distance * cos(cntPolar.pitch));

  if (roborts[beatRobotype.first].armors[t].size() ==
      1) {  //只有一帧信息无法做预测，因此将cntPoint作为预测点
    predictPoint = cntPoint;
  }

  else {
    // 预测时间，单位s
    float bulletArriveTime = sqrt(disHorizontal / recvMsg.bullet_speed);
    // float bulletArriveTime = sqrt(disHorizontal / 1500);

    float yawPredictTime =  //竖直方向不进行预测
        (bulletArriveTime +
         (getTimeNow() -
          roborts[beatRobotype.first].armors[t].back().timeStap));

    // ZLL-TODO:
    // 理论上应该使用滤波器给出的位置信息，但一是现有框架大都使用的是解算信息，二是22赛季使用效果并不理想，原因待查
    //预留接口，上面为解算位置 ， 下面为滤波器位置

    float vx = roborts[beatRobotype.first].statePost[t].first.at<float>(1, 0);
    float vz = roborts[beatRobotype.first].statePost[t].first.at<float>(3, 0);

    if (vx > 70) {
      vx *= 0.7;
    }
    if (vz > 70) {
      vz *= 0.7;
    }

    predictPoint.x = cntPoint.x + yawPredictTime * vx;
    predictPoint.y = cntPoint.y;
    predictPoint.z = cntPoint.z + yawPredictTime * vz;

    // predictPoint.x =
    //     roborts[beatRobotype.first].statePost[t].first.at<float>(0, 0) +
    //     yawPredictTime *
    //         roborts[beatRobotype.first].statePost[t].first.at<float>(1, 0);

    // predictPoint.y = cntPoint.y;

    // predictPoint.z =
    //     roborts[beatRobotype.first].statePost[t].first.at<float>(2, 0) +
    //     yawPredictTime *
    //         roborts[beatRobotype.first].statePost[t].first.at<float>(3, 0);

    // predictPoint = cntPoint;
    if (!ifPredict || sqrt(pow(vx, 2) + pow(vz, 2)) > 100)
      predictPoint = cntPoint;
  }

  prepareBeat(predictPoint, disHorizontal, msg, beatRobotype.first);
}

void armor_predict::spinModeShoot_2(vision_interface::msg::PredictPub &msg) {
  cv::Point3f cntAxis = roborts[beatRobotype.first].robotsAxis_.getAxis();

  if (cntAxis == cv::Point3f(-1, -1, -1)) {  // 轴心数目少 或 轴心速度过大
    FollowModeShoot(msg);
    return;
  }

  float bulletArriveTime =
      cv::norm(DistanceCompensation(cntAxis, beatRobotype.first,
                                    beatRobotype.second)) /
      recvMsg.bullet_speed;  // 单位：s

  float yawPredictTime = (bulletArriveTime);  // 单位：秒

  cv::Point3f predictAxis =
      roborts[beatRobotype.first]
          .robotsAxis_.getAxis();  //未对轴心做预测，暂且采用当前值

  //整个底盘进行建模
  cv::Point3f cntPosition[4];      // 当前点
  cv::Point3f predictPosition[4];  //预测点

  auto cntArmors = roborts[beatRobotype.first].lastArmors;

  //计算角速度
  float angleVel = 0;  // 弧度/s

  if (cntArmors.size() == 2) {
    angleVel = (cntArmors[0].getLinerVel() + cntArmors[1].getLinerVel()) /
               (roborts[cntArmors[0].armorType].robotsAxis_.getRadius(
                    cntArmors[0].armorType, cntArmors[0].armorTag) +
                roborts[cntArmors[1].armorType].robotsAxis_.getRadius(
                    cntArmors[1].armorType, cntArmors[1].armorTag));
  } else {
    angleVel = cntArmors.front().getLinerVel() /
               roborts[cntArmors[0].armorType].robotsAxis_.getRadius(
                   cntArmors[0].armorType, cntArmors[0].armorTag);
  }

  float spinAngle = angleVel * yawPredictTime;
  float theta = CV_PI / 2;  // 弧度
  int turn = roborts[beatRobotype.first].robotsAxis_.selfTurn;
  switch (turn) {
    case 0:  // 未知
      spinAngle = 0;
      break;
    case 1:  // 顺时针
      spinAngle = -spinAngle;
      break;
    case -1:  // 逆时针
      break;
  }

  if (roborts[beatRobotype.first].lastArmors.size() == 1) {
    cv::Point3f tvec = tdttoolkit::MatToPoint3f(cntArmors[0].tvec_world_armor);

    cntPosition[cntArmors[0].armorTag] = tvec;
    predictPosition[cntArmors[0].armorTag] =
        spinPoint(cntAxis, cntPosition[cntArmors[0].armorTag], spinAngle,
                  predictAxis - cntAxis);

    cntPosition[(cntArmors[0].armorTag + 2) % 4] =
        cv::Point3f((2 * cntAxis.x - tvec.x), tvec.y,
                    (2 * cntAxis.z - tvec.z));  //这里计算了正对面的装甲板中心点

    predictPosition[(cntArmors[0].armorTag + 2) % 4] =
        spinPoint(cntAxis, cntPosition[(cntArmors[0].armorTag + 2) % 4],
                  spinAngle, predictAxis - cntAxis);

    // cv::Mat rotateMatrix =
    //     (cv::Mat_<float>(2, 2) << cos(theta), -sin(theta), sin(theta),
    //      cos(theta));  //旋转矩阵，表示一个向量逆时针旋转theta弧度

    //这里计算了逆时针方向上旋转90度的装甲板中心点
    cntPosition[(cntArmors[0].armorTag + 1) % 4] =
        spinPoint(cntAxis, tvec, theta);

    predictPosition[(cntArmors[0].armorTag + 1) % 4] =
        spinPoint(cntAxis, cntPosition[(cntArmors[0].armorTag + 1) % 4],
                  spinAngle, predictAxis - cntAxis);

    cntPosition[(cntArmors[0].armorTag + 3) % 4] =
        spinPoint(cntAxis, tvec, -theta);

    predictPosition[(cntArmors[0].armorTag + 3) % 4] =
        spinPoint(cntAxis, cntPosition[(cntArmors[0].armorTag + 3) % 4],
                  spinAngle, predictAxis - cntAxis);

  } else {
    for (auto &armor : cntArmors) {
      cv::Point3f tvec = tdttoolkit::MatToPoint3f(armor.tvec_world_armor);

      cntPosition[armor.armorTag] = tvec;

      predictPosition[armor.armorTag] =
          spinPoint(cntAxis, cntPosition[armor.armorTag], spinAngle,
                    predictAxis - cntAxis);

      cntPosition[(armor.armorTag + 2) % 4] = cv::Point3f(
          (2 * cntAxis.x - tvec.x), tvec.y, (2 * cntAxis.z - tvec.z));

      predictPosition[(armor.armorTag + 2) % 4] =
          spinPoint(cntAxis, cntPosition[(armor.armorTag + 2) % 4], spinAngle,
                    predictAxis - cntAxis);
    }
  }

  float angle[4] = {0};
  for (int i = 0; i < 4; i++) {
    /*叉乘判断角度大小 bymomo 291590819*/

    cv::Point3f cross = predictAxis.cross(predictPosition[i]);
    bool flag = cross.y > 0 ? true : false;
    bool if_explementary = false; /*是否取360度补角 */

    if (turn == 1) /*顺时针*/ {  //注意!!世界坐标Y正方向是向下的
      if_explementary = !flag;
    } else if (turn == -1) /*逆时针*/ {
      if_explementary = flag;
    }
    /*点乘计算旋转角度*/
    angle[i] = acos(
        (predictPosition[i] - predictAxis).dot(-predictAxis) /
        (cv::norm(predictPosition[i] - predictAxis) * cv::norm(predictAxis)));

    angle[i] = if_explementary ? 2 * CV_PI - angle[i] : angle[i];
  }

  int min = 0;

  for (int i = 1; i < 4; i++) {
    float angle_ = (angle[i] > 180. ? 360. - angle[i] : angle[i]);
    float angle_min = (angle[min] > 180. ? 360. - angle[min] : angle[min]);
    if (fabs(angle_) < fabs(angle_min)) {
      min = i;
    }
  }

  float angle_min = (angle[min] > 180. ? 360. - angle[min] : angle[min]);

  if (fabs(angle_min) < spinBeatMinAngle) {
    // 角度
    float levelDis =
        sqrt(pow(predictPosition[min].x, 2) + pow(predictPosition[min].z, 2));

    prepareBeat(predictPosition[min], levelDis, msg, beatRobotype.first);
  }
}

void armor_predict::prepareBeat(cv::Point3f predictPoint, float disHorizontal,
                                vision_interface::msg::PredictPub &msg,
                                int beatType) {
  auto gravity_compensation = tdttoolkit::ParabolaSolve(
      cv::Point2f(disHorizontal, predictPoint.y), recvMsg.bullet_speed);

  float pitch = fabs(gravity_compensation[0] - recvMsg.pitch) <
                        fabs(gravity_compensation[1] - recvMsg.pitch)
                    ? gravity_compensation[0]
                    : gravity_compensation[1];

  tdttoolkit::Polar3f predictPolar =
      tdttoolkit::RectangularToPolar(predictPoint);

  msg.yaw = predictPolar.yaw;
  msg.pitch = pitch;
  msg.no_obj = 0;
  msg.beat = 0;

  int tool = (beatType == 7 ? 6 : beatType);
  if (((recvMsg.colorvincible >> (tool)) & 1) == 1) {
    return;
  }
  std::cout << "wudi= " << (int)recvMsg.colorvincible << std::endl;
  if (fabs(sin(recvMsg.yaw - msg.yaw)) * disHorizontal <
          fireLength + fabs(sin(yawOffset)) * disHorizontal &&
      fabs(sin(recvMsg.pitch - pitch)) * disHorizontal <
          fireLength + fabs(sin(pitchOffset)) * disHorizontal) {
    msg.beat = 1;
  }

  std::cout << "NO_OBJ= " << (int)msg.no_obj << " TYPE=  " << beatType << "  "
            << (int)msg.beat << "type= " << beatRobotype.first << std::endl;
}

void armor_predict::EKF(std::vector<Armor> &armors) {
  //对此命名有疑问的请查阅lastArmors的定义，在此不再赘述
  for (auto &cntArmor : armors) {
    int t = cntArmor.armorTag;
    cv::Point3f cntPoint = tdttoolkit::MatToPoint3f(cntArmor.tvec_world_armor);
    int size = roborts[cntArmor.armorType].armors[t].size();
    if (size == 1) {
      //没有历史信息时 使用当前位置初始化滤波器 ，使其快速收敛
      roborts[cntArmor.armorType].ekfFilter[t].InitParam("predict");
      roborts[cntArmor.armorType].ekfFilter[t].init(cntPoint);
      continue;
    }
    float deltaT =
        (roborts[cntArmor.armorType].armors[t].back().timeStap -
         roborts[cntArmor.armorType].armors[t][size - 2].timeStap);  // s

    roborts[cntArmor.armorType].ekfFilter[t].SetR();
    roborts[cntArmor.armorType].ekfFilter[t].SetQ(deltaT);
    roborts[cntArmor.armorType].ekfFilter[t].Predict();

    tdttoolkit::Polar3f cntPolar = tdttoolkit::RectangularToPolar(cntPoint);

    cv::Mat measure =
        (cv::Mat_<float>(2, 1) << cntPolar.yaw,
         abs(cntPolar.distance *
             cos(cntPolar.pitch)));  //理论上电控水平pitch为0，-40 ~ 40°范围

    roborts[cntArmor.armorType].statePost[t] = std::make_pair(
        roborts[cntArmor.armorType].ekfFilter[t].Correct(measure, true),
        cntArmor.timeStap);  //  cm/s  , s

    auto &armor = roborts[cntArmor.armorType].armors[t].back();
    auto &statePost = roborts[cntArmor.armorType].statePost[t];

    // armor.tvec_world_armor.at<float>(0, 0) = statePost.first.at<float>(0, 0);
    // armor.tvec_world_armor.at<float>(2, 0) = statePost.first.at<float>(2, 0);

    armor.armorLinerVel.at<float>(0, 0) =
        statePost.first.at<float>(1, 0);  // Vx
    armor.armorLinerVel.at<float>(2, 0) =
        statePost.first.at<float>(3, 0);  // Vz
  }
}

void armor_predict::overCircle(vision_interface::msg::PredictPub &sendMsg) {
  float recv_yaw = recvMsg.yaw * Radian_to_Angle;  //云台yaw的角度
  float platformPosition = recv_yaw - (int)(recv_yaw / 360.) * 360.;
  float visionPosition = platformPosition > 180.
                             ? platformPosition - 360.
                             : platformPosition;  //云台位置在视觉坐标系的位置
  float diff = sendMsg.yaw * Radian_to_Angle - visionPosition;

  if (diff > 180.) {
    diff -= 360.;

  } else if (diff < -180.) {
    diff += 360.;
  }

  sendMsg.yaw = recvMsg.yaw + diff * Angle_to_Radian;  //弧度
}

void armor_predict::MeasureAxis(
    std::vector<Armor>
        &armors) {  //所有轴心从计算到优化全部在相机系下进行，然后再同步到世界系
  std::vector<cv::Point3f> axisPoints;
  switch (armors.size()) {
    case 1:
      // CalcAxisYawBySingleArmor(armors.back(), getTimeNow());
      // CalcAxisYawBySingleArmor_2_0(armors.back());
      axisPoints.push_back(calcAxisBy_normalExtension(armors.back()));
      break;
    case 2:
      // CalcAxisYawByDoubleArmor_2_0(armors.front(), armors.back());
      // CalcAxisYawByDoubleArmor(armors.front(), armors.back(), getTimeNow());
      axisPoints.push_back(calcAxisBy_normalExtension(armors.front()));
      axisPoints.push_back(calcAxisBy_normalExtension(armors.back()));
      axisPoints.push_back(
          calcAxisBy_normalIntersection(armors.front(), armors.back()));

      cost_function = new ceres::AutoDiffCostFunction<MyCostFunction, 1, 1, 1>(
          new MyCostFunction(
              cv::Point2f(armors.front().tvec_camera_armor.at<float>(0, 0),
                          armors.front().tvec_camera_armor.at<float>(2, 0)),
              cv::Point2f(armors.back().tvec_camera_armor.at<float>(0, 0),
                          armors.back().tvec_camera_armor.at<float>(2, 0)),
              getRobotYawInCamera(
                  armors.front().armorType)));  // TODO: 未给轴心像素yaw角

      break;
  }
  cv::Point3f axisPoint_corrected =
      correctAxis_2_0(axisPoints, armors[0].armorType);

  if (axisPoint_corrected !=
      cv::Point3f(0, 0, 0)) {  //得到有效的轴心值，相机系转移到世界系
    cv::Point3f axisInWorld = camera_TO_world(axisPoint_corrected);

    roborts[armors[0].armorType].robotsAxis_.axis.push_back(
        std::make_pair(axisInWorld, armors[0].timeStap));

    int size = roborts[armors[0].armorType].robotsAxis_.axis.size();

    if (size > dequeMaxNum) {
      roborts[armors[0].armorType].robotsAxis_.axis.pop_front();
    }

    if (size == 1) {
      roborts[armors[0].armorType].robotsAxis_.axisFilter.InitParam(
          "predict",
          1);  // spinMode EKF
      roborts[armors[0].armorType].robotsAxis_.axisFilter.init(axisInWorld);
    } else {
      float deltaT =
          (roborts[armors[0].armorType].robotsAxis_.axis.back().second -
           roborts[armors[0].armorType]
               .robotsAxis_.axis[size - 2]
               .second);  // s

      roborts[armors[0].armorType].robotsAxis_.axisFilter.SetR();
      roborts[armors[0].armorType].robotsAxis_.axisFilter.SetQ(deltaT);
      roborts[armors[0].armorType].robotsAxis_.axisFilter.Predict();
      tdttoolkit::Polar3f cntPolar =
          tdttoolkit::RectangularToPolar(axisInWorld);

      cv::Mat measure =
          (cv::Mat_<float>(2, 1) << cntPolar.yaw,
           abs(cntPolar.distance *
               cos(cntPolar.pitch)));  //理论上电控水平pitch为0，-40 ~ 40°范围
      roborts[armors[0].armorType].robotsAxis_.statePostAxis = std::make_pair(
          roborts[armors[0].armorType].robotsAxis_.axisFilter.Correct(measure,
                                                                      true),
          armors[0].timeStap);
    }

    for (auto &armor : armors) {  //轴长

      float length = cv::norm(axisInWorld -
                              tdttoolkit::MatToPoint3f(armor.tvec_world_armor));

      roborts[armor.armorType]
          .robotsAxis_.radiusLength[armor.armorTag % 2]
          .push_back(std::make_pair(length, armor.timeStap));

      int size = roborts[armors[0].armorType]
                     .robotsAxis_.radiusLength[armor.armorTag % 2]
                     .size();

      if (size > dequeMaxNum) {
        roborts[armor.armorType]
            .robotsAxis_.radiusLength[armor.armorTag % 2]
            .pop_front();
      }

      if (size == 1) {
        roborts[armors[0].armorType].robotsAxis_.radiusFilter.init(25.);
      } else {
        roborts[armors[0].armorType].robotsAxis_.radiusFilter.SetR(
            radiusFilter_R);
        roborts[armors[0].armorType].robotsAxis_.radiusFilter.SetQ(
            radiusFilter_Q);
      }
      roborts[armors[0].armorType]
          .robotsAxis_.statePostRadius[armor.armorTag % 2] = std::make_pair(
          roborts[armors[0].armorType].robotsAxis_.radiusFilter.Estimate(
              length),
          armors[0].timeStap);
    }
  }
}

void armor_predict::entryPoint(std::vector<Armor> &armors) {
  int centerTag = 0;
  for (auto &armor : armors) {
    bool inCenterFortag = false, inCenterForTurn = false;
    int t = armor.armorTag;
    // 计算转速
    if (spinModeByDeepLearning &&
        armor.armor_center_point != cv::Point2f(0, 0)) {
      float ifCenter = fabs(roborts[armor.armorType].robotCenterInImage.x -
                            armor.armor_center_point.x) /
                       armor.GetlightCenter_dis_();
      if (ifCenter < calcVelthreshold) {  // 进入中心区域
        inCenterForTurn = true;
      }
      if (ifCenter < centerTagthreshold) {
        inCenterFortag = true;
      }
    } else {
      // cv::Point3f cntAxis = roborts[armor.armorType].axis.getAxis();
      cv::Point3f cntAxis = roborts[armor.armorType].robotsAxis_.getAxis();
      float cntArmorYaw =
          tdttoolkit::RectangularToPolar(armor.getRectangular()).yaw;
      if (cntAxis != cv::Point3f(-1, -1, -1) &&
          fabs(cntArmorYaw - tdttoolkit::RectangularToPolar(cntAxis).yaw *
                                 Radian_to_Angle <
               centerTagthreshold)) {
        inCenterFortag = true;
      }
      if (cntAxis != cv::Point3f(-1, -1, -1) &&
          fabs(cntArmorYaw - tdttoolkit::RectangularToPolar(cntAxis).yaw *
                                 Radian_to_Angle <
               calcVelthreshold)) {
        inCenterForTurn = true;
      }
    }

    if (inCenterForTurn) {
      int size = roborts[armor.armorType].armors[t].size();
      if (size > 1) {
        //判断转向
        float cntYaw =
            tdttoolkit::RectangularToPolar(armor.getRectangular()).yaw;

        float lastYaw =
            tdttoolkit::RectangularToPolar(
                roborts[armor.armorType].armors[t][size - 2].getRectangular())
                .yaw;

        if (cntYaw - lastYaw < 0 || cntYaw - lastYaw > 180) {
          // roborts[armor.armorType].axis.selfTurn = -1;
          roborts[armor.armorType].robotsAxis_.selfTurn = -1;
          // 0->未知  1->逆时针 -1->顺时针
        } else if (cntYaw - lastYaw > 0 || cntYaw - lastYaw < -180) {
          // roborts[armor.armorType].axis.selfTurn = 1;
          roborts[armor.armorType].robotsAxis_.selfTurn = 1;
        }
      }
    }
    if (inCenterFortag) {
      centerTag += (1 << t);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (((centerTag >> i) % 2) &&
        ((roborts[armors[0].armorType].lastTag >> i) % 2 ==
         0)) {  // 该装甲板首次进入中心区域.
                // 逻辑就是centerTag为第一次进入的装甲板

      lastEnterCenterTime = armors[0].timeStap;
      roborts[armors[0].armorType].lastTag = centerTag;
    }
  }
}

cv::Point3f armor_predict::DistanceCompensation(cv::Point3f axis_point,
                                                int armorType, int armorTag) {
  float axis_distance =
      sqrt(axis_point.x * axis_point.x +
           axis_point.z * axis_point.z);  // 轴心到世界坐标系原点抛弃y轴距离大小
                                          // //轴心到世界坐标系原点距离大小
  float armor_distance =
      axis_distance -
      (roborts[armorType].robotsAxis_.getRadius(
          armorType, armorTag));  // 装甲板中心抛弃y轴到世界坐标系原点大小

  axis_point = axis_point / axis_distance * armor_distance;

  axis_point.y =
      roborts[armorType].armors[armorTag].back().tvec_world_armor.at<float>(1,
                                                                            0);

  return axis_point;
}

cv::Point3f armor_predict::calcAxisBy_normalExtension(Armor armor) {
  //获得装甲板坐标系到世界坐标系的pitch角
  cv::Mat tvec = armor.tvec_camera_armor;
  cv::Mat rvec = armor.rvec_camera_armor;
  cv::Mat rMat;
  tdttoolkit::Rodrigues(rvec, rMat);

  // std::pair<cv::Vec3f, cv::Vec3f> euler = Rotation2Euler(rMat);

  // ZLL-YYDS: horizonTheta取值待定

  // float horizonTheta = euler.first[0];  //装甲板到水平面的夹角 [-pi/2 ,
  // +pi/2]

  float horizonTheta =
      armorRecvPitch;  // 这种是直接给定值（机械结构），测试阶段先用定值，计算值基本不准
  cv::Mat z =
      (cv::Mat_<float>(3, 1) << 0., -sin(horizonTheta), cos(horizonTheta));

  float radius = roborts[armor.armorType].robotsAxis_.getRadius(armor.armorType,
                                                                armor.armorTag);

  //得到水平补偿下的轴心
  cv::Point3f axisPoint = tdttoolkit::MatToPoint3f(rMat * z) * radius +
                          tdttoolkit::MatToPoint3f(tvec);

  axisPoint.y = tvec.at<float>(1, 0);

  return axisPoint;
}

cv::Point3f armor_predict::calcAxisBy_normalIntersection(Armor armor1,
                                                         Armor armor2) {
  cv::Point3f axisPoints = cv::Point3f(0, 0, 0);

  cv::Mat rvec1 = armor1.rvec_camera_armor;
  cv::Mat rvec2 = armor2.rvec_camera_armor;
  cv::Mat rMat1, rMat2;

  tdttoolkit::Rodrigues(rvec1, rMat1);  // 得到旋转矩阵
  tdttoolkit::Rodrigues(rvec2, rMat2);

  cv::Point3f tvec1 = tdttoolkit::MatToPoint3f(
      armor1.tvec_camera_armor);  // 得到两个装甲板的世界坐标
  cv::Point3f tvec2 = tdttoolkit::MatToPoint3f(armor2.tvec_camera_armor);

  cv::Mat tool = (cv::Mat_<float>(3, 1) << 0, 0, 1.);

  cv::Point3f tvec1_ =
      tdttoolkit::MatToPoint3f(rMat1 * tool + armor1.tvec_camera_armor);

  cv::Point3f tvec2_ =
      tdttoolkit::MatToPoint3f(rMat2 * tool + armor2.tvec_camera_armor);

  float k1, k2;  // 两个向量的斜率k
  k1 = (tvec1_.x - tvec1.x) / (tvec1_.z - tvec1.z);
  k2 = (tvec2_.x - tvec2.x) / (tvec2_.z - tvec2.z);

  axisPoints.x =
      ((tvec1_.z - tvec2_.z) * k1 * k2 + k1 * tvec2_.x - k2 * tvec1_.x) /
      (k1 - k2);
  axisPoints.z = (axisPoints.x - tvec2_.x) / k2 + tvec2_.z;

  //   AxisPoint.y=0;
  axisPoints.y = 0.5 * (armor1.tvec_camera_armor.at<float>(1, 0) +
                        armor2.tvec_camera_armor.at<float>(1, 0));

  // todo: 未加入轴长比例计算优化

  float radius1 = sqrt(
      pow(axisPoints.x - tvec1.x, 2) +
      pow(axisPoints.z - tvec1.z, 2));  //计算得到的轴长,将其比例纳入约束范围

  float radius2 =
      sqrt(pow(axisPoints.x - tvec2.x, 2) + pow(axisPoints.z - tvec2.z, 2));

  return axisPoints;
}

cv::Point3f armor_predict::correctAxis_2_0(  //此处坐标系均为相机坐标系
    std::vector<cv::Point3f> &axisPoints, int roboType) {
  cv::Point3f axisPoint = axisPoints.back();

  if (axisPoints.size() == 1) {  // single Armor correct

    float yaw_dp =
        getRobotYawInCamera(roboType);  //深度学习给的车像素中心点算出的yaw角

    if (yaw_dp == FLT_MAX) {
      return axisPoint;
    }
    tdttoolkit::Polar3f axisPolar = tdttoolkit::RectangularToPolar(axisPoint);

    if (fabs(sin(yaw_dp - axisPolar.yaw)) * axisPolar.distance >
        robotLength) {  //不进行矫正
      return axisPoint;
    }
    axisPolar.yaw = yaw_dp;
    //转换为直角坐标系

    axisPoint = tdttoolkit::PolarToRectangular(axisPolar);

    return axisPoint;
  }

  //三个点
  else {  // double Armors correct ，不选择圆，而是三个点的最小外接矩形

    cv::Point3f axis_aver =
        (axisPoints[0] + axisPoints[1] + axisPoints[2]) / 3.;

    float max_x = std::max({axisPoints[0].x, axisPoints[1].x,
                            axisPoints[2].x}),  //确定取值范围
        min_x = std::min({axisPoints[0].x, axisPoints[1].x, axisPoints[2].x}),
          max_z = std::max({axisPoints[0].z, axisPoints[1].z, axisPoints[2].z}),
          min_z = std::max({axisPoints[0].z, axisPoints[1].z, axisPoints[2].z});
    //构建最小外接矩形

    //构建评价方程，进行ceres优化,得到矫正后轴心的x,z值

    double x = axis_aver.x;
    double z = axis_aver.z;

    problem.AddResidualBlock(cost_function, nullptr, &x, &z);
    //未定义上下界范围

    problem.SetParameterLowerBound(&x, 0, min_x);
    problem.SetParameterUpperBound(&x, 0, max_x);
    problem.SetParameterLowerBound(&z, 0, min_z);
    problem.SetParameterUpperBound(&z, 0, max_z);

    ceres::Solve(options, &problem, &summary);

    axisPoint.x = x;
    axisPoint.z = z;
    //得到矫正后的直角坐标
    axisPoint.y = 0.5 * (axisPoints[0].y + axisPoints[1].y);

    return axisPoint;
  }

  return cv::Point3f(0, 0, 0);
}

// clang-format off
std::pair<cv::Vec3f, cv::Vec3f> armor_predict::Rotation2Euler(
    cv::Mat rotation) {
      //参考网站：https://blog.csdn.net/weixin_39675633/article/details/103434557
      cv::Vec3f euler1 = {0,0,0};
      cv::Vec3f euler2 = {0,0,0};

      if(rotation.at<float>(3,1)!=1 && rotation.at<float>(3,1)!= - 1){
          // y
          euler1[1] = -asin(rotation.at<float>(3,1));
          euler2[1] = CV_PI - euler1[1];

          // x

          euler1[0] = atan2(rotation.at<float>(3,2)/cos(euler1[1]),rotation.at<float>(3,3)/cos(euler1[1]));
          euler2[0] = atan2(rotation.at<float>(3,2)/cos(euler2[1]),rotation.at<float>(3,3)/cos(euler2[1]));

          // z
          euler1[2] = atan2(rotation.at<float>(2,1)/cos(euler1[1]),rotation.at<float>(1,1)/cos(euler1[1]));
          euler2[2] = atan2(rotation.at<float>(2,1)/cos(euler2[1]),rotation.at<float>(1,1)/cos(euler2[1]));
      }
      else{
            euler1[2] = 0 ; 
            if(rotation.at<float>(3,1) == -1){
                euler1[1] = CV_PI/2;
                euler1[0] = euler1[2] + atan2(rotation.at<float>(1,2),rotation.at<float>(1,3));
            }
            else{
                euler1[1] = -CV_PI/2;
                euler1[0] = -euler1[2] + atan2(-rotation.at<float>(1,2),-rotation.at<float>(1,3));
            }
      }

      return std::make_pair(euler1,euler2);

}

float armor_predict::getRobotYawInCamera(int roboType){
   
   if(roborts[roboType].robotCenterInImage==cv::Point2f(0,0)||!spinModeByDeepLearning){
      return FLT_MAX;
   }
      float yaw_dp=0;  //深度学习给的车像素中心点算出的yaw角
    cv::Mat axisInPixel =
        (cv::Mat_<double>(3, 1)
             << (double)roborts[roboType].robotCenterInImage.x,
         (double)roborts[roboType].robotCenterInImage.y,
         1.);  // 轴心在像素坐标系下的坐标

    // 此处矩阵为64F,务必保持矩阵数据结构的类型一致性
    cv::Mat new_axisInCamera =
        (cameraMatrix.inv() * axisInPixel);  // 轴心在相机坐标系下的单位坐标
                                             // cv::Point3f= (Xc/Zc,Yc/Zc,1)
                                             float x = (float)new_axisInCamera.at<double>(0,0);
                                             float y = (float)new_axisInCamera.at<double>(1,0) ; 
                                             float z = (float)new_axisInCamera.at<double>(2,0) ; 
    cv::Point3f axis = cv::Point3f(x,y,z);
        float a = atan(axis.x/axis.z);

    yaw_dp = tdttoolkit::RectangularToPolar(
                 axis)
                 .yaw;


       std::cout<<"yaw= "<<yaw_dp<<"  "<<axis <<std::endl;      
    return yaw_dp;             
  }
// clang-format on

cv::Point3f armor_predict::camera_TO_world(cv::Point3f axisPoint_corrected , bool ifUseTvec) {
  cv::Vec3f euler_camera_to_world = {-platForm.pitch, -platForm.yaw, 0};
  cv::Vec3f euler_world_to_camera = -euler_camera_to_world;

  cv::Mat rotation_matrix_camera_to_world =
      tdttoolkit::EulerAnglesToRotationMatrix(euler_camera_to_world, true);

  cv::Mat rotation_matrix_world_to_camera =
      tdttoolkit::EulerAnglesToRotationMatrix(
          euler_world_to_camera,
          true);  // Z - Y - X true);  // Z - Y - X
  cv::Mat tvec_world_to_camera =
      rotation_matrix_world_to_camera * cv::Mat_<float>(TvecCameraInWorld_);

  cv::Mat matrix_axisPoint = (cv::Mat_<float>(3, 1) << axisPoint_corrected.x,
                              axisPoint_corrected.y, axisPoint_corrected.z);
  if(ifUseTvec)
  return tdttoolkit::MatToPoint3f(rotation_matrix_camera_to_world *
                                      matrix_axisPoint +
                                  tvec_world_to_camera);
  else{
    return tdttoolkit::MatToPoint3f(rotation_matrix_camera_to_world *
                                      matrix_axisPoint );
  }                                
}

cv::Point3f armor_predict::spinPoint(cv::Point3f axis, cv::Point3f armor,
                                     float theta, cv::Point3f move_axis) {
  cv::Point3f l = armor - axis;
  return cv::Point3f(
      cos(theta) * l.x - sin(theta) * l.y + axis.x + move_axis.x, armor.y,
      sin(theta) * l.x + cos(theta) * l.y + axis.z + move_axis.z);
}

void armor_predict::perception_sub(
    const perception_interface::msg::Perception2Nav &msg) {
      if(!msg.num_robots){
          return ;
      }

  std::cout<<"inPer"<<std::endl;

  precption_robot.first.clear();
  precption_robot.second = 0.;
  for (size_t i = 0; i < msg.num_robots; i++) {  //记得叫tk改坐标系
    cv::Point3f robot_center_world =
        // -tdttoolkit::MatToPoint3f(TvecCameraInWorld_) +
        // // cv::Point3f(msg.robots[i].center.x, msg.robots[i].center.y,
        // //             msg.robots[i].center.z);
             cv::Point3f(msg.robots[i].center.x, 0,
                    msg.robots[i].center.y);

    precption_robot.first.push_back(
        std::make_pair(robot_center_world, msg.robots[i].class_id));
  }
  precption_robot.second =
      msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;

    ////////////////////////////////
      static double lastRecvTime = precption_robot.second ;

      static bool first_run = true ;
      if(getRosTime() - lastRecvTime <perceptionTime && !first_run ){
          return ;
      }
      first_run = false ;
      std::vector<float> percption_yaw;
      int heroIndex = -1;
      for (size_t i = 0; i < precption_robot.first.size(); i++) {
        if (precption_robot.first[i].second == tdttoolkit::RobotType::HERO) {
          heroIndex = i;
          break;
        }
        percption_yaw.push_back(
           fabs( tdttoolkit::RectangularToPolar( precption_robot.first[i].first).yaw));
      }

      int minPosition = (heroIndex != -1 ? heroIndex  //此处进行感知决策
                                         : min_element(percption_yaw.begin(),
                                                       percption_yaw.end()) -
                                               percption_yaw.begin());
    // decision_yaw =  -atan2(precption_robot.first[minPosition].first.x,precption_robot.first[minPosition].first.z) ; //世界系的yaw
    
    decision_yaw = tdttoolkit::RectangularToPolar(camera_TO_world(cv::Point3f(precption_robot.first[minPosition].first.x,0,precption_robot.first[minPosition].first.z),false)).yaw ; 

    lastRecvTime = getRosTime();

    std::cout<<decision_yaw<<"  "<<lastRecvTime<<std::endl;
 }




  // perciation_yaw_his = recvMsg.yaw  ;

}

  // namespace armor_predict