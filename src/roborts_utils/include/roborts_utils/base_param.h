/*
 * @Name: LoadParam
 * @Description: 读取参数程序头文件,使用时直接包含此头文件，！！使用方法：
 * TDTParam::robot_param.(函数名)
 * @Version: 1.0.0.2
 * @Author: 严俊涵
 * @Date: 2023-01-03 16:34
 */
#ifndef __BASE_PARAM_H
#define __BASE_PARAM_H

#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <sys/file.h>

#include <fstream>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <vector>

#include "base_msg.h"

class JsonParam {
 public:
  void Init(std::string path);

  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void ReadTheParam(const std::string &check_name, std::string &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void ReadTheParam(const std::string &check_name, int &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void ReadTheParam(const std::string &check_name, double &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void ReadTheParam(const std::string &check_name, float &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void ReadTheParam(const std::string &check_name, bool &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void ReadTheParam(const std::string &check_name, cv::Mat &param);

  /**
   * @name WriteTheParam
   * @brief 更改程序中对应的参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [in] param 变量的值
   */
  void WriteTheParam(const std::string &check_name, const int &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void WriteTheParam(const std::string &check_name, const float &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void WriteTheParam(const std::string &check_name, const double &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void WriteTheParam(const std::string &check_name, const std::string &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void WriteTheParam(const std::string &check_name, const bool &param);
  /**
   * @name ReadTheParam
   * @brief 读取参数
   * @param [in] check_name 变量在config文件中的名字
   * @param [out] param 将结果保存在该变量中
   */
  void WriteTheParam(const std::string &check_name, const cv::Mat &param);

  /**
   * @name OutPutParam
   * @brief 保存参数到文件
   */
  void OutPutParam();

  inline Json::Value GetJsonValue() { return jsonValue; }

 private:
  Json::Value jsonValue;
  std::string path_;

  Json::Value empty_node;

  Json::Value *FindValue(const std::string &check_name);

  template <class T>
  void JsonReadParam(const std::string &check_name, T &param) {
    auto node = FindValue(check_name);
    if (node == nullptr) {
      TDT_ERROR("未找到参数%s", check_name.c_str());
    } else {
      param = node->as<T>();
    }
  };

  template <class T>
  void JsonWriteParam(const std::string &check_name, const T &param) {
    auto node = FindValue(check_name);
    if (node == nullptr) {
      TDT_ERROR("未找到参数%s", check_name.c_str());
    } else {
      Json::Value temp = param;
      node->swapPayload(temp);  // 只有通过这种方法赋值才能保留注释
    }
  };
};

class LoadParam {
 private:
  static std::map<std::string, JsonParam> params;

 public:
  inline static void InitParam(const std::string &param_name,
                               std::string path) {
    params[param_name] = JsonParam();
    params[param_name].Init(path);
  }
  template <class T>
  inline static void ReadParam(const std::string &param_name,
                               const std::string &check_name, T &param) {
    params[param_name].ReadTheParam(check_name, param);
  }
  template <class T>
  inline static void WriteParam(const std::string &param_name,
                                const std::string &check_name, const T &param) {
    params[param_name].WriteTheParam(check_name, param);
  }
  inline static void OutPutParam(const std::string &param_name) {
    params[param_name].OutPutParam();
  }
  inline static void OutPutAllParam() {
    for (auto &param : params) {
      param.second.OutPutParam();
    }
  }

  inline static int Size() { return params.size(); }

  /****
   * @brief 通过字符串数据更新参数
   * @param [in] param_name 参数命名空间
   * @param [in] check_name 参数名
   * @param [in] type 参数类型
   * @param [out] value 字符串参数值
   */
  template <class T>
  static void WriteParamString(const std::string &param_name,
                               const std::string &check_name,
                               const std::string &type, const T &value) {
    if (type == "int") {
      int value_int = std::stoi(value);
      WriteParam(param_name, check_name, value_int);
    } else if (type == "float") {
      float value_float = std::stof(value);
      WriteParam(param_name, check_name, value_float);
    } else if (type == "double") {
      double value_double = std::stod(value);
      WriteParam(param_name, check_name, value_double);
    } else if (type == "bool") {
      bool value_bool = (bool)std::stoi(value);
      WriteParam(param_name, check_name, value_bool);
    } else if (type == "cv::Mat") {
    } else {
      TDT_WARNING("type error");
    }
  }

  static Json::Value GetJsonValue(const std::string &param_name) {
    return params[param_name].GetJsonValue();
  }
};  // namespace LoadParam

#endif  // _TDT__LOAD_PARAM_H