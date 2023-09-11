#ifndef SRM_COMMON_CONFIG_H_
#define SRM_COMMON_CONFIG_H_

#include <glog/logging.h>

#include <opencv2/core/mat.hpp>
#include <string>
#include <variant>

#include "srm/common/tags.h"

namespace srm {
/// 参数解析类
class Config final {
  using param = std::variant<bool, int, double, std::string, std::vector<double>, cv::Mat>;

 public:
  /**
   * @brief 获取 Config 类唯一实例
   * @return 唯一实例的引用
   */
  static Config &Instance();

  /**
   * @brief 获取参数
   * @details 例如对于如下yaml文件：
    ```yaml
    core:
      fps_limit: 100.0
    ```
    在程序内可使用 @code cfg.Get<double>({"core.fps_limit"}); @endcode
    或者 @code cfg.Get<double>({"core", "fps_limit"}); @endcode 来获取值
   * @tparam T 参数的数据类型
   * @param [in] list 参数路径列表
   * @return T 参数的值
   */
  template <class T>
  T Get(std::initializer_list<std::string> FWD_IN list);

  /**
   * @brief 解析配置文件参数
   * @param config_path 配置文件路径
   * @return 是否配置成功
   * @todo 添加对复杂类型的支持，判断重复输入的变量
   * @todo 添加热更新
   * @todo 添加对下层子配置的覆盖选项
   */
  bool Parse(std::string config_file);

 private:
  /// @note 该类用于封装全局变量，故采用单例模式
  Config() = default;
  ~Config() = default;
  std::unordered_map<std::string, param> registry_;
};

template <class T>
T Config::Get(std::initializer_list<std::string> FWD_IN list) {
  std::string para_name{};
  for (auto &it : list) para_name += it + ".";
  para_name.pop_back();
  if (!registry_.count(para_name)) {
    LOG(ERROR) << para_name << " doesn't exist.";
    LOG(INFO) << this << " " << &registry_;
    LOG(INFO) << registry_.size();
    return {};
  }
  auto &para = registry_[para_name];
  T *data = std::get_if<T>(&para);
  if (data == nullptr) {
    LOG(ERROR) << para_name << "指定的类型错误";
    return {};
  } else {
    return std::move(*data);
  }
}

inline Config &cfg = Config::Instance();  ///< 封装命令行参数的全局变量

}  // namespace srm

#endif  // SRM_COMMON_CONFIG_H_