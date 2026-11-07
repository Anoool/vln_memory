#pragma once

#include <vln_memory/engine/build_input.hpp>

#include <cmath>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace vln_memory::cli
{
using Arguments = std::map<std::string, std::string>;
inline constexpr char selectionArguments[] =
  "--input DATASET --output NEW_DIRECTORY --purpose rgb|depth_map|prepared_rgbd --batch-size N "
  "[--distance-m M] [--yaw-deg D] [--max-step-m M] [--max-step-yaw-deg D]";

/** @brief 解析成对参数；各入口可以额外声明自己的选项。 */
inline Arguments parseArguments(int argc, char ** argv, std::initializer_list<const char *> extra = {})
{
  std::set<std::string> allowed{"--input", "--output", "--purpose", "--batch-size",
    "--distance-m", "--yaw-deg", "--max-step-m", "--max-step-yaw-deg"};
  allowed.insert(extra.begin(), extra.end());
  Arguments result;
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc || !allowed.count(argv[i]) || !result.emplace(argv[i], argv[i + 1]).second) {
      throw std::invalid_argument("Unknown, repeated or incomplete option");
    }
  }
  return result;
}

/** @brief 读取必填参数，缺项或空值报告参数错误。 */
inline const std::string & required(const Arguments & args, const char * key)
{
  const auto found = args.find(key);
  if (found == args.end() || found->second.empty()) {
    throw std::invalid_argument(std::string("Missing required option: ") + key);
  }
  return found->second;
}

/** @brief 读取有限阈值，正值和业务范围由共享选择模块检查。 */
inline double number(const std::string & text)
{
  std::size_t end = 0;
  const auto value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) {throw std::invalid_argument("Invalid number");}
  return value;
}

/** @brief 解析两个离线入口共用的用途、批大小和采样阈值。 */
inline BuildInputOptions selectionOptions(const Arguments & args)
{
  BuildInputOptions options;
  const auto & purpose = required(args, "--purpose");
  if (purpose == "rgb") {options.purpose = BuildInputPurpose::rgb;}
  else if (purpose == "depth_map") {options.purpose = BuildInputPurpose::depth_map;}
  else if (purpose == "prepared_rgbd") {options.purpose = BuildInputPurpose::prepared_rgbd;}
  else {throw std::invalid_argument("Unknown purpose");}

  const auto & batch = required(args, "--batch-size");
  if (batch.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("batch-size must be a positive integer");
  }
  const auto size = std::stoull(batch);
  if (size == 0 || size > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument("batch-size out of range");
  }
  options.batch_size = static_cast<std::size_t>(size);
  if (args.count("--distance-m")) {options.distance_m = number(args.at("--distance-m"));}
  if (args.count("--yaw-deg")) {options.yaw_rad = number(args.at("--yaw-deg")) * std::acos(-1.0) / 180;}
  if (args.count("--max-step-m")) {options.max_step_m = number(args.at("--max-step-m"));}
  if (args.count("--max-step-yaw-deg")) {
    options.max_step_yaw_rad = number(args.at("--max-step-yaw-deg")) * std::acos(-1.0) / 180;
  }
  return options;
}
}  // namespace vln_memory::cli
