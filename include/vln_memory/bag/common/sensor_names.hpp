#pragma once

#include <cstddef>
#include <string_view>

namespace vln_memory
{

/**
 * @brief 检查已经解析的 ROS 2 Humble 绝对话题名，不修改输入，也不执行 remap。
 * @param topic 完整字节视图，支持检查含 NUL 的坏输入，无需以 NUL 结尾。
 * @return 符合绝对名称语法且不超过 Humble RMW 247 字节上限时为 true。
 * @note token 由 ASCII 字母/数字/下划线组成，不能以数字开头；/ 分隔，
 *       不允许空 token、尾部 /、空白、替换表达式或 URL 前缀。
 *       这是格式检查，不证明话题存在、消息类型正确或来源可信。
 */
inline bool isResolvedTopicName(std::string_view topic) noexcept
{
  if (topic.size() < 2 || topic.size() > 247 || topic.front() != '/' || topic.back() == '/') {
    return false;
  }
  bool token_start = true;
  for (std::size_t i = 1; i < topic.size(); ++i) {
    const char c = topic[i];
    if (c == '/') {
      if (token_start) {return false;}
      token_start = true;
      continue;
    }
    // 不使用 locale 敏感的 isalnum；ASCII 范围也会拒绝 NUL 和非 ASCII 字节。
    const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    const bool digit = c >= '0' && c <= '9';
    if (!letter && !(digit && !token_start)) {return false;}
    token_start = false;
  }
  return !token_start;
}

}  // namespace vln_memory
