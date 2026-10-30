# sensor_names.hpp：统一的已解析话题名称校验

## 编码前规划与自审

核查发现 TF 核心与 ROS 适配对来源话题的检查不一致，CameraInfo 还维护重复规则。修复采用一个无 ROS、无分配的 header-only 函数，三个调用点复用；不增加业务对象或单独共享库，不执行命名修正/remap。

## 文件与接口

文件 include/vln_memory/bag/common/sensor_names.hpp，安装路径保持不变；通过各核心目标已有的公开 include 路径可用。

```cpp
bool vln_memory::isResolvedTopicName(std::string_view topic) noexcept;
```

参数为完整、调用期间有效的字节视图，不要求 NUL 结尾；返回 true 表示绝对名称语法与 Humble 长度限制通过，不证明话题实际存在、消息类型正确或来源可信。函数不保存引用、不分配内存，可并发使用。

## 规则

- 名称以 / 开头，有至少一个非空 token，不能以 / 结尾或含 //。
- token 只含 ASCII 字母、数字和下划线，不能以数字开头；_hidden、__stream 合法。
- 空白、NUL、非 ASCII 字节、~、替换表达式、URL 前缀均拒绝，不静默清理。
- 最长 247 字节，采用目标 ROS 2 Humble 的 RMW 限制；247 接受、248 拒绝。该实现检查的是已经展开/解析的名称，不是待 remap 的表达式。

校对依据：[Humble RMW 实现](https://github.com/ros2/rmw/blob/humble/rmw/src/validate_full_topic_name.c) 与 [长度常量](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/validate_full_topic_name.h)。未来变更 ROS 版本或名称长度策略时，应同时更新函数、回归与文档。

## 三处调用与返回语义

| 调用处 | 失败结果 |
| --- | --- |
| TfIndex::insert 的来源校验 | missing_provenance；不插入、不增加边/样本、不制造拓扑阻断 |
| ingestRosTf 的 binding 校验 | invalid_binding；不处理消息内条目 |
| decodeCameraInfo / checkCameraInfoImage 的 topic/binding 校验 | invalid_topic 或既有匹配错误，继续使用原接口约定 |

不改变已有接口签名、枚举或成功谓词。合法名称和 remap 仍可使用；超过 247 字节的名称现在在所有入口一致拒绝，这是显式输入约束收紧。

## 使用示例

```cpp
#include <vln_memory/bag/common/sensor_names.hpp>

int main()
{
  using vln_memory::isResolvedTopicName;
  return isResolvedTopicName("/robot_1/tf") && !isResolvedTopicName("/bad topic") ? 0 : 1;
}
```

回归见 [test_sensor_names](tests/test_sensor_names.md)。真实 ROS 测试还与 rmw_validate_full_topic_name_with_size 对照，含 NUL、隐藏名称与长度边界，不给核心添加 ROS 运行依赖。
