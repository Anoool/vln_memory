# CameraInfo 可读性整理

日期：2026-10-29。依据仓库 AGENTS.md，本轮整理 CameraInfo 的结构和调用方式，保持校验规则、错误码、首个错误顺序、原始字段与所有权语义。

## 编码前规划

- 输入：完整 CameraInfo、指定图像元数据和显式匹配配置；输出：原有拥有型解码结果与匹配诊断。
- 将 CameraImageReference 的 width/height/frame_id 提到顶层；将 CameraInfoMatchOptions 的话题、光学帧、依据、图像域、时钟关系和纳秒容差放在同一层，删除仅在此处包一层的 CameraInfoBinding。
- 保留 SensorStamp、原始 CameraRoi 和可选结果：它们分别表达采样时刻、ROS 原始 ROI 和失败无数据，避免为了扁平化丢失语义。
- 拆开畸变检查、ROI/binning 网格检查及匹配参数检查；主流程按原有检查顺序调用，返回第一个错误。
- 辅助函数采用能说明目的的名称，补函数注释；展开测试/消费者中的密集赋值和多层聚合初始化。
- ROS 适配仍只复制真实消息，公共时间检查仍复用 sensor_time；不引入新依赖，不调整 RGB/Depth/TF 接口。
- 一并迁移仓库内调用点、公开接口文档和完整示例。旧调用者需按字段迁移并重新编译。

## 变更与验证范围

仅修改 CameraInfo 生产代码、相关测试/安装消费者及配套文档，保留工作区既有改动。当前工作区已有其他模块的暂存修改，本轮不切换共享分支；完成后仅暂存本轮涉及文件，合并留给仓库所有者审阅。

重构前无 ROS 严格编译和核心 CTest 6/6 已通过。随后验证同一批用例、完整诊断的重构前后对照、真实 ROS 适配、安装后消费和文档示例。构造消息验证不代表真实 bag 或物理标定验证。

## 复杂度与边界

标定校验为 O(D + 名称长度)，K/R/P 和 ROI 的工作量固定；验证额外空间 O(1)。解码保留按值输入和移动输出；左值调用仍需复制字符串和 D，未新增标定深拷贝。检查时间始终使用原始整数纳秒，动态/静态策略含义不变。


## 实施完成与人工复核

- CameraImageReference 的 width/height/frame_id、CameraInfoMatchOptions 的绑定和时间参数均直接位于顶层；移除 CameraInfoBinding，无继承、重复字段或兼容包装。
- 保留 SensorStamp、CameraRoi 和可选结果。ROS ROI 使用逐字段赋值，原始值、数据所有权和时间策略不变。
- validateCameraInfo 按原顺序组织来源/数值、畸变、K/R/P 与 ROI/binning 检查；后续匹配仍依次检查配置、图像、绑定、网格和时间。
- 生产函数、辅助函数及模块测试入口补齐目的/边界注释；变量名说明矩阵角色、ROI 尺寸、binning 和系数数目。原有断言预期未放宽。
- 已迁移核心测试、安装消费者、完整文档示例、接口手册和文档导航。共用消费者保留同时发生的 Depth 扁平接口修改。

## 实际验证

1. 严格参数 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror：重构前后无 ROS 核心 CTest 均为 6/6，真实 ROS 2 Humble CTest 为 10/10，零失败/跳过。
2. 临时独立程序分别编译本轮快照与当前 CameraInfo 核心：45 组标定结果和 10,800 组匹配组合逐字节一致，覆盖全部原始输出字段、错误码/文本、多个错误的优先级、网格尺寸、时间诊断和静态策略。
3. 仅引用安装目标的独立工程严格编译通过，RGB、Depth、Depth ROS、CameraInfo、CameraInfo ROS、TF、TF ROS 共 7 个消费者均运行通过。
4. CameraInfo 文档完整示例严格编译并运行，输出 640x480；本轮变更文档的 83 个本地文件链接目标有效。生产代码和相关消费者无超过 120 字符的行。

本次直接 CMake 安装到 /tmp/vln-camera-info-readability-install，运行 ROS 消费者前加载 /opt/ros/humble/setup.bash 和安装目录 share/vln_memory/local_setup.bash，以提供包级动态库路径。该验证没有依赖未安装的源码 include 路径。

本次是公开结构源码/布局的有意调整，下游须按模块文档迁移字段并重新编译；校验行为保持原语义。测试使用构造消息，未进行真实 bag 或物理标定验证。
