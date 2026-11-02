# camera_info_index.hpp / camera_info_index.cpp

## 职责

[头](../../../include/vln_memory/bag/builder/camera_info_index.hpp) 定义 BagCameraInfoRecord、CameraInfoSelection、CameraInfoIndex；[实现](../../../src/bag/builder/camera_info_index.cpp) 只负责候选检索、模型变化/同刻歧义分析，最终校验复用 checkCameraInfoImage。

## 接口与输入

CameraInfoIndex(const vector<BagCameraInfoRecord>& records) 借用同一 clock/segment 的不可变记录；索引使用期间不得修改/销毁 records。构造按话题排序索引，并一次分析 K/D/R/P、frame、ROI/binning 变化和同刻冲突；额外空间 O(C)。

select(CameraImageReference image, CameraInfoMatchOptions options) → CameraInfoSelection，返回 code/message/calibration_id/match。先 matched() 再消费；成功只说明候选及元数据匹配，不是有效内参计算。

## 选择规则

逐帧策略二分取最近，等距取较早时间，同刻取最早来源顺序；使用独立标定时间容差，标定可复用多次。同刻不同模型返回 ambiguous_calibration；最近候选坏则 invalid_camera_info，不回退旧模型。

static_calibration 必须段内模型一致，否则 static_calibration_changed。非法标定时间使版本边界未知，整组返回 invalid_calibration_stamp；不存在话题返回 missing_calibration。其他错误保留核心 CameraInfoMatchCode。

排序 O(C log C)，逐图查找 O(log C) 加固定大小模型/畸变检查，不逐图扫描所有标定。独立测试见 [test_camera_info_index](tests/test_camera_info_index.md)。

## 2026-11-03 原始与派生标定

BagCameraInfoRecord.data、validation_code/message 是原始值与严格校验。normalized 为可选拥有型派生数据，另存 normalized_code、normalization_applied、compatibility。reader 一次解释并保存；索引仍按原始数据检查同刻冲突及全段变化，避免抹去来源/版本边界。

选择时只有记录与调用配置策略一致才使用已保存派生数据，否则重新由原值执行调用者策略。匹配后 CameraInfoSelection.info 借用该候选的可用字段，normalization_applied 记录来源转换；其生命周期随不可变 records。不能缓存到 scan 销毁后。模型仍须根据实际图像域/网格构建。

直接调用、reader 和索引的策略一致性有核心及序列化集成回归，严格调用者不能因为缓存了派生 D5 而绕过策略。
