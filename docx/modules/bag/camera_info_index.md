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
