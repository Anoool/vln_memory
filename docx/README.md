# vln_memory 文档导航

当前处于 **S3 几何与工作库阶段**。已实现 preparation 导出与回读、正式观测发布与索引回读、深度原图区域投影，以及物品和空间的创建、SQLite 多观测追加和显式补定位。可用 `memory_semantic_build` 接收结构化结果，创建实体并保存归属和距离关系。详见 [语义构建入口](modules/engine/semantic_build.md)。

| 从哪里开始 | 入口 |
| --- | --- |
| 安装并从录包发布正式观测 | [项目 README](../README.md) |
| 理解层次、调用链与产物边界 | [当前架构](project/architecture.md) |
| 查看后续任务与原设计验收范围 | [系统规划](project/system_plan.md#102-后续任务与依赖)、[第 1、2 部分对照](project/sections_1_2_scope.md) |
| 使用安装包和公开目标 | [CMake 构建与导出](project/CMakeLists.md)、[头文件路径](project/header_include_paths.md) |

## 从录包到实体

| 层 | 模块与接口 |
| --- | --- |
| 录包配置与准备 | [配置](config/vln_memory.md)、[BagBuilder API](interfaces/bag_builder_api.md)、[bag_builder CLI](modules/bag/bag_builder_main.md)、[录包读取](modules/bag/bag_reader_ros.md)、[压缩图像解码](modules/bag/image_decoder_ros.md) |
| 配对与标定来源 | [RGB-D 配对](modules/bag/rgbd_sync.md)、[标定索引](modules/bag/camera_info_index.md)、[准备层接口总览](interfaces/observation_preparation_api.md) |
| preparation 回读与存储 | [PreparationReader](modules/bag/preparation_reader.md)、[共享元数据校验](modules/bag/preparation_metadata.md)、[精简转换](modules/bag/preparation_compactor.md)、[媒体写入](modules/bag/media_writer.md)、[JSONL 契约](modules/bag/jsonl_writer.md)、[目录发布](modules/bag/preparation_storage.md) |
| 选帧与观测 | [离线输入、关键帧及分批](modules/engine/build_input.md)、[观测组装](modules/engine/observation_builder.md)、[正式 Observation](modules/memory/observation.md)、[参考时刻](project/observation_time_contract.md)、[数据集发布及便携模式](modules/engine/observation_dataset.md)、[数据集读取器](modules/engine/observation_dataset_reader.md) |
| 几何投影 | [单点](modules/depth/depth_projection.md)、[深度原图区域](modules/depth/depth_region_projection.md)、[正式观测回读与投影](modules/engine/observation_depth_region.md) |
| 实体 | [Entity 数据契约](modules/memory/entity.md)、[物品与空间创建](modules/engine/entity_builder.md)、[SQLite 实体工作库](modules/memory/entity_store.md)、[观测集写入接入](modules/engine/observed_entity_store.md) |

## 传感器基础

| 模块 | 核心与适配 |
| --- | --- |
| 共用类型 | [采样时间和图像网格](common/sensor_types.md)、[时间比较](common/sensor_time.md)、[话题名称](common/sensor_names.md) |
| RGB | [核心](modules/rgb/rgb.md)、[ROS Image 适配](modules/rgb/rgb_ros.md) |
| Depth | [核心](modules/depth/depth.md)、[ROS Image 适配](modules/depth/depth_ros.md) |
| CameraInfo | [原始标定](modules/camera_info/camera_info.md)、[ROS 适配](modules/camera_info/camera_info_ros.md)、[有效相机模型](modules/camera_info/camera_model.md) |
| TF | [历史索引和链](modules/tf/tf.md)、[数值位姿](modules/tf/tf_pose.md)、[ROS 适配](modules/tf/tf_ros.md) |

## 验收与历史记录

- 现有测试入口和安装后验证方法见 [构建说明](project/CMakeLists.md#验证)。实体正向与失败场景见 [实体创建说明](modules/engine/entity_builder.md)。
- 观测链路的阶段验收见 [输入组织](reviews/build_input_verification.md)、[Reader 组装元数据](reviews/preparation_reader_assembly_metadata_verification.md)、[正式观测](reviews/observation_verification.md)。
- 录包和几何的阶段验收见 [BagBuilder 磁盘回归](reviews/bag_builder_v1_verification.md)、[时间配对](reviews/rgbd_nearest_pairing_verification.md)、[相机模型](reviews/camera_model_verification.md)、[CDR 存储](reviews/cdr_storage_verification.md)。
- 原始录包审计和空间占用记录见 [camera_record 提取审计](reviews/camera_record_extraction_audit.md)、[产物清理](reviews/build_cleanup_20261103.md)。

`project/` 保存当前架构、范围和规划，`interfaces/` 汇总公开接口，`modules/` 与源码模块对应，`reviews/` 保留各阶段的验收结论。指向源码、配置及测试的相对链接以源码检出目录为准。
