# test_camera_info.cpp：标定、流绑定和时间契约

源码 test/bag/camera_info/test_camera_info.cpp；目标 test_camera_info，CTest 名 camera_info_core。使用轻量异常检查，不受 NDEBUG 关闭 assert 影响。

先构造标准完整 CameraInfo，再分三组验证：

1. 标定/所有权：支持的三种模型、K/P 不同和双目平移保留、复制后无借用；拒绝未标定、负焦距、非标准 K、反射/全零 R、错误 P、模型/系数不一致、非有限数值、无效 topic/frame/stamp/dimensions。
2. 图像契约：默认 ROI、裁剪与 binning 的实际网格、不整除和越界/溢出 ROI、显式话题/frame/basis/domain、校正 ROI 的不支持状态；公开可变 CameraInfo 仍重新校验。
3. 时间：以 image 为 reference 的正时差、容差边界/超限、未知时钟拒绝、固定标定不伪造同步、静态策略仍拒绝坏 image stamp。

运行：ctest --test-dir build/camera-core -R camera_info_core --output-on-failure。它校验结构与规则，不证明物理精度，也不选择跨消息候选。接口见 [核心](../camera_info.md)。
