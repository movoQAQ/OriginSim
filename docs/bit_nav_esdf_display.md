# bit-nav ESDF 显示对齐（2026-10-23）

依据本机 `/home/lyu/bit` 的 bit-nav，HEAD d60334aa7ece78d96bcf221123030b99e2ae0cdf：

- `navi_minco_bit/src/navigation/navi2_bringup/config/our.rviz` 的 `2DESDF`：PointCloud2、Intensity、Use rainbow=true、Invert Rainbow=false、Autocompute Intensity Bounds=true、Alpha=0.6。
- `navi_minco_bit/src/perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp` 的 fillFieldCloud：将原始距离放入 intensity，跳过非有限距离。
- [RViz IntensityPCTransformer](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.cpp)：自动求有限值范围，颜色输入为 `1-(distance-min)/(max-min)`，调用 getRainbowColor。

当前 `/map_server/esdf_grid` 改为同一距离语义：有限距离按当帧 min/max 归一化为 0..100，四舍五入；非有限值 -1，透明。全等距离使用归一化 0，和 RViz 一样显示统一的最小距离颜色；没有有限值时整层透明。插件直接调用已安装 RViz 的原生 getRainbowColor，方向与 bit-nav 一致：近红，依次黄、绿、青、蓝，远端紫色。默认透明度 0.6。

按用户要求保留 MapDisplay 的纹理栅格，不发布点云。100 现在表示距离范围的最远端，而不是硬碰撞；这是只供显示的 OccupancyGrid 编码，不能作为规划障碍图消费。删除已无用途的 visualization_max_cost 参数和显示包 OpenCV 依赖。规划继续读取原始 costmap/collision_mask，地图参数和控制器没有修改。

bit-nav 使用有符号距离，本工程仍使用已有的非负 EDT，没有为模仿显示额外计算障碍内部负距离。因此障碍内部距离统一为 0；同一距离及范围的颜色函数一致，整幅地图并非逐像素复制 bit-nav 的 signed ESDF。0..100 栅格编码的归一化误差最多 0.005。

实际运行检查见 bit_nav_live_grid.json：逐格核对距离编码、2026 YAML/PCD、0.30 m 半径、Depth 为空。此前 TDT 色表记录保留为历史，已由本说明替代。
