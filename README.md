# FPGA + RK3568 辅助驾驶视觉系统

项目基于紫光同创 PG2L100H FPGA 和瑞芯微 RK3568，实现道路视频采集、图像处理、目标检测、车道信息融合和驾驶提示。

## 目录

- `FPGA_PDS工程/`：FPGA 工程和烧录文件，使用 PDS 2022.2-SP6.4 打开 `ADAS_HDMI_PCIE_100H.pds`。
- `RK3568软件/`：板端驱动、启动界面、运行程序和模型。
- `RK3568软件_部署包.tar.gz`：保留 Linux 可执行权限的板端部署包。
- `机械结构设计/`：3D 打印底框、亚克力罩、装配模型和加工图纸，尺寸单位为毫米。

## 使用

FPGA Flash 文件：`FPGA_PDS工程/generate_bitstream/pcie_dma_test.sfc`。

在 RK3568 开发板上解压并启动：

```sh
tar -xzf RK3568软件_部署包.tar.gz
cd RK3568软件
sudo sh install_driver.sh
sudo sh start.sh
```

主界面包含巡航、环视、路口和导航四种模式。路口模式使用十类检测模型，其余模式使用六类检测模型。
