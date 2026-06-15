---
name: data-flow-architecture
description: SonarTracker715 完整数据流向分析 — 从数据输入到UI展示的三层管道
metadata:
  type: reference
---

# SonarTracker715 数据流向架构

## 整体三层管道

```
数据输入层 (.raw文件 / UDP网络包) → 核心算法层 (DspWorker QThread) → UI展示层 (MainWindow 主线程)
```

## 一、数据输入层 (两种模式)

**离线文件模式** (`WorkMode::MODE_FILE`):
- `.raw` 文件 (float32二进制) → `RawReader::read_raw_file()` → `Eigen::MatrixXd (M × NFFT_R)`
- 按时间戳排序，每帧读取对应文件

**UDP实时模式** (`WorkMode::MODE_UDP`):
- 网络UDP包 (自定义包头 `PacketHeader`: magic=0xAA55AA55, frameIdx, chunkIdx, totalChunks, shipLon/Lat/Heading)
- `UdpReceiver` 线程: 接收分片 → 按 frameIdx 拼包重组
- 推入 `DataBuffer` (线程安全环形队列, `QQueue<QByteArray>` + `QMutex` + `QWaitCondition`)
- `DspWorker::run()` 中 `popData()` 阻塞取出 → `reinterpret_cast<const float*>` → `Eigen::Map` → `cast<double>()`

## 二、核心算法管道 (DspWorker::run() 主循环, 每帧10步)

**步骤1: 滑动窗叠加** — `signal_w = [旧数据(NFFT_WIN-NFFT_R) | 新数据(NFFT_R)]`, 维度 M×NFFT_WIN (512×30000)

**步骤2: CBF 宽带波束形成** — `CBFProcessor::process(signal_w)` → `CBFResult`:
- `P_cbf_spatial`: VectorXd (257,) 1D空间功率谱
- `P_out`: MatrixXd (257 × f_num) 2D方位-频率LOFAR谱
- `signal_fft_lofar`: MatrixXcd (M × f_num) 复频域数据(LOFAR频段)
- `signal_fft_demon`: MatrixXcd (M × demon_f_num) 复频域数据(DEMON频段)

**步骤3: DCV Richardson-Lucy 反卷积** — 每频率bin并行(OpenMP):
- `RL_1D(P_f, precomputed_PSF, 20次迭代)` → 锐化空间谱
- `P_dcv_out`: MatrixXd (257 × f_num)
- `p_dcv_1d = rowwise sum` → VectorXd (257,) 1D高分辨空间谱

**步骤4: 自适应方位峰值检测**:
- `p_dcv_1d` → 中值滤波背景估计(MAD) → `threshold = max(mean_bg + 5*std_bg, 0.02*max_beam)`
- `findPeaks()` → `detected_angles[]`, `theta_index_current[]`

**步骤5: 多目标M/N航迹关联** — `TrackManager::updateTracks()`:
- 最近邻关联 (波门=6°)
- M/N逻辑确认 (累积命中10次确认)
- 航迹熄火检测

**步骤6: LOFAR瞬时线谱提取** (每确认目标):
- 该目标DCV能量谱 → 中值滤波背景 → SNR阈值 → 峰值提取
- → `TargetTrack.lineSpectra`, `lofarFullLinear`

**步骤7: DCV累积线谱** (每目标):
- 逐帧累加 → `accumDcvSum/accumDcvCount` → dB转换 → 中值滤波背景 → SNR阈值
- → `TargetTrack.lineSpectraDcv`

**步骤8: DEMON轴频检测** (每目标):
- DEMON频段波束形成 → IFFT → 平方律包络检波 → FIR低通滤波 → FFT回频域 → 峰值搜索
- → `TargetTrack.shaftFreq`, `demonSpectrum`

**步骤9: 批次汇总** (每 batchSize=40 帧):
- `emit batchFinished(batchIndex, startFrame, endFrame, batchFeatures)`

**步骤10: TPSW+DP离线分析 & 评估**:
- `detect_line_spectrum_from_lofar_change()`: TPSW背景归一化 → 动态规划线谱轨迹提取
- → `emit offlineResultsReady(offResults)`
- → `generateAndEmitEvaluation()` → `emit evaluationResultReady(evalRes)`

## 三、UI Signal → Slot 映射

| 信号 | 数据结构 | UI更新目标 |
|------|---------|-----------|
| `frameProcessed(FrameResult)` | 1D CBF/DCV谱, detected_angles, QList\<TargetTrack\> | Tab0实时态势 |
| `logReady(QString)` | 帧级检测日志文本 | 日志控制台 |
| `offlineResultsReady(QList<OfflineTargetResult>)` | rawLofarDb/tpswLofarDb/dpCounter | Tab1离线瀑布图 |
| `evaluationResultReady(SystemEvaluationResult)` | 目标数/正确率/角度/深度 | Tab2统计评估 |
| `reportReady(QString)` | 完整文本报告 | 报告控制台 |
| `processingFinished()` | — | 恢复UI控件 |

## 四、核心数据结构

- `DspConfig`: 45个DSP参数 (fs, M, d, c, r_scan, 阈值参数...)
- `FrameResult`: 每帧输出 (thetaAxis, cbfData, dcvData, detectedAngles, tracks)
- `TargetTrack`: 目标航迹 (currentAngle, lofarSpectrum, demonSpectrum, lineSpectra, shaftFreq, estimatedDepth...)
- `OfflineTargetResult`: 批次离线输出 (rawLofarDb, tpswLofarDb, dpCounter 瀑布图数据)
- `SystemEvaluationResult`: 系统评估 (totalTimeSec, confirmedTargetCount, targetEvals)
- `TargetEvaluation`: 单目标评估 (accuracy, accuracyDcv, shaftFreq, hasTruth, trueAngles, calcAngles, estimatedDepth, targetClass)

## 五、UI修改关键入口

1. UI布局定义: `MainWindow::setupUi()` (MainWindow.cpp 前半部分)
2. 实时图表更新: `MainWindow::onFrameProcessed()` (line 2763)
3. 离线瀑布图更新: `MainWindow::onOfflineResultsReady()` (line 2948)
4. 统计数据更新: `MainWindow::onEvaluationResultReady()` (line 3495)
5. 自定义控件: `NotificationWidget`, `wrapPlotWithRangeControl`
6. 主题/样式: 大量 inline `setStyleSheet()` 调用，暗黑主题

**How to apply:** 修改UI时先确定要改哪个Tab/图表，找到对应的信号槽函数，理解数据从DspWorker到该槽的完整路径后再动手。
