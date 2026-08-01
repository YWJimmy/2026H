# 路程统计设计与接入

## 数据来源

路程模块不直接访问 TIM3/TIM4，也不再次调用 `BSP_Encoder_Sample()`。
`Chassis_Update()` 完成左右编码器同步采样和合理性检查后，将同一帧有效增量传给：

```c
DistanceTracker_Update(
    encoder_sample.left_delta,
    encoder_sample.right_delta,
    encoder_sample.sequence,
    encoder_sample.timestamp_ms);
```

因此底盘闭环、路程统计和调试输出共享同一份编码器快照。

## 换算参数

- 编码器计数：1468 count/rev
- 车轮直径：65 mm
- 理论轮周长：204204 um
- 本版本不包含左右轮比例系数或实车距离标定

## 距离定义

- `center_signed_mm`：车辆中心有符号路程，前进增加、后退减少，适合任务路线进度判断。
- `traveled_mm`：左右轮绝对增量平均后累计，前进和后退均增加，适合 OLED 总路程显示和运行统计。
- `wheel_difference_mm`：左右轮有符号距离差，用于辅助诊断跑偏。

## 生命周期

- `Chassis_Init()` 初始化路程模块。
- 每次任务二次确认启动时，`AppTaskPort_Start()` 调用 `DistanceTracker_Reset()` 清零软件路程。
- `FINISHED` 页面保留最终路程；返回菜单不会清零，下一次任务启动时再清零。

## 输出

OLED 在 `RUNNING`、`STOPPING` 和 `FINISHED` 页面显示累计路程：

```text
D:1234MM
```

MainApp 周期日志增加：

```text
DIST=1234,SDIST=1228
```

直行测试日志增加：

```text
DIST,L=1000,R=998,C=999,TR=1002,DIFF=2
```
