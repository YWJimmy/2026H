# V5巡线稳定修复说明

## 根因

V4将巡线PD产生的左右轮目标整体送入慢速限跃度规划器。巡线误差发生变化后，目标差速不能及时建立，形成明显相位滞后：车辆越过黑线后，上一方向的差速仍在继续建立，造成左右摆动逐渐放大。

## 新控制结构

```text
line error
   ├─ base command ─ slow jerk-limited profile
   └─ turn command ─ fast slew limiter

left target  = base target + turn target
right target = base target - turn target
```

基础速度仍由`chassis_speed_profile`生成；快速转向通道位于`chassis.c`，只在`Chassis_SetLineFollowCommandMmps()`启用。

普通底盘命令、柔和停车、快速停车、换向和急停仍使用原V4规划器。

## 输入稳定处理

`line_follow_control.c`增加：

- 3帧中值滤波，抑制单帧离群；
- ±500误差死区，减少中心抖动；
- 微分变化限制±2000；
- 外环转向修正变化率5000 mm/s²；
- 启动时要求|error|≤1000连续80 ms；
- 丢线搜索速度降到100 mm/s。

## 边界保护

巡线API同时限制：

```text
0 ≤ left ≤ 260
0 ≤ right ≤ 260
```

基础速度尚未建立时，转向修正会按当前基础速度和轮速余量动态限幅，不会产生反向中间目标。

## 停车衔接

从巡线双通道进入SOFT或FAST停车前，底盘会把当前实际左右轮规划目标同步回通用S曲线规划器，再进入同比例停车，避免丢失当前差速状态。
