# DECODE 手动程序

这套代码基于 FTC SDK 11.1、PedroPathing 2.1.2 和 SDK 自带的
`GoBildaPinpointDriver`。主程序有两个固定联盟版本：

- `DECODE TeleOp - RED`
- `DECODE TeleOp - BLUE`

所有尚未实测的参数都集中在
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode/config/DecodeConfig.java`
和 `Alliance.java` 中，并带有 `TODO`。不要把占位值直接用于正式比赛。

## 操作

驾驶员 1：

- 左摇杆：场地中心驾驶
- 右 trigger：向一个方向旋转
- 左 trigger：向另一个方向旋转；任一 trigger 有输入时立即取消自瞄
- `A`：开启底盘自瞄并持续锁定
- `B`：取消自瞄
- 按住左 bumper：慢速模式
- 按住 `START` 再点 `X`：把 Pinpoint 当前位姿重置为点位 1
- 按住 `START` 再点 `Y`：把 Pinpoint 当前位姿重置为点位 2

副操作手：

- `Y`：飞轮开/关
- `A`：请求发射；若速度未到，程序会等待，速度合格后自动发射
- 按住右 bumper：吸球
- 按住左 bumper：反吐
- 远点时点 dpad 左/右：水平瞄准偏移每次减/加 0.25 度
- 按下左摇杆：把当前联盟的远点偏移清零

红蓝偏移分别保存在 Robot Controller 的本地设置中，切换或重启 OpMode 后仍然存在。

Control Hub IMU 按 Logo 朝右、USB 朝上初始化，显示在 Telemetry 中作为诊断数据。实际
场地中心驾驶和自瞄使用 Pinpoint/Pedro heading，确保手动重置 Pose 后驾驶坐标和自瞄坐标
仍然属于同一套参考系。

## 飞轮 PIDF

两个飞轮接收完全相同的功率，`ShooterL` 是唯一编码器反馈源，`ShooterR` 是无反馈的从电机。
运行 `Motor PIDF Tuner (Panels)`，连接机器人 Wi-Fi 后访问
`http://192.168.43.1:8001`。在 Panels 中可以实时调整：

- `TARGET_TICKS_PER_SECOND`
- `KP`、`KI`、`KD`、`KF`
- `INTEGRAL_POWER_LIMIT`
- `ENABLED`

Panels 会显示目标速度、ShooterL 实际速度、误差和左右共享功率。调好后把数值复制到
`DecodeConfig.SHOOTER_KP/KI/KD/KF`。该结构只能保证左右收到相同命令，不能检测或修正
ShooterR 单独产生的转速差。

## 自动阶段结束位姿交接

SDK 11.1 提供的 `blackboard` 会跨 OpMode 保存数据。在每个自动程序中加入：

```java
import org.firstinspires.ftc.teamcode.decode.config.Alliance;
import org.firstinspires.ftc.teamcode.decode.localization.AutoPoseHandoff;

@Override
public void stop() {
    if (follower != null) {
        AutoPoseHandoff.save(blackboard, follower.getPose(), Alliance.RED);
    }
    Algorithm.stopShoot();
}
```

蓝方自动把最后一个参数改成 `Alliance.BLUE`。TeleOp 初始化时会用这个结束位姿重新设置
Pinpoint，然后 Pinpoint 的编码器增量会继续更新实时位置。如果自动没有运行、联盟不匹配、
Robot Controller 重启或重新下载过程序，TeleOp 会使用 `Alliance.java` 中的 fallback
占位点；此时必须用两个场地点位之一手动重置。

## 如何打表

1. 在正式场地上选 5–8 个常用射击距离，近、中、远都要覆盖；用卷尺量“机器人定位中心到代码
   中球门瞄准点”的水平距离。
2. 运行 `DECODE Shot Table Tuner`，将机器人放在第一个距离，按 `B/X` 把屏幕距离标签调到
   相同数值。
3. 点 `Y` 开飞轮。用 dpad 上/下调目标速度，用 dpad 左/右调俯仰位置。
4. 点 `A` 试射。每个参数至少连续试射 5 个球；只记录稳定命中而不是偶然命中的组合。
5. 点 `START`，遥测中的 `COPY THIS ROW` 会给出：

   ```java
   {距离inch, 速度ticks/s, rs俯仰servo位置},
   ```

6. 按距离从小到大把各行填进 `DecodeConfig.SHOT_TABLE`。程序会在相邻行之间做线性插值，
   超出表格范围时使用最近一行，不会危险地无限外推。
7. 换满电和比赛常用电量各复测一次。若两种电量差异明显，先用 Panels 调好共享功率
   PIDF，再重新打表。

## 上场前必须完成

1. 确认 Hardware Map 中存在：
   `LeftFrontDrive`、`LeftBackDrive`、`RightFrontDrive`、`RightBackDrive`、
   `Intake`、`ShooterL`、`ShooterR`、`rs`、`hao`、`pinpoint`、`imu`。
2. 用 Pedro `Tuning` 中的 Localization Test 检查：前进时 X 增加，向左平移时 Y 增加，
   逆时针旋转时 heading 增加。
3. 测量并填写 Pinpoint 两个 pod 偏移。
4. 实测红蓝球门瞄准坐标、两个重置点和 shooter 后向安装角。
5. 调整底盘自瞄 `AIM_KP`、`AIM_KD`，确认不会震荡。
6. 确认 `hao=0.63` 确实关闭、`hao=1.0` 确实打开，并实测发射持续时间。
7. 确认 `rs` 在 `0.20–0.52` 内运动安全，其中 `0.20` 最竖直。
8. 确认吸球 `-0.8`、反吐 `0.4`、发射送料 `-1.0` 的实际方向正确。
9. 完成距离打表，删除所有相关 `TODO`。
