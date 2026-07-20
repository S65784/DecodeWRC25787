# FTC DECODE 程序交接说明

更新时间：2026-07-20  
项目目录：`/Users/sunny/StudioProjects/QuickstartWRC`

这份文件是当前代码的交接入口。遇到说明与实现不一致时，以当前 Java 源码为准。

## 1. 接手后先做什么

1. 使用 Android Studio Ladybug 2024.2 或更新版本打开项目根目录。
2. 等待 Gradle Sync 完成。
3. 不要先执行 reset、checkout 或批量格式化。当前项目包含尚未提交的修改和新文件，
   先创建自己的备份分支或提交当前快照。
4. 编译一次：

   ```bash
   ./gradlew :TeamCode:assembleDebug
   ```

5. 核对 Robot Configuration 中的 Hardware Map 名称。
6. 架空底盘和飞轮，分别验证电机方向、Pinpoint 坐标方向、servo 安全范围。
7. 最后才在场地上逐段测试自动，不要第一次就完整运行。

截至 2026-07-20，当前代码执行 `:TeamCode:assembleDebug` 为 `BUILD SUCCESSFUL`。
这只说明编译通过，不代表全部路径和射击参数已经完成实机验证。

## 2. 版本与坐标系

- FTC SDK：11.1.0
- Pedro Pathing：2.1.2
- Pedro Telemetry/Panels：1.0.0
- Android Gradle Plugin：8.7.0
- `compileSdk`：34
- `minSdk`：24
- 定位：FTC SDK 的 `GoBildaPinpointDriver`，通过 Pedro `PinpointConstants` 使用
- 场地坐标范围：`0..144 inch`
- Pedro 坐标约定：`+X`、`+Y` 和 heading 正方向必须通过 Localization Test 实测确认

本项目的红蓝自动镜像规则是：

```text
Blue X       = 144 - Red X
Blue Y       = Red Y
Blue Heading = 180° - Red Heading
```

heading 超出 `0..360°` 不一定是错误；Pedro 接受等价角度。不要只为“看起来整齐”而改角度。

## 3. Hardware Map 与方向

必须存在以下名称：

| 类型 | Hardware Map 名称 |
| --- | --- |
| 左前底盘电机 | `LeftFrontDrive` |
| 左后底盘电机 | `LeftBackDrive` |
| 右前底盘电机 | `RightFrontDrive` |
| 右后底盘电机 | `RightBackDrive` |
| 吸球/送料电机 | `Intake` |
| 左飞轮 | `ShooterL` |
| 右飞轮及编码器反馈 | `ShooterR` |
| 俯仰 servo | `rs` |
| 发射屏蔽器 servo | `hao` |
| 定位模块 | `pinpoint` |
| Control Hub IMU | `imu` |

当前底盘方向在 `pedroPathing/Constants.java`：

- LeftFront：FORWARD
- LeftBack：FORWARD
- RightFront：REVERSE
- RightBack：FORWARD

当前发射机构方向在 `ShooterSubsystem.java`：

- ShooterL：FORWARD
- ShooterR：REVERSE
- ShooterR 是唯一编码器反馈源
- 两个飞轮机械连接同一根轴，软件向两边发送完全相同的功率
- 两个飞轮都使用 `RUN_WITHOUT_ENCODER`，由外部共享 PIDF 控制功率

不要再额外开启 Hub 内置 velocity PIDF，否则会和现有共享 PIDF 叠加。

## 4. 代码结构

### 手动和机构

| 文件 | 作用 |
| --- | --- |
| `decode/config/DecodeConfig.java` | 机器人硬件名、功率、PIDF、servo、安全范围、超时和射击表 |
| `decode/config/Alliance.java` | 红蓝球门目标、两个手动重置点、TeleOp fallback Pose |
| `decode/opmodes/DecodeTeleOpBase.java` | 红蓝手动共用的驾驶、自瞄、射击和键位逻辑 |
| `decode/control/AutoAimController.java` | 后射机构的底盘自瞄 PD，Panels 可实时调 `AIM_KP/AIM_KD` |
| `decode/control/SharedFlywheelController.java` | 用 ShooterR 速度闭环，给左右飞轮相同功率 |
| `decode/subsystems/ShooterSubsystem.java` | 飞轮、rs、hao、到速屏蔽和非阻塞发射状态机 |
| `decode/subsystems/IntakeSubsystem.java` | 单 Intake 的吸球、反吐、送料和定时动作优先级 |
| `decode/shooting/ShotLookupTable.java` | 距离表线性插值 |
| `decode/localization/AutoPoseHandoff.java` | 用 SDK blackboard 把自动结束 Pose 交给 TeleOp |
| `MotorPIDFTunerPanels.java` | 飞轮共享 PIDF 实时调试 |
| `ShotTableTuner.java` | 距离、速度、rs 和送料时长打表 |

### Pedro 和自动

| 文件 | 作用 |
| --- | --- |
| `pedroPathing/Constants.java` | Pedro PIDF、底盘、Pinpoint pod 偏移和路径约束 |
| `pedroPathing/Tuning.java` | Pedro 官方调试 OpMode 入口 |
| `Auto/PathStateAutoBase.java` | 自动共用的 follower、机构更新、超时和 Pose 交接 |
| `Auto/AutoPaths.java` | `line()`、`curve()` 等路径创建辅助方法 |
| `Auto/Red/*.java` | 红方各自动的 Pose、路径和状态机；当前坐标调整的主基准 |
| `Auto/Blue/*.java` | 对应红方的蓝方镜像 |
| `Auto/AutoSequence.java` | 早期序列封装，目前正式自动没有引用，不是当前主结构 |
| `tools/` | 红蓝镜像和 `.pp` 生成/检查脚本 |
| `visualizer/` | Pedro Visualizer 文件和旧检查记录 |

## 5. 当前 Driver Station 程序

### TeleOp

当前有四个手动程序：

- `DECODE RED - STICK TURN 80%`
- `DECODE RED - STICK TURN 100%`
- `DECODE BLUE - STICK TURN 80%`
- `DECODE BLUE - STICK TURN 100%`

两种模式都使用右摇杆旋转；区别只是旋转输入乘以 `0.8` 或 `1.0`。
右摇杆向右推时，代码输出顺时针旋转命令。

### 自动

当前启用：

- 红/蓝 AutoCero：近端不开门
- 红/蓝 AutoCinco：近端推门两次
- 红/蓝 AutoSiete：远端多次吸球
- 红/蓝 AutoOcho：在 Siete 逻辑中增加最后一组取球和射击

当前禁用：

- 红/蓝 AutoDos：整个 Java 内容被注释，因此不会出现在 Driver Station

不要只取消 AutoDos 的 `@Autonomous` 注释；它当前是整份源码一起注释，恢复时需要完整检查、
编译并逐段实测。

## 6. 当前 TeleOp 键位

### Gamepad 1

- 左摇杆：场地中心平移
- 右摇杆 X：旋转；有明显输入时立即取消自瞄
- `dpad_left`：保持 X/Y，IMU yaw 归零，并把 Pedro heading 设置为联盟正前方
  - 红方：0°
  - 蓝方：180°
  - 按键前车头必须实际朝远离本方 Driver Station
- `A`：开启并持续保持底盘自瞄
- `B`：取消自瞄
- `X`：飞轮开/关
- `Y`：请求发射；未到速会等待，到速后自动开 hao 并送料
- 按住右 bumper：吸球
- 按住左 bumper：反吐

蓝方场地中心平移输入会整体旋转 180°，因为两方 Driver Station 视角相反。

### Gamepad 2

- `A`：把当前 Pose 重置为联盟点位 1
- `Y`：把当前 Pose 重置为联盟点位 2
- 距离球门至少 90 inch 时：
  - `dpad_left/right`：远点水平瞄准偏移每次 `- / +0.25°`
  - `dpad_up/down`：远点目标速度每次 `+ / -50 ticks/s`
- 按下右摇杆：清零当前联盟的远点水平偏移

远点水平偏移和速度偏移通过 Android `SharedPreferences` 分联盟保存，重启 OpMode 后仍保留。
右摇杆只清水平偏移，目前没有独立按键清除速度偏移。

注意：TeleOp 实际远点门槛是 `DecodeTeleOpBase` 内的
`FAR_ADJUSTMENT_MIN_DISTANCE_INCHES = 90`，不是
`DecodeConfig.FAR_SHOT_DISTANCE_INCHES` 当前的 80。修改门槛时必须明确两者是否应该统一。

## 7. 发射和 Intake 状态机

`ShooterSubsystem.requestFire()` 不会阻塞 OpMode：

1. 自动打开飞轮并进入 `WAITING_FOR_SPEED`。
2. ShooterR 实测速度进入目标值的允许误差后：
   - `hao` 打开；
   - Intake 切到发射送料；
   - 状态进入 `FEEDING`。
3. 送料时间结束后关闭 `hao`、停止送料，回到 `IDLE`。

Intake 命令优先级：

```text
发射送料 > 定时动作 > 手动吸球/反吐
```

当前关键值请以 `DecodeConfig.java` 为准。2026-07-20 的代码快照是：

- `hao` 关闭：0.63
- `hao` 打开：1.00
- `rs` 安全范围：0.20–0.52
- Intake 吸球：-1.00
- Intake 反吐：+0.40
- 发射送料：-1.00
- 飞轮到速允许误差：±50 ticks/s
- 手动送料时长：随射击表距离在 300–420 ms 之间线性变化
- 自动近端送料：350 ms
- 自动远端送料：400 ms
- 自动等待到速超时：1000 ms
- 自动路径超时：3580 ms

不要从本文件把这些数字复制回 `DecodeConfig.java`。`DecodeConfig.java` 是唯一实时基准，
修改前后都要看 Git diff，防止覆盖现场已经调好的数。

## 8. 自瞄逻辑

射手安装方向是机器人正后方：

```text
目标底盘角 =
atan2(goalY - robotY, goalX - robotX)
- SHOOTER_DIRECTION_FROM_ROBOT_FORWARD_RAD
+ 远点水平偏移
```

`AutoAimController` 对归一化后的 heading error 使用 PD：

```text
turn = AIM_KP × error + AIM_KD × derivative
```

然后应用最大输出、最小克服静摩擦输出和角度容差。

`AIM_KP`、`AIM_KD` 在 Panels 中是实时副本。Panels 调整不会自动写回
`DecodeConfig.java`；确定最终值后必须人工抄回。

红蓝球门目标在 `Alliance.java` 中。目前目标点不是严格镜像值，不要在没有实测依据时
自动把球门坐标也做 `144-x` 镜像。

## 9. 自动程序如何修改

当前自动采用“每个文件独立 Pose + `buildPaths()` + `switch(pathState)`”结构。

修改一条自动的推荐顺序：

1. 只编辑对应红方 Java 中有清楚名称的 Pose、控制点或单轮参数。
2. 在 `buildPaths()` 中确认路径起点等于上一段终点。
3. 在 `autonomousPathUpdate()` 中确认状态顺序、Intake 开关、settle、发射和 park。
4. 编译红方。
5. 用红方生成蓝方镜像。
6. 对比 Git diff，确认没有修改任何红方文件。
7. 更新对应 `.pp`，在 Visualizer 检查。
8. 实机逐段调试。

生成全部蓝方镜像：

```bash
node tools/mirror-red-auto-to-blue.mjs \
  RedAutoCero RedAutoCinco RedAutoDos RedAutoOcho RedAutoSiete
```

这个脚本：

- 读取红方文件；
- 只写蓝方文件；
- 同步路径、状态机、动作和非坐标参数；
- 对所有 `pose(x, y, heading)` 使用 144 inch 镜像；
- AutoDos 的注释状态也会原样同步。

运行脚本会覆盖蓝方文件。若蓝方存在故意的单独微调，必须先提交或备份，并决定是否还应
使用“蓝方严格镜像红方”的维护方式。

当前 `.pp` 工具覆盖 Cero、Cinco、Dos、Siete；Ocho 还没有独立的 `.pp` 生成支持。
Java 文件永远是路径的主基准，不能让旧 `.pp` 反向覆盖 Java 坐标。

## 10. 定位与自动到手动交接

Pedro follower 使用 Pinpoint 定位。关键配置在 `Constants.java`：

- 单位：inch
- Hardware Map：`pinpoint`
- pod：goBILDA 4 Bar
- forward encoder：REVERSED
- strafe encoder：REVERSED
- forward pod Y：-0.978
- strafe pod X：-3.103

pod 偏移或安装方向改变后必须重新跑 Pedro Tuning，不能只调路径。

Localization Test 一打开显示 `(72, 72)` 可以是测试 OpMode 的默认场地中心 Pose；
真正要检查的是移动方向：

- 沿定义的机器人前方移动时，坐标变化方向是否符合 Pedro 配置；
- 左右平移是否符合坐标定义；
- 逆时针转动时 heading 是否按期望增加；
- 原地旋转时 X/Y 不应明显漂移。

每个正式自动继承 `PathStateAutoBase`。停止时会调用：

```java
AutoPoseHandoff.save(blackboard, follower.getPose(), alliance);
```

TeleOp 初始化时读取同联盟 Pose。blackboard 不会跨 Robot Controller 重启或重新下载程序
永久保存；无有效 Pose 时使用 `Alliance.teleOpFallbackPose()`，必要时用 Gamepad 2 的两个
重置点校正。

## 11. 调试工具

### 飞轮 PIDF

运行 `Motor PIDF Tuner (Panels)`，连接机器人 Wi-Fi 后访问：

```text
http://192.168.43.1:8001
```

可实时调：

- `TARGET_TICKS_PER_SECOND`
- `KP/KI/KD/KF`
- `INTEGRAL_POWER_LIMIT`
- `ENABLED`

危险提示：`ENABLED=true` 会打开 hao 并让 Intake 持续送料。架空飞轮并清空弹仓后再启动。

只有 OpMode 正在运行且 `panelsTelemetry.update()` 持续发送数据时，Graph Variables 才会
出现可选曲线。当前曲线包括目标速度、ShooterR 实速、误差、共享功率和 RPM。

### 射击表

运行 `DECODE Shot Table Tuner`：

- dpad 上/下：速度 ±25 ticks/s
- dpad 右/左：rs ±0.005
- `B/X`：距离标签 ±6 inch
- `Y`：飞轮开/关
- `A`：请求发射
- `START`：生成一行可复制的射击表
- 右/左 bumper：模拟比赛吸球/反吐

Panels 中的 `TEST_FIRE_DURATION_MS` 只对本次调试生效，不会写入配置。

## 12. 修改边界与已知风险

- 未经明确确认，不要批量重写 `DecodeConfig.java`。
- 不要为了同步蓝方而重写红方；红方现有数值可能是现场刚调好的。
- 不要用旧备份、旧 Notes 附件或旧 `.pp` 覆盖当前 Java。
- 不要在 OpMode 循环中加入 `Thread.sleep()`；现有机构和自动都是非阻塞状态机。
- AutoDos 当前禁用。
- Ocho 当前没有对应 `.pp` 自动生成工具。
- `Alliance.java` 仍有 TODO 注释；球门点和重置点需要按正式场地复核。
- `Constants.java` 仍有 Pinpoint pod 偏移 TODO；机械位置改变后必须重测。
- 共享飞轮控制只能闭环 ShooterR。虽然同轴能机械同步，但软件无法检测 ShooterL 独立故障。
- 当前工作树不是干净状态。交接前应提交一个可追溯快照，并写明实机验证到哪一步。

## 13. 上场前检查单

- [ ] Android Studio Gradle Sync 成功
- [ ] `:TeamCode:assembleDebug` 编译成功
- [ ] Hardware Map 名称全部一致
- [ ] 底盘四电机方向正确
- [ ] ShooterL/ShooterR 对转方向正确
- [ ] ShooterR 速度反馈符号为正且数值合理
- [ ] `hao=0.63/1.00` 的关/开方向正确
- [ ] `rs=0.20..0.52` 无机械干涉
- [ ] Intake 三种功率方向正确
- [ ] Localization Test 的 X/Y/heading 方向正确
- [ ] Pinpoint pod 偏移已按最终机械安装复核
- [ ] 红蓝 Gamepad 1 场地中心驾驶方向正确
- [ ] 红蓝右摇杆向右均为顺时针
- [ ] 红蓝 heading 归零分别得到 0°/180°
- [ ] 两个联盟重置点实际摆放后 Pose 正确
- [ ] 自瞄旋转方向正确且 PD 不震荡
- [ ] 远点水平和速度偏移只在 ≥90 inch 生效
- [ ] Shot Table 在比赛电量下复测
- [ ] 每条自动在 Visualizer 中检查
- [ ] 每条自动低功率、逐段实机检查
- [ ] 自动结束 Pose 能正确交给同联盟 TeleOp

## 14. 建议的交接记录

交接时让上一位程序员补充下面内容：

```text
最后一次成功编译：
最后一次实机测试日期：
已完整实测的 TeleOp：
已完整实测的自动：
只在 Visualizer 检查、尚未实测的自动：
当前比赛使用的红/蓝自动：
当前电池电压范围：
最近一次修改的 DecodeConfig 参数：
最近一次修改的红方 Pose：
已知机械问题：
下一步最优先事项：
Git commit / tag：
```
