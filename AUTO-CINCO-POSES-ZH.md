# AutoCinco / AutoDos 点位修改说明

## 现在的代码结构

- `AutoCincoBase.java`、`AutoDosBase.java`：公共执行逻辑。负责建路径、状态机、吸球、飞轮、
  发射互锁、超时和自动结束 Pose 交接。通常不在这里调坐标。
- `AutoCincoRoute.java`、`AutoDosRoute.java`：只保存一组路线数据，不进行红蓝镜像。
- `Red/RedAutoCinco.java`、`Blue/BlueAutoCinco.java`：红蓝 AutoCinco 的全部独立点位。
- `Red/RedAutoDos.java`、`Blue/BlueAutoDos.java`：红蓝 AutoDos 的全部独立点位。

以后需要微调哪一方，就只改该联盟的入口文件。所有坐标单位为 inch，heading 单位为 degree。
`Pxx` 是该编号路径的终点，`Cxx` 是该路径的 Bezier 控制点。

## AutoCinco 编号
##

| 编号 | 路径/动作 | 需要修改的变量 |
|---|---|---|
| 00 | 自动起点 | `p00Start` |
| 01 | 预装球 → 后向发射点 | `p01PreloadScore` |
| 02 | 发射点 → 第一排准备点 | `c02PickupOneReady`, `p02PickupOneReady` |
| 03 | 吸取第一排 | `p03PickupOne` |
| 04 | 推第一处 Gate | `c04GateOne`, `p04GateOne` |
| 05 | Gate → 第一次后向发射 | `c05ScoreOne`, `p05ScoreOne` |
| 06 | 发射点 → 第二排准备点 | `c06PickupTwoReady`, `p06PickupTwoReady` |
| 07 | 吸取第二排 | `p07PickupTwo` |
| 08 | 推第二处 Gate | `c08GateTwo`, `p08GateTwo` |
| 09 | Gate → 第二次后向发射 | `c09ScoreTwo`, `p09ScoreTwo` |
| 10 | 发射点 → 第三排准备点 | `c10PickupThreeReady`, `p10PickupThreeReady` |
| 11 | 吸取第三排 | `p11PickupThree` |
| 12 | 第三排 → 第三次后向发射 | `c12ScoreThree`, `p12ScoreThree` |
| 13 | 发射点 → 停车 | `p13Park` |

## AutoDos 编号

AutoDos 的 `P00–P08` 和 `C02–C07` 也按相同规则命名。第二轮 `05/06/07` 已经拥有独立
点位；调整第二轮不会再连带改变第一轮。

## 140 → 144 的处理

AutoCinco 使用与 AutoDos 相同的迁移原则：

- 红方原始 X 是参考值，保持不变。
- 旧蓝方使用 `blueX = 140 - redX`；现在改为 `blueX = 144 - redX`，所以蓝方 X 增加4 inch。
- 相对于误迁移的上一版文件，这等价于红方全部 X 减4、蓝方全部 X 加4。
- 旧程序已有的蓝方独立微调继续保留，例如第二排 Y、第二处 Gate 和最终停车点。
- Java 和 `.pp` 中不再保留运行时 `140 - x` 或自动镜像公式；所有数值都已展开，红蓝互不影响。

例如红方 X 为96时，旧蓝方是`140 - 96 = 44`，正确蓝方应为`144 - 96 = 48`。

## Driver Station 与 Visualizer

自动运行时：

- `Active step` 显示当前编号；
- `Target pose` 是该段的目标；
- `Pose` 是 Pinpoint 当前实测；
- `Notice` 会显示路径或发射超时。

Visualizer 文件：

- `visualizer/RED-AutoCinco-FrontIntake-RearShooter.pp`
- `visualizer/BLUE-AutoCinco-FrontIntake-RearShooter.pp`
- `visualizer/RED-AutoDos-Far-144-FrontIntake-RearShooter.pp`
- `visualizer/BLUE-AutoDos-Far-144-FrontIntake-RearShooter.pp`

`.pp` 使用 16.65 × 13.03 inch 机器人外框和 1 inch 安全余量。校验器仍会对旧路线的贴边和
球门区域给出警告，因此它们适合逐段核对，但不能代替低速实车测试。
