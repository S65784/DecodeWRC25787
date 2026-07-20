# AutoCero / AutoSiete 外来路径移植核对

## 来源与结论

- `BLUE23close18.java`、`RED23close18.java`：坐标本身按144英寸场地镜像，但源码实际包含
  两次门区往返，并不是“不开门”。
- `BLUE7far.java`、`RED7far.java`：红蓝起点、发射点、A/B取球点和停车点满足
  `redX + blueX = 144`，六次A/B交替循环逻辑一致。

外来车使用Turret、独立Flywheel/Pitch/IntakeShooter子系统，其RPM、Pitch和Turret角度不能
直接用于当前机器人。新版只移植XY路线及动作顺序，改用当前机器人的：

- `ShooterSubsystem`速度互锁和`ShooterR`反馈；
- `IntakeSubsystem`；
- `DecodeConfig`现有自动发射参数/打表；
- 固定后向Shooter的底盘发射heading；
- `AutoPoseHandoff`自动到手动Pose交接。

`DecodeConfig.java`没有因本次移植而修改。

## AutoCero：近端零次开门

AutoCero删除了外来程序中的两次`kaimenzuo/fashekaimenzuo`门区循环，路径编号为：

| 编号 | 动作 |
|---|---|
| 01 | 起点 → 预装后向发射 |
| 02 | 吸取第二排 |
| 03 | 第二排 → 后向发射 |
| 04 | 前往第一排准备点 |
| 05 | 吸取第一排 |
| 06 | 第一排 → 后向发射 |
| 07 | 前往第三排准备点 |
| 08 | 吸取第三排 |
| 09 | 第三排 → 最终后向发射 |

同时修复了外来程序三处路径不连续：

- 初始化`Y=134.6`，首段原本声明从`Y=135.0`开始；
- 第二排吸取终点与返程声明起点相差2英寸；
- 第三排吸取终点与最终返程声明起点约差1英寸。

在16.65 × 13.03英寸机器人外框、四周1英寸余量下，红蓝AutoCero的采样检查均未检测到
场地边界或Goal多边形相交。

## AutoSiete：远端六次A/B吸球

AutoSiete保留外来程序的六次循环：A1、B1、A2、B2、A3、B3。`.pp`把重复路线展开为
01–14，便于逐段查看。

当前不能判定为可直接上场：

- 起点`Y=8`在当前机器人外框下非常贴近底边；
- A点为红`(134,10)`、蓝`(10,10)`，车体旋转过程中会触发边界包络警告；
- A循环重复三次，所以02/03、06/07、10/11均出现同类警告；
- 六次来回、七次发射和各次等待很可能接近或超过30秒，必须实测计时。

因此AutoSiete应先在Visualizer中检查，再架空机构测试，最后低功率逐段实跑。确认A点实际
球位后，应将A点和必要的起点向场内移动。

## 发射heading的依据

当前Shooter固定朝车尾，发射终点heading按当前`Alliance.goal()`占位坐标计算：

- 红Goal：`(134,138)`
- 蓝Goal：`(10,138)`

这些Goal仍是待实测数据。Goal坐标变化后，必须重新计算Java和`.pp`中的发射heading。
