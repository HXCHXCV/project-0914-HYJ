# OBS 输出功能交接说明

## 1. 功能说明

本次在现有 GPS L1 C/A 中频信号仿真项目中增加了 RINEX 3.02 OBS 输出功能，目前接入位置欺骗分支：

```matlab
settings.SpoofingTypes == 1
```

支持全通道和部分通道两种位置欺骗模式。

全通道模式输出：

```text
Direct.obs
Virtual.obs
```

其中：

- `Direct.obs`：真实接收机轨迹对应的直达观测；
- `Virtual.obs`：虚拟轨迹对应的完整转发欺骗观测。

部分通道模式输出：

```text
Direct.obs
PartialSpoof.obs
```

其中：

- `Direct.obs`：真实接收机轨迹对应的直达观测；
- `PartialSpoof.obs`：被骗通道采用转发欺骗观测，其余通道采用直达观测。

当前输出 GPS L1 C/A 的以下观测量：

```text
C1C  L1C  D1C  S1C
```

## 2. 使用方法

项目入口仍为：

```matlab
init.m
```

OBS 及全/部分通道相关参数在 `initSettings.m` 中设置：

```matlab
settings.obsEnable            = 1;
settings.obsInterval          = 1.0;       % 0.001 or 1.0 [s]
settings.directObsFile        = 'Direct.obs';
settings.virtualObsFile       = 'Virtual.obs';
settings.partialSpoofObsFile  = 'PartialSpoof.obs';

settings.SpoofingChannelMode = 1;  % 1-full channel, 2-partial channel
settings.PartialSpoofingMask = [1 1 1 1 0 0 0 0 0]; % 1-spoofed, 0-direct
```

参数说明：

- `obsEnable`：OBS 输出开关；
- `obsInterval`：OBS 输出间隔，支持 `1.0 s` 和 `0.001 s`；
- `SpoofingChannelMode`：`1` 为全通道，`2` 为部分通道；
- `PartialSpoofingMask`：部分通道模式下的通道选择向量，`1` 表示对应可见通道加入欺骗信号，`0` 表示保持 Direct。

例如：

```matlab
settings.PartialSpoofingMask = [1 1 1 1 0 0 0 0 0];
```

表示当前 `satList` 中前四个通道加入欺骗信号，其余通道保持 Direct；若当前可见通道超过 9 个，新增通道也默认保持 Direct。

设置完成后正常运行 `init.m` 即可同时生成对应模式的 IF 和 OBS。

## 3. OBS 与原仿真链路的关系

OBS 直接复用原 IF 生成过程中已经计算的传播时间和卫星钟差，没有改变原有欺骗模型和传播链路。

核心对应关系为：

```text
TxTime  -> Direct
TxTime2 -> 转发欺骗观测
```

Direct 使用 `GetTravelTime`。

转发欺骗使用 `GetTravelTime_deltaT2`，其传播链包含：

```text
卫星 -> 虚拟轨迹
欺骗源 -> 真实接收机
附加延迟
```

全通道的 `Virtual.obs` 和部分通道中被骗卫星使用的是同一套完整转发传播计算。

部分通道 IF 保留原模型，并由 `PartialSpoofingMask` 控制通道选择：

```text
欺骗开启前：
所有通道 = Direct IF

欺骗开启后：
Mask = 1 的通道 = Direct IF + Virtual IF
Mask = 0 的通道 = Direct IF
```

对应的 `PartialSpoof.obs` 与 IF 使用相同的通道选择配置：

```text
欺骗开启前：
所有卫星使用 Direct 观测

欺骗开启后：
Mask = 1 的卫星 = 转发欺骗观测
Mask = 0 的卫星 = Direct 观测
```

## 4. 观测量计算

`calcObsFromTxTime_GPSL1CA.m` 根据 IF 链路中的传播时间和卫星钟差计算观测量：

```text
C1C = c × (travelTime - satClkErr)
L1C = C1C / λL1 + N
D1C = -ΔC1C / (λL1 × Δt)
S1C = 对应信号的 C/N0
```

Direct 和转发欺骗观测使用相同的卫星列表和 RINEX 输出格式，载波相位使用按 PRN 固定的整周模糊度。

Virtual 以及部分通道中的被骗卫星，其 `S1C` 会在 Direct C/N0 的基础上增加 `powerIncreaseFactor`，与 IF 中的欺骗功率设置对应。

## 5. 代码修改

主要修改文件：

- `initSettings.m`：增加 OBS 配置、全/部分通道模式配置及部分通道 `PartialSpoofingMask`；
- `signalGenerating_v1_SingPreset.m`：接入 Direct / Virtual 观测量采集，增加全/部分通道模式切换，根据 `PartialSpoofingMask` 完成部分通道选择，并输出对应 OBS；
- `GetTravelTime.m`：增加直达信号传播时间输出；
- `GetTravelTime_deltaT2.m`：增加转发欺骗链路传播时间输出。

新增文件：

- `calcObsFromTxTime_GPSL1CA.m`
- `writeRinex302Obs_GPS_L1CA.m`
- `GPST2Cal.m`
- `selectEphemerisByPrn_GPS.m`

其中，`selectEphemerisByPrn_GPS.m` 用于根据 PRN 和仿真起始时刻选择对应的广播星历记录。

具体修改请结合 Git diff 查看。

## 6. 已完成测试

目前已完成以下基础测试：

- 全通道和部分通道 OBS 均可正常生成；
- `1 s` 和 `1 ms` OBS 均可正常输出；
- `1 s` OBS 已使用 GINav 进行 GPS 单频 SPP 基础检查：Direct / Virtual 的平均速度分别为 `2.9949 m/s` 和 `3.9977 m/s`，与设定的 `3 m/s` 和 `4 m/s` 基本一致；两者相对位置关系与设定轨迹相比，三维误差平均为 `0.0364 m`；
- 部分通道 OBS 已核对欺骗前后通道切换关系，IF 与 `PartialSpoof.obs` 使用相同的通道选择配置，被选中的通道使用欺骗分支，其余通道保持 Direct；
- 开启 OBS 输出后未发现对原 IF 输出产生影响。

## 7. 当前范围

- OBS 目前只接入 `SpoofingTypes == 1` 的位置欺骗分支；
- 当前仅输出 GPS L1 C/A 的 `C1C / L1C / D1C / S1C`；
- 尚未使用 FGI 将 IF 信号经过软件接收机处理后得到的观测量与直接导出的 OBS 做一致性比较。

后续如需增加其他欺骗类型、观测量或接收机端验证，可在当前版本基础上继续扩展。
