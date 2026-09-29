# linux-mobile-ports

主线（torvalds/linux）基座上的 Qualcomm 移动设备移植内核仓。设备身份由分支承载，
本仓库不绑定单一机型/SoC。

## 支持的设备与分支

| 分支 | 设备 | SoC |
|---|---|---|
| `meizu20/7.3` | Meizu 20（m2381） | SM8550 / kalama |

分支命名：`<设备代号>/<内核版本>`。新设备 = 新分支，不共享设备代码。

## 基线策略

- 基座 = torvalds/linux master（`meizu20/7.3` 基于 `587858367581`，2026-09-14，7.3.0-rc3 窗口）。
- 在树内容 = 上游主线 + 少量按「携带清单」精选的提交：
  1. 通用驱动修复（qrtr/IPA/a6xx/battmgr/tfa98xx/ath12k/MAC 派生等），上游暂无；
  2. 上游邮件列表在途补丁的 vendored 快照（PCI qcom #iommu-cells、Akhil GMU 组件化系列）——
     上游合并后摘除换上游版；
  3. 设备使能：m2381 DTS、自写 panel-boe-meizu20 驱动、serial-MAC 组。
- 完整来源与取舍记录见移植工作区 `docs/meizu20/silime-exit-carry-list.md`。

## 上游化政策

**搬运车，不是终点**：树内每一条非上游提交都是待上游化的候选——熟一个发一个，
上游合并后摘除；终局目标是本仓库趋近于零非上游提交（只剩 DTS 与设备专属驱动）。
上游动作节奏由移植工作区统一调度，不在此仓内各自为战。

## 构建

通过 postmarketOS 管道构建（pmaports `device/downstream/linux-meizu-meizu20`，
phase-1 以本仓 `git archive` tarball 为源，phase-2 切 git URL）。内核命令行/
交付形态由 Mu UEFI 链载链约定，见工作区文档，不在本仓展开。

## 不承诺

- 不追 rc / 不无纪律 rebase（升版走专项会话 + 全回归清单）；
- 不背任何第三方 fork 的历史包袱（提交来源逐条可溯，cherry-pick 带 `-x`）；
- 无设备的空想清理/重构一律不进。
