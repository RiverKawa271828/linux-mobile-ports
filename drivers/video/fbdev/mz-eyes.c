// SPDX-License-Identifier: GPL-2.0
/*
 * mz-eyes —— 早期 panic 的"涂色心跳"（Meizu 20 专用，2026-09-25）
 *
 * 为什么需要它：本机主线实验的失败形态是"内核早期 panic"，而所有现有观测通道都要
 * 等驱动/userspace（pstore 要 ramoops probe、黑匣子要 userspace、USB 要 dwc3、
 * EDL 读不到 DDR）⇒ 死得比所有眼睛都早，于是只能盲打。这个驱动不依赖任何外设：
 * 直接把内容画进 ABL 留给内核的 splash 缓冲（工厂 DTB 的 splash_region，
 * 0xb8000000 + 43MiB，面板在 ABL 交接后仍在扫描它）。
 *
 * 屏幕语义（用一张照片即可判读）：
 *   · 背景蓝 = kernel 已到 early_initcall（"涂色通道可用"的证明）
 *   · 顶部 4 行 = initcall 阶段号（二进制，bit0 在最上）
 *   · 中部 16 行 = 已跑过的 initcall 计数（二进制，bit0 在最上）——计数条
 *   · 背景红 = panic 已发生（panic notifier 刷红，计数/阶段条用白色重画保留现场）
 *   · 蓝底 + 计数条停住不动 = 挂死（非 panic），停在哪个计数也看得见
 * 纯色与行条对 framebuffer 的 bpp/stride/旋转错配免疫（全行填同一个 32bit 字，
 * 任何格式下都还是"一条实心横带"）——这是它能当"第一只眼睛"的关键。
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/panic_notifier.h>
#include <linux/types.h>

#define MZ_FB_PA	0x0b800000UL	/* splash_region（工厂 DTB） */
#define MZ_FB_LEN	0x02b00000UL	/* 43 MiB */
#define MZ_ROW_PX	1080		/* 面板宽（qcom,mdss-dsi-panel-width） */
#define MZ_ROW_BYTES	(MZ_ROW_PX * 4)
#define MZ_MAX_ROWS	(MZ_FB_LEN / MZ_ROW_BYTES)
#define MZ_BIT_ROWS	16		/* 计数位行数（2^16 计数） */
#define MZ_LVL_ROWS	4		/* 阶段位行数 */
#define MZ_TOP_ROW	8		/* 顶部留白行 */

#define MZ_BLUE		0xff0000ffu	/* writel 值 → B=ff ⇒ 蓝 */
#define MZ_RED		0xffff0000u	/* writel 值 → R=ff ⇒ 红 */
#define MZ_WHITE	0xffffffffu

static void __iomem *mz_fb;
static unsigned long mz_ticks;
static unsigned int mz_level;
static bool mz_on;

/* 填满第 row 行（整行同色 ⇒ 对几何错配免疫） */
static void mz_fill_row(unsigned int row, u32 color)
{
	void __iomem *p;
	unsigned int x;

	if (!mz_fb || row >= MZ_MAX_ROWS)
		return;
	p = mz_fb + (u64)row * MZ_ROW_BYTES;
	for (x = 0; x < MZ_ROW_PX; x++)
		writel(color, p + x * 4);
}

/* 画一个 N 位的"二进制条"：bit i 亮 ⇒ 第 (start+i) 行全白 */
static void mz_paint_bits(unsigned int start, unsigned long value, unsigned int bits,
			  u32 on, u32 off)
{
	unsigned int i;

	for (i = 0; i < bits; i++) {
		u32 c = (value >> i) & 1 ? on : off;
		mz_fill_row(start + i * 2, c);	/* 隔行画，便于人眼/照片分辨 */
	}
}

static void mz_paint_all(u32 color)
{
	unsigned long i;

	for (i = 0; i < MZ_MAX_ROWS; i++)
		mz_fill_row(i, color);
}

/* 只重画"变化的位行"：每个 tick 最多几位翻转 ⇒ 每 tick 只写几 KB，可承受 */
static void mz_update_counter(void)
{
	mz_paint_bits(MZ_TOP_ROW + MZ_LVL_ROWS * 2, mz_ticks, MZ_BIT_ROWS,
		      MZ_WHITE, mz_fb ? MZ_BLUE : MZ_BLUE);
}

static int mz_panic_notify(struct notifier_block *nb, unsigned long ev, void *p)
{
	mz_on = false;
	mz_paint_all(MZ_RED);
	mz_paint_bits(MZ_TOP_ROW, mz_ticks, MZ_BIT_ROWS, MZ_WHITE, MZ_RED);
	mz_paint_bits(MZ_TOP_ROW + MZ_BIT_ROWS * 2, mz_level, MZ_LVL_ROWS,
		      MZ_WHITE, MZ_RED);
	return NOTIFY_DONE;
}

static struct notifier_block mz_panic_nb = { .notifier_call = mz_panic_notify };

/* 供 init/main.c 调用：每个 initcall 一次（定义在 CONFIG_MZ_EYES 下） */
void mz_eyes_tick(void)
{
	if (!mz_on)
		return;
	mz_ticks++;
	mz_update_counter();
}

void mz_eyes_level(unsigned int level)
{
	if (!mz_on || level == mz_level)
		return;
	mz_level = level;
	mz_paint_bits(MZ_TOP_ROW, mz_level, MZ_LVL_ROWS, MZ_WHITE, MZ_BLUE);
	mz_update_counter();
}

static int __init mz_eyes_init(void)
{
	mz_fb = ioremap(MZ_FB_PA, MZ_FB_LEN);
	if (!mz_fb)
		return 0;
	mz_paint_all(MZ_BLUE);			/* 蓝屏 = 活到 early_initcall 且涂色可用 */
	mz_paint_bits(MZ_TOP_ROW, 0, MZ_LVL_ROWS, MZ_WHITE, MZ_BLUE);
	mz_paint_bits(MZ_TOP_ROW + MZ_LVL_ROWS * 2, 0, MZ_BIT_ROWS, MZ_WHITE, MZ_BLUE);
	pr_info("mz-eyes: framebuffer @%#lx %luKiB mapped, 蓝屏就位\n",
		MZ_FB_PA, MZ_FB_LEN >> 10);
	atomic_notifier_chain_register(&panic_notifier_list, &mz_panic_nb);
	mz_on = true;
	return 0;
}
early_initcall(mz_eyes_init);
