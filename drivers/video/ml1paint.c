// SPDX-License-Identifier: GPL-2.0
/*
 * ml1paint.c — ML1 屏幕控制台(无 USB/串口条件下的主线内核诊断)
 * 每 1s 扫描 ramoops console 区(0xa5100000/2MB)中最后一次 "ML1CALL "
 * 标记,将其后的 16 位十六进制函数地址以 64x64 大字号渲染到
 * ABL 保留帧缓冲(0xb8000000)顶部。内核在任何 initcall 挂死时,
 * 屏幕冻结显示挂死点的地址;宿主用 System.map 解析符号。
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>

#define ML1_FB_PHYS	0xb8000000UL
#define ML1_FB_SIZE	0x1000000	/* 16MB 覆盖整屏 */
#define ML1_RO_PHYS	0xa5100000UL
#define ML1_RO_SIZE	0x200000	/* 2MB ramoops */
#define ML1_WIDTH	1080
#define ML1_TAG		"ML1CALL "

static void __iomem *fb;
static void __iomem *ro;
static struct timer_list ml1_timer;

/* 经典 8x8 点阵 0-9 A-F(末位 bit7 在左) */
static const unsigned char ml1_hexfont[16][8] = {
	{0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, /* 0 */
	{0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, /* 1 */
	{0x3C,0x66,0x06,0x0C,0x30,0x60,0x7E,0x00}, /* 2 */
	{0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, /* 3 */
	{0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00}, /* 4 */
	{0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, /* 5 */
	{0x3C,0x66,0x60,0x7C,0x66,0x66,0x3C,0x00}, /* 6 */
	{0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00}, /* 7 */
	{0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, /* 8 */
	{0x3C,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00}, /* 9 */
	{0x18,0x3C,0x66,0x66,0x7E,0x66,0x66,0x00}, /* A */
	{0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00}, /* B */
	{0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, /* C */
	{0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, /* D */
	{0x7E,0x60,0x60,0x7C,0x60,0x60,0x7E,0x00}, /* E */
	{0x7E,0x60,0x60,0x7C,0x60,0x60,0x60,0x00}, /* F */
};

static int ml1_hexval(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

static void ml1_px(int x, int y, u32 c)
{
	writel(c, fb + (y * ML1_WIDTH + x) * 4);
}

static void ml1_glyph(int x0, int y0, char c)
{
	int d = ml1_hexval(c);
	int r, b, sx, sy;

	for (r = 0; r < 8; r++) {
		unsigned char row = (d >= 0) ? ml1_hexfont[d][r] : 0;
		for (b = 0; b < 8; b++) {
			u32 col = ((row >> (7 - b)) & 1) ? 0xFFFFFFFF : 0xFF000000;
			for (sy = 0; sy < 8; sy++)
				for (sx = 0; sx < 8; sx++)
					ml1_px(x0 + b*8 + sx, y0 + r*8 + sy, col);
		}
	}
}

static void ml1_render(void)
{
	static unsigned char buf[ML1_RO_SIZE];
	int i, j, last = -1;
	int digits[16], nd = 0;

	memcpy_fromio(buf, ro, ML1_RO_SIZE);
	/* 找最后一次 ML1CALL */
	for (i = 0; i <= ML1_RO_SIZE - 8; i++) {
		if (buf[i]=='M' && buf[i+1]=='L' && buf[i+2]=='1' &&
		    buf[i+3]=='C' && buf[i+4]=='A' && buf[i+5]=='L' &&
		    buf[i+6]=='L' && buf[i+7]==' ')
			last = i + 8;
	}
	/* 提取后续十六进制串(至多 16 字符) */
	for (i = last; i >= 0 && i < ML1_RO_SIZE && nd < 16; i++) {
		int v = ml1_hexval(buf[i]);
		if (v >= 0) { digits[nd++] = v; }
		else if (nd) break;
	}
	if (last < 0 || nd == 0) {
		/* 没有任何 ML1CALL: 画一个蓝角标示"活着但没到 initcall 阶段" */
		for (i = 0; i < 4096; i++) ml1_px(i % 64, i / 64, 0xFFFF0000);
		return;
	}
	/* 渲染: 顶部居中一排大号十六进制 */
	{
		int x0 = (ML1_WIDTH - nd * 64) / 2;
		for (j = 0; j < nd; j++)
			ml1_glyph(x0 + j*64, 0, "0123456789ABCDEF"[digits[j]]);
	}
}

static void ml1_timer_fn(struct timer_list *t)
{
	ml1_render();
	mod_timer(&ml1_timer, jiffies + msecs_to_jiffies(1000));
}

static int __init ml1paint_init(void)
{
	fb = ioremap(ML1_FB_PHYS, ML1_FB_SIZE);
	ro = ioremap(ML1_RO_PHYS, ML1_RO_SIZE);
	if (!fb || !ro) {
		pr_err("ml1paint: ioremap failed fb=%p ro=%p\n", fb, ro);
		return -ENOMEM;
	}
	timer_setup(&ml1_timer, ml1_timer_fn, 0);
	mod_timer(&ml1_timer, jiffies + msecs_to_jiffies(500));
	pr_info("ml1paint: active (fb=%p ro=%p)\n", fb, ro);
	return 0;
}
early_initcall(ml1paint_init);
MODULE_LICENSE("GPL");
