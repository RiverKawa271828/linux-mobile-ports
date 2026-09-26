// SPDX-License-Identifier: GPL-2.0-only
/*
 * mz_dump.c — Meizu 20 (m2381) bring-up 专用「内核侧黑匣子」
 *
 * 为什么需要它（2026-09-24 M-L3 两发实测的结论）：
 *   ABL 已能收下 v2 载荷并 Start Kernel（ABL 日志实证），但**内核侧一条观测通道都没有**：
 *     · USB gadget 起不来（宿主全程无 ttyACM / 无 NCM）
 *     · 面板是 cmd 模式且 ABL 停了管线（屏幕只有最后一帧 logo，内核画什么都上不去）
 *     · ttyMSM0 串口无测试点；IMEM 写进去读不回（dump 白名单无该区、日志区加密）
 *     · 连用户态都没起来（logfs 黑匣子一次没落盘）⇒ 卡在哪一步无从得知
 *   本驱动补的正是这条通道：把**内核自己的 log buffer**（cmdline 已带 initcall_debug，
 *   因此含 "calling/returned <fn>" 配对）周期性写进 UFS 上 logfs 分区 FAT 卷之外的
 *   闲置尾巴；宿主用 EDL 读回一步到位。另附 ramoops(pstore) 原始 2MB dump ⇒
 *   即使上一轮是 panic，这一轮也能把它捞出来（panic 时还走 kmsg_dump 回调补一刀）。
 *
 * 落点依据（实测）：logfs 分区前 5.00MB 是 XBL/ABL 的 FAT12 日志卷
 * （BPB: 4096B/sector, 1280 sectors），0x500000-0x800000 三兆全零、无任何结构。
 * 本驱动只碰这 3MB，不触碰上游卷结构。（不选 `reserved`：厂商私有类型，实测写入触发 900E。）
 *
 * 宿主侧读法：
 *   edl.py r logfs logfs.bin
 *     logfs.bin[0x500000 ..]            记录（MZD1 头 128B + 正文）
 *     logfs.bin[0x600000 .. 0x800000]   ramoops 原始 2MB（每轮只 dump 一次）
 *
 * 这是仪器，不是上游代码；bring-up 完成后应移除。
 */
#define pr_fmt(fmt) "mz_dump: " fmt

#include <linux/blkdev.h>
#include <linux/blk_types.h>
#include <linux/device/class.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kmsg_dump.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/string.h>

/*
 * logfs 分区指纹（来自 GPT 实测：offset 0xa8587000, size 0x800000，512B 扇区）
 * 用「起始扇区 + 扇区数」双条件匹配 ⇒ 不依赖 sdX 命名、不依赖分区号。
 */
#define MZ_LOGFS_START_SECT	5516348ULL
#define MZ_LOGFS_NR_SECTS	16384ULL
#define MZ_REC_OFF		0x500000ULL	/* 5MB FAT 卷之后；占用 0x500000-0x540000 */
#define MZ_REC_HDR		128		/* 头固定 128B，正文紧随 */
#define MZ_REC_MAX		(256 * 1024 - 256)	/* 正文上限（含 NUL） */
#define MZ_RAMOOPS_OFF		0x540000ULL	/* 0x540000-0x640000 */
#define MZ_RAMOOPS_ADDR		0xa5100000ULL	/* 与工厂 DTB ramoops 节点一致 */
#define MZ_RAMOOPS_SIZE		0x100000ULL	/* console 区 0x7d800，1MB 足够 */
#define MZ_MAGIC		"MZD1"

static struct kmsg_dumper mz_dumper;
static struct block_device *mz_bdev;
static char *mz_buf;
static bool mz_ramoops_done;
static DEFINE_MUTEX(mz_lock);

struct mz_scan {
	struct block_device *found;
};

static int mz_scan_one(struct device *dev, void *data)
{
	struct mz_scan *s = (struct mz_scan *)data;
	struct block_device *bdev = dev_to_bdev(dev);

	if (!bdev || !bdev_is_partition(bdev))
		return 0;
	if (get_start_sect(bdev) == MZ_LOGFS_START_SECT &&
	    bdev_nr_sectors(bdev) == MZ_LOGFS_NR_SECTS) {
		s->found = bdev;
		return 1;
	}
	return 0;
}

static struct block_device *mz_find_logfs(void)
{
	struct mz_scan s = { };

	class_for_each_device(&block_class, NULL, &s, mz_scan_one);
	return s.found;
}

static int mz_write_at(struct block_device *bdev, loff_t off,
		       const void *data, size_t len)
{
	struct file *f;
	ssize_t ret;
	loff_t pos = off;

	f = bdev_file_open_by_dev(bdev->bd_dev, BLK_OPEN_READ | BLK_OPEN_WRITE,
				  NULL, NULL);
	if (IS_ERR(f))
		return PTR_ERR(f);
	ret = kernel_write(f, data, len, &pos);
	fput(f);
	if (ret < 0)
		return ret;
	return ret == (ssize_t)len ? 0 : -EIO;
}

static void mz_emit(const char *why, bool with_ramoops)
{
	struct kmsg_dump_iter iter = { };
	char hdr[MZ_REC_HDR];
	size_t len = 0;
	int n;

	if (!mz_bdev || !mz_buf)
		return;
	if (!mutex_trylock(&mz_lock))
		return;

	kmsg_dump_rewind(&iter);
	if (!kmsg_dump_get_buffer(&iter, false, mz_buf, MZ_REC_MAX, &len))
		len = 0;
	mz_buf[len] = '\0';

	n = scnprintf(hdr, sizeof(hdr), MZ_MAGIC " why=%s uptime_ms=%llu len=%zu\n",
		      why, ktime_to_ms(ktime_get_boottime()), len);
	mz_write_at(mz_bdev, MZ_REC_OFF, hdr, sizeof(hdr));
	mz_write_at(mz_bdev, MZ_REC_OFF + MZ_REC_HDR, mz_buf, len + 1);

	if (with_ramoops && !mz_ramoops_done) {
		void *ra = memremap(MZ_RAMOOPS_ADDR, MZ_RAMOOPS_SIZE, MEMREMAP_WB);

		if (ra) {
			if (mz_write_at(mz_bdev, MZ_RAMOOPS_OFF, ra,
					MZ_RAMOOPS_SIZE) == 0)
				mz_ramoops_done = true;
			memunmap(ra);
		}
	}
	mutex_unlock(&mz_lock);
}

static void mz_dump_cb(struct kmsg_dumper *dumper, struct kmsg_dump_detail *detail)
{
	mz_emit(detail && detail->reason == KMSG_DUMP_PANIC ? "panic" : "oops",
		false);
}

static int mz_thread(void *arg)
{
	unsigned long start = jiffies;
	bool complained = false;

	while (!kthread_should_stop()) {
		if (!mz_bdev)
			mz_bdev = mz_find_logfs();
		if (mz_bdev) {
			mz_emit("tick", true);
		} else if (!complained && time_after(jiffies, start + 8 * HZ)) {
			/*
			 * 8 秒还拿不到 logfs 块设备 = UFS/块层没起来。
			 * 配合 cmdline 的 panic=10：本函数会 panic ⇒ **设备自己重启**
			 * ⇒ 外部可见的"~10s 周期复位循环"就是「内核确实执行到了 postcore、
			 *   但 UFS 没起来」的不可伪造证据（反之：无记录 + 不复位 = 更早就死/没执行）。
			 */
			complained = true;
			panic("MZ_DUMP: logfs block device not found 8s after postcore (UFS/block layer down)");
		}
		msleep(1000);
	}
	return 0;
}

static int __init mz_dump_init(void)
{
	mz_buf = kmalloc(MZ_REC_MAX + 1, GFP_KERNEL);
	if (!mz_buf)
		return -ENOMEM;

	mz_dumper.dump = mz_dump_cb;
	mz_dumper.max_reason = KMSG_DUMP_OOPS;
	kmsg_dump_register(&mz_dumper);
	kthread_run(mz_thread, NULL, "mz_dump");

	pr_info("armed: logfs start_sect=%llu nr_sects=%llu rec@0x%llx ramoops@0x%llx\n",
		MZ_LOGFS_START_SECT, MZ_LOGFS_NR_SECTS,
		(u64)MZ_REC_OFF, (u64)MZ_RAMOOPS_OFF);
	return 0;
}
postcore_initcall(mz_dump_init);
