/*
 * Remote processor machine-specific module for OMAP4
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/sched.h>
#include <linux/rproc_drm.h>

#include <plat/iommu.h>
#include <plat/omap_device.h>
#include <plat/remoteproc.h>
#include <plat/mailbox.h>
#include <plat/common.h>
#include <plat/omap-pm.h>
#include <plat/dmtimer.h>
#include "../../arch/arm/mach-omap2/dvfs.h"
#include "../../arch/arm/mach-omap2/clockdomain.h"

#define PM_SUSPEND_MBOX		0xffffff07
#define PM_SUSPEND_TIMEOUT	300

struct omap_rproc_priv {
	struct iommu *iommu;
	int (*iommu_cb)(struct rproc *, u64, u32);
	int (*wdt_cb)(struct rproc *);
	u64 bootaddr;
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	struct omap_mbox *mbox;
	void __iomem *idle;
	u32 idle_mask;
	void __iomem *suspend;
	u32 suspend_mask;
#endif
};

#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
static bool _may_suspend(struct omap_rproc_priv *rpp)
{
	return readl(rpp->idle) & rpp->idle_mask;
}

static int _suspend(struct omap_rproc_priv *rpp)
{
	unsigned long timeout = msecs_to_jiffies(PM_SUSPEND_TIMEOUT) + jiffies;

	omap_mbox_msg_send(rpp->mbox, PM_SUSPEND_MBOX);

	while (time_after(timeout, jiffies)) {
		if ((readl(rpp->suspend) & rpp->suspend_mask) &&
				(readl(rpp->idle) & rpp->idle_mask))
			return 0;
		schedule();
	}

	return -EAGAIN;
}

static int omap_suspend(struct rproc *rproc, bool force)
{
	struct omap_rproc_priv *rpp = rproc->priv;

	if (rpp->idle && (force || _may_suspend(rpp)))
		return _suspend(rpp);

	return -EBUSY;
}
#endif

static void omap_rproc_dump_registers(struct rproc *rproc)
{
	struct device *dev = rproc->dev;
	struct omap_rproc_pdata *pdata = dev->platform_data;

	pdata->ops->dump_registers(rproc);
}

static int
omap_rproc_map(struct device *dev, struct iommu *obj, u32 da, u32 pa, u32 size)
{
	struct iotlb_entry e;
	u32 all_bits;
	u32 pg_size[] = {SZ_16M, SZ_1M, SZ_64K, SZ_4K};
	int size_flag[] = {MMU_CAM_PGSZ_16M, MMU_CAM_PGSZ_1M,
		MMU_CAM_PGSZ_64K, MMU_CAM_PGSZ_4K};
	int i, ret;

	while (size) {
		/*
		 * To find the max. page size with which both PA & VA are
		 * aligned
		 */
		all_bits = pa | da;
		for (i = 0; i < 4; i++) {
			if ((size >= pg_size[i]) &&
				((all_bits & (pg_size[i] - 1)) == 0)) {
				break;
			}
		}

		memset(&e, 0, sizeof(e));

		e.da = da;
		e.pa = pa;
		e.valid = 1;
		e.pgsz = size_flag[i];
		e.endian = MMU_RAM_ENDIAN_LITTLE;
		e.elsz = MMU_RAM_ELSZ_32;

		ret = iopgtable_store_entry(obj, &e);
		if (ret) {
			dev_err(dev, "iopgtable_store_entry fail: %d\n", ret);
			return ret;
		}

		size -= pg_size[i];
		da += pg_size[i];
		pa += pg_size[i];
	}

	return 0;
}


static int omap_rproc_iommu_isr(struct iommu *iommu, u32 da, u32 errs, void *p)
{
	struct rproc *rproc = p;
	struct omap_rproc_priv *rpp = rproc->priv;
	int ret = -EIO;

	if (rpp && rpp->iommu_cb)
		ret = rpp->iommu_cb(rproc, (u64)da, errs);

	return ret;
}

static inline void _load_boot_addr(struct rproc *rproc, u64 bootaddr)
{
	struct omap_rproc_pdata *pdata = rproc->dev->platform_data;

	if (pdata->boot_reg)
		omap_writel(bootaddr, pdata->boot_reg);
	return;
}

int omap_rproc_activate(struct omap_device *od)
{
	int i, ret = 0;
	struct rproc *rproc = platform_get_drvdata(&od->pdev);
	struct device *dev = rproc->dev;
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc_timers_info *timers = pdata->timers;
	struct omap_rproc_priv *rpp = rproc->priv;
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	struct iommu *iommu;

	if (!rpp->iommu) {
		iommu = iommu_get(pdata->iommu_name);
		if (IS_ERR(iommu)) {
			dev_err(dev, "iommu_get error: %ld\n",
				PTR_ERR(iommu));
			return PTR_ERR(iommu);
		}
		rpp->iommu = iommu;
	}

	if (!rpp->mbox)
		rpp->mbox = omap_mbox_get(pdata->sus_mbox_name, NULL);
#endif
	/**
	 * explicitly configure a boot address from which remoteproc
	 * starts executing code when taken out of reset.
	 */
	_load_boot_addr(rproc, rpp->bootaddr);

	/**
	 * Domain is in HW SUP thus in hw_auto but
	 * since remoteproc will be enabled clkdm
	 * needs to be in sw_sup (Do not let it idle).
	 */
	if (pdata->clkdm)
		clkdm_wakeup(pdata->clkdm);

	for (i = 0; i < pdata->timers_cnt; i++)
		omap_dm_timer_start(timers[i].odt);

	for (i = 0; i < od->hwmods_cnt; i++) {
		ret = omap_hwmod_enable(od->hwmods[i]);
		if (ret) {
			for (i = 0; i < pdata->timers_cnt; i++)
				omap_dm_timer_stop(timers[i].odt);
			break;
		}
	}

	/**
	 * Domain is in force_wkup but since remoteproc
	 * was enabled it is safe now to switch clkdm
	 * to hw_auto (let it idle).
	 */
	if (pdata->clkdm)
		clkdm_allow_idle(pdata->clkdm);

	return ret;
}

int omap_rproc_deactivate(struct omap_device *od)
{
	int i, ret = 0;
	struct rproc *rproc = platform_get_drvdata(&od->pdev);
	struct device *dev = rproc->dev;
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc_timers_info *timers = pdata->timers;
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	struct omap_rproc_priv *rpp = rproc->priv;
#endif
	if (pdata->clkdm)
		clkdm_wakeup(pdata->clkdm);

	for (i = 0; i < od->hwmods_cnt; i++) {
		ret = omap_hwmod_shutdown(od->hwmods[i]);
		if (ret)
			goto err;
	}

	for (i = 0; i < pdata->timers_cnt; i++)
		omap_dm_timer_stop(timers[i].odt);

#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	if (rpp->iommu) {
		iommu_put(rpp->iommu);
		rpp->iommu = NULL;
	}

	if (rpp->mbox) {
		omap_mbox_put(rpp->mbox, NULL);
		rpp->mbox = NULL;
	}
#endif
err:
	if (pdata->clkdm)
		clkdm_allow_idle(pdata->clkdm);

	return ret;
}

static int omap_rproc_iommu_init(struct rproc *rproc,
		 int (*callback)(struct rproc *rproc, u64 fa, u32 flags))
{
	struct device *dev = rproc->dev;
	struct omap_rproc_pdata *pdata = dev->platform_data;
	int ret, i;
	struct iommu *iommu;
	struct omap_rproc_priv *rpp;

	rpp = kzalloc(sizeof(*rpp), GFP_KERNEL);
	if (!rpp)
		return -ENOMEM;

	if (pdata->clkdm)
		clkdm_wakeup(pdata->clkdm);
	iommu_set_isr(pdata->iommu_name, omap_rproc_iommu_isr, rproc);
	iommu_set_secure(pdata->iommu_name, rproc->secure_mode,
						rproc->secure_ttb);
	iommu = iommu_get(pdata->iommu_name);
	if (IS_ERR(iommu)) {
		ret = PTR_ERR(iommu);
		dev_err(dev, "iommu_get error: %d\n", ret);
		goto err_mmu;
	}

	rpp->iommu = iommu;
	rpp->iommu_cb = callback;
	rproc->priv = rpp;

	if (!rproc->secure_mode) {
		for (i = 0; rproc->memory_maps[i].size; i++) {
			const struct rproc_mem_entry *me =
							&rproc->memory_maps[i];

			ret = omap_rproc_map(dev, iommu, me->da, me->pa,
								 me->size);
			if (ret)
				goto err_map;
		}
	}
	if (pdata->clkdm)
		clkdm_allow_idle(pdata->clkdm);

	return 0;

err_map:
	iommu_put(iommu);
err_mmu:
	iommu_set_secure(pdata->iommu_name, false, NULL);
	if (pdata->clkdm)
		clkdm_allow_idle(pdata->clkdm);
	kfree(rpp);
	return ret;
}

#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
static int _init_pm_flags(struct rproc *rproc)
{
	struct omap_rproc_pdata *pdata = rproc->dev->platform_data;
	struct omap_rproc_priv *rpp = rproc->priv;
	struct omap_mbox *mbox;

	if (!rpp->mbox) {
		mbox = omap_mbox_get(pdata->sus_mbox_name, NULL);
		if (IS_ERR(mbox))
			return PTR_ERR(mbox);
		rpp->mbox = mbox;
	}
	if (!pdata->idle_addr)
		goto err_idle;

	rpp->idle = ioremap(pdata->idle_addr, sizeof(u32));
	if (!rpp->idle)
		goto err_idle;

	if (!pdata->suspend_addr)
		goto err_suspend;

	rpp->suspend = ioremap(pdata->suspend_addr, sizeof(u32));
	if (!rpp->suspend)
		goto err_suspend;

	rpp->idle_mask = pdata->idle_mask;
	rpp->suspend_mask = pdata->suspend_mask;

	return 0;
err_suspend:
	iounmap(rpp->idle);
	rpp->idle = NULL;
err_idle:
	omap_mbox_put(rpp->mbox, NULL);
	rpp->mbox = NULL;
	return -EIO;
}

static void _destroy_pm_flags(struct rproc *rproc)
{
	struct omap_rproc_priv *rpp = rproc->priv;

	if (rpp->mbox) {
		omap_mbox_put(rpp->mbox, NULL);
		rpp->mbox = NULL;
	}
	if (rpp->idle) {
		iounmap(rpp->idle);
		rpp->idle = NULL;
	}
	if (rpp->suspend) {
		iounmap(rpp->suspend);
		rpp->suspend = NULL;
	}
}
#endif
#ifdef CONFIG_REMOTEPROC_WATCHDOG
static int omap_rproc_watchdog_init(struct rproc *rproc,
		 int (*callback)(struct rproc *rproc))
{
	struct omap_rproc_priv *rpp = rproc->priv;

	rpp->wdt_cb = callback;
	return 0;
}

static int omap_rproc_watchdog_exit(struct rproc *rproc)
{
	struct omap_rproc_priv *rpp = rproc->priv;

	rpp->wdt_cb = NULL;
	return 0;
}

static irqreturn_t omap_rproc_watchdog_isr(int irq, void *p)
{
	struct rproc *rproc = p;
	struct omap_rproc_pdata *pdata = rproc->dev->platform_data;
	struct omap_rproc_timers_info *timers = pdata->timers;
	struct omap_dm_timer *timer = NULL;
	struct omap_rproc_priv *rpp = rproc->priv;
	int i;

	for (i = 0; i < pdata->timers_cnt; i++) {
		if (irq == omap_dm_timer_get_irq(timers[i].odt)) {
			timer = timers[i].odt;
			break;
		}
	}

	if (!timer)
		return IRQ_NONE;

	omap_dm_timer_write_status(timer, OMAP_TIMER_INT_OVERFLOW);

	if (rpp->wdt_cb)
		rpp->wdt_cb(rproc);

	return IRQ_HANDLED;
}
#endif

static int omap_rproc_pm_init(struct rproc *rproc, u64 susp_addr)
{
	struct omap_rproc_pdata *pdata = rproc->dev->platform_data;
	phys_addr_t pa;
	int ret;

	ret = rproc_da_to_pa(rproc, susp_addr, &pa);
	if (!ret)
		pdata->suspend_addr = (u32)pa;

	return ret;
}

static inline int omap_rproc_start(struct rproc *rproc, u64 bootaddr)
{
	struct device *dev = rproc->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc_timers_info *timers = pdata->timers;
	struct omap_rproc_priv *rpp = rproc->priv;
	int i;
	int ret = 0;

	if (rproc->secure_mode) {
		rproc->secure_reset = true;
		ret = rproc_drm_invoke_service(rproc->secure_mode);
		if (ret) {
			dev_err(rproc->dev, "rproc_drm_invoke_service failed "
					"for secure_enable ret = 0x%x\n", ret);
			return -ENXIO;
		}
	}

#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	ret = _init_pm_flags(rproc);
	if (ret)
		return ret;
#endif

	for (i = 0; i < pdata->timers_cnt; i++) {
		timers[i].odt = omap_dm_timer_request_specific(timers[i].id);
		if (!timers[i].odt) {
			ret = -EBUSY;
			goto out;
		}
		omap_dm_timer_set_source(timers[i].odt, OMAP_TIMER_SRC_SYS_CLK);
#ifdef CONFIG_REMOTEPROC_WATCHDOG
		/* GPT 9 & 11 (ipu); GPT 6 (dsp) are used as watchdog timers */
		if ((!strcmp(rproc->name, "dsp") && timers[i].id == 6) ||
		    (!strcmp(rproc->name, "ipu") &&
				(timers[i].id == 9 || timers[i].id == 11))) {
			ret = request_irq(omap_dm_timer_get_irq(timers[i].odt),
					 omap_rproc_watchdog_isr, IRQF_DISABLED,
					"rproc-wdt", rproc);
			/* Clean counter, remoteproc proc will set the value */
			omap_dm_timer_set_load(timers[i].odt, 0, 0);
		}
#endif
	}

	rpp->bootaddr = bootaddr;
	ret = omap_device_enable(pdev);
out:
	if (ret) {
		while (i--) {
			omap_dm_timer_free(timers[i].odt);
			timers[i].odt = NULL;
		}
	}

	return ret;
}

static int omap_rproc_iommu_exit(struct rproc *rproc)
{
	struct omap_rproc_priv *rpp = rproc->priv;
	struct omap_rproc_pdata *pdata = rproc->dev->platform_data;

	if (pdata->clkdm)
		clkdm_wakeup(pdata->clkdm);

	if (rpp->iommu)
		iommu_put(rpp->iommu);
	kfree(rpp);
	if (pdata->clkdm)
		clkdm_allow_idle(pdata->clkdm);

	return 0;
}

static inline int omap_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc_timers_info *timers = pdata->timers;
	int ret, i;

#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	_destroy_pm_flags(rproc);
#endif
	if (rproc->secure_reset) {
		ret = rproc_drm_invoke_service(false);
		if (ret)
			dev_err(rproc->dev, "rproc_drm_invoke_service failed "
					"for secure disable ret = 0x%x\n", ret);
		rproc->secure_reset = false;
	}

	ret = omap_device_idle(pdev);
	if (ret)
		goto err;

	for (i = 0; i < pdata->timers_cnt; i++) {
#ifdef CONFIG_REMOTEPROC_WATCHDOG
		/* GPT 9 & 11 (ipu); GPT 6 (dsp) are used as watchdog timers */
		if ((!strcmp(rproc->name, "dsp") && timers[i].id == 6) ||
		    (!strcmp(rproc->name, "ipu") &&
				(timers[i].id == 9 || timers[i].id == 11)))
			free_irq(omap_dm_timer_get_irq(timers[i].odt), rproc);
#endif
		omap_dm_timer_free(timers[i].odt);
		timers[i].odt = NULL;
	}
err:
	return ret;
}

static int omap_rproc_set_lat(struct rproc *rproc, long val)
{
	int ret = 0;

	if (!strcmp(rproc->name, "ipu"))
		pm_qos_update_request(rproc->qos_request, val);
	else
		ret = omap_pm_set_max_dev_wakeup_lat(rproc->dev,
						rproc->dev, val);

	return ret;
}

static int omap_rproc_set_l3_bw(struct rproc *rproc, long val)
{
	return omap_pm_set_min_bus_tput(rproc->dev, OCP_INITIATOR_AGENT, val);
}

static int omap_rproc_scale(struct rproc *rproc, long val)
{
	return omap_device_scale(rproc->dev, rproc->dev, val);
}

static struct rproc_ops omap_rproc_ops = {
	.start = omap_rproc_start,
	.stop = omap_rproc_stop,
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	.suspend = omap_suspend,
#endif
	.iommu_init = omap_rproc_iommu_init,
	.iommu_exit = omap_rproc_iommu_exit,
	.set_lat = omap_rproc_set_lat,
	.set_bw = omap_rproc_set_l3_bw,
	.scale = omap_rproc_scale,
#ifdef CONFIG_REMOTEPROC_WATCHDOG
	.watchdog_init = omap_rproc_watchdog_init,
	.watchdog_exit = omap_rproc_watchdog_exit,
#endif
	.dump_registers = omap_rproc_dump_registers,
	.pm_init = omap_rproc_pm_init,
};

static int omap_rproc_probe(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;

	pdata->clkdm = clkdm_lookup(pdata->clkdm_name);

	return rproc_register(&pdev->dev, pdata->name, &omap_rproc_ops,
				pdata->firmware, pdata->memory_pool,
				THIS_MODULE, pdata->sus_timeout);
}

static int __devexit omap_rproc_remove(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;

	return rproc_unregister(pdata->name);
}

static struct platform_driver omap_rproc_driver = {
	.probe = omap_rproc_probe,
	.remove = __devexit_p(omap_rproc_remove),
	.driver = {
		.name = "omap-rproc",
		.owner = THIS_MODULE,
		.pm = GENERIC_RPROC_PM_OPS,
	},
};

static int __init omap_rproc_init(void)
{
	return platform_driver_register(&omap_rproc_driver);
}
/* must be ready in time for device_initcall users */
subsys_initcall(omap_rproc_init);

static void __exit omap_rproc_exit(void)
{
	platform_driver_unregister(&omap_rproc_driver);
}
module_exit(omap_rproc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Remote Processor control driver");
