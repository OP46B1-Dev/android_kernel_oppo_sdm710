/* Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/pm_qos.h>
#include <soc/qcom/pm.h>
#include <sound/soc.h>
#include "msm-analog-cdc.h"
#include "sdm660-cdc-irq.h"
#include "sdm660-cdc-registers.h"

#define MAX_NUM_IRQS 14
#define NUM_IRQ_REGS 2
#ifdef CONFIG_MACH_OPLUS_SDM710
#define WCD9XXX_SYSTEM_RESUME_TIMEOUT_MS 2000
#else
#define WCD9XXX_SYSTEM_RESUME_TIMEOUT_MS 700
#endif

#define BYTE_BIT_MASK(nr) (1UL << ((nr) % BITS_PER_BYTE))
#define BIT_BYTE(nr) ((nr) / BITS_PER_BYTE)

static irqreturn_t wcd9xxx_spmi_irq_handler(int linux_irq, void *data);

char *irq_names[MAX_NUM_IRQS] = {
	"spk_cnp_int",
	"spk_clip_int",
	"spk_ocp_int",
	"ins_rem_det1",
	"but_rel_det",
	"but_press_det",
	"ins_rem_det",
	"mbhc_int",
	"ear_ocp_int",
	"hphr_ocp_int",
	"hphl_ocp_det",
	"ear_cnp_int",
	"hphr_cnp_int",
	"hphl_cnp_int"
};

int order[MAX_NUM_IRQS] = {
	MSM89XX_IRQ_SPKR_CNP,
	MSM89XX_IRQ_SPKR_CLIP,
	MSM89XX_IRQ_SPKR_OCP,
	MSM89XX_IRQ_MBHC_INSREM_DET1,
	MSM89XX_IRQ_MBHC_RELEASE,
	MSM89XX_IRQ_MBHC_PRESS,
	MSM89XX_IRQ_MBHC_INSREM_DET,
	MSM89XX_IRQ_MBHC_HS_DET,
	MSM89XX_IRQ_EAR_OCP,
	MSM89XX_IRQ_HPHR_OCP,
	MSM89XX_IRQ_HPHL_OCP,
	MSM89XX_IRQ_EAR_CNP,
	MSM89XX_IRQ_HPHR_CNP,
	MSM89XX_IRQ_HPHL_CNP,
};

enum wcd9xxx_spmi_pm_state {
	WCD9XXX_PM_SLEEPABLE,
	WCD9XXX_PM_AWAKE,
	WCD9XXX_PM_ASLEEP,
};

struct wcd9xxx_spmi_map {
	uint8_t handled[NUM_IRQ_REGS];
	uint8_t mask[NUM_IRQ_REGS];
	int linuxirq[MAX_NUM_IRQS];
	irq_handler_t handler[MAX_NUM_IRQS];
	spinlock_t irq_lock;
	struct platform_device *spmi[NUM_IRQ_REGS];
	struct snd_soc_codec *codec;

	enum wcd9xxx_spmi_pm_state pm_state;
	struct mutex pm_lock;
	/* pm_wq notifies change of pm_state */
	wait_queue_head_t pm_wq;
	struct pm_qos_request pm_qos_req;
	int wlock_holders;
};

struct wcd9xxx_spmi_map map;

static bool wcd9xxx_spmi_irq_valid(int irq)
{
	return irq >= 0 && irq < MAX_NUM_IRQS;
}

void wcd9xxx_spmi_enable_irq(int irq)
{
	unsigned long flags;
	int linux_irq = -1;
	bool enable = false;

	pr_debug("%s: irqno =%d\n", __func__, irq);

	if (!wcd9xxx_spmi_irq_valid(irq)) {
		pr_warn("%s: invalid irq index %d\n", __func__, irq);
		return;
	}

	spin_lock_irqsave(&map.irq_lock, flags);
	if (map.linuxirq[irq] >= 0 &&
	    (map.mask[BIT_BYTE(irq)] & BYTE_BIT_MASK(irq))) {
		map.mask[BIT_BYTE(irq)] &= ~BYTE_BIT_MASK(irq);
		linux_irq = map.linuxirq[irq];
		enable = true;
	}
	spin_unlock_irqrestore(&map.irq_lock, flags);

	if (enable)
		enable_irq(linux_irq);
}

void wcd9xxx_spmi_disable_irq(int irq)
{
	unsigned long flags;
	int linux_irq = -1;
	bool disable = false;

	pr_debug("%s: irqno =%d\n", __func__, irq);

	if (!wcd9xxx_spmi_irq_valid(irq)) {
		pr_warn("%s: invalid irq index %d\n", __func__, irq);
		return;
	}

	spin_lock_irqsave(&map.irq_lock, flags);
	if (map.linuxirq[irq] >= 0 &&
	    !(map.mask[BIT_BYTE(irq)] & BYTE_BIT_MASK(irq))) {
		/* Set the software mask before disabling the Linux IRQ. */
		map.mask[BIT_BYTE(irq)] |= BYTE_BIT_MASK(irq);
		linux_irq = map.linuxirq[irq];
		disable = true;
	}
	spin_unlock_irqrestore(&map.irq_lock, flags);

	if (disable)
		disable_irq_nosync(linux_irq);
}

int wcd9xxx_spmi_request_irq(int irq, irq_handler_t handler,
			const char *name, void *priv)
{
	struct platform_device *spmi;
	unsigned long flags;
	int rc;
	int linux_irq;
	unsigned long irq_flags;

	if (!wcd9xxx_spmi_irq_valid(irq) || !handler || !name)
		return -EINVAL;

	spmi = map.spmi[BIT_BYTE(irq)];
	if (!spmi)
		return -ENODEV;

	linux_irq = platform_get_irq_byname(spmi, irq_names[irq]);
	if (linux_irq < 0) {
		dev_err(&spmi->dev, "Can't get %s IRQ: %d\n",
			irq_names[irq], linux_irq);
		return linux_irq;
	}

	/* Keep the source masked until both the Linux IRQ and callback exist. */
	spin_lock_irqsave(&map.irq_lock, flags);
	if (map.linuxirq[irq] >= 0 || map.handler[irq]) {
		spin_unlock_irqrestore(&map.irq_lock, flags);
		return -EBUSY;
	}
	map.linuxirq[irq] = linux_irq;
	map.mask[BIT_BYTE(irq)] |= BYTE_BIT_MASK(irq);
	spin_unlock_irqrestore(&map.irq_lock, flags);

	if (strcmp(name, "mbhc sw intr"))
		irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT;
	else
		irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT | IRQF_NO_SUSPEND;
	pr_debug("%s: name:%s irq_flags = %lx\n", __func__, name, irq_flags);

	rc = devm_request_threaded_irq(&spmi->dev, linux_irq, NULL,
				wcd9xxx_spmi_irq_handler,
				irq_flags, name, priv);
	if (rc < 0) {
		dev_err(&spmi->dev, "Can't request %d IRQ\n", irq);
		spin_lock_irqsave(&map.irq_lock, flags);
		map.linuxirq[irq] = -1;
		spin_unlock_irqrestore(&map.irq_lock, flags);
		return rc;
	}

	dev_dbg(&spmi->dev, "irq %d linuxIRQ: %d\n", irq, linux_irq);
	spin_lock_irqsave(&map.irq_lock, flags);
	WRITE_ONCE(map.handler[irq], handler);
	/* Publish the callback before allowing the child IRQ to dispatch. */
	smp_wmb();
	map.mask[BIT_BYTE(irq)] &= ~BYTE_BIT_MASK(irq);
	spin_unlock_irqrestore(&map.irq_lock, flags);
	enable_irq_wake(linux_irq);
	return 0;
}

int wcd9xxx_spmi_free_irq(int irq, void *priv)
{
	struct platform_device *spmi;
	unsigned long flags;
	int linux_irq;

	if (!wcd9xxx_spmi_irq_valid(irq))
		return -EINVAL;

	spmi = map.spmi[BIT_BYTE(irq)];
	if (!spmi)
		return -ENODEV;

	/*
	 * Clear the callback before releasing the Linux IRQ so a pending event
	 * cannot call a handler after its owner has been torn down.
	 */
	spin_lock_irqsave(&map.irq_lock, flags);
	linux_irq = map.linuxirq[irq];
	if (linux_irq < 0) {
		spin_unlock_irqrestore(&map.irq_lock, flags);
		return -EINVAL;
	}
	map.mask[BIT_BYTE(irq)] |= BYTE_BIT_MASK(irq);
	WRITE_ONCE(map.handler[irq], NULL);
	spin_unlock_irqrestore(&map.irq_lock, flags);

	devm_free_irq(&spmi->dev, linux_irq, priv);

	spin_lock_irqsave(&map.irq_lock, flags);
	map.linuxirq[irq] = -1;
	spin_unlock_irqrestore(&map.irq_lock, flags);
	return 0;
}

static int get_irq_bit(int linux_irq)
{
	int i = 0;

	for (; i < MAX_NUM_IRQS; i++)
		if (map.linuxirq[i] == linux_irq)
			return i;

	return i;
}

#ifndef CONFIG_MACH_OPLUS_SDM710
static int get_order_irq(int i)
{
	return order[i];
}
#endif

static irqreturn_t wcd9xxx_spmi_irq_handler(int linux_irq, void *data)
{
	int irq;
	unsigned long flags;
	irq_handler_t handler;
#ifndef CONFIG_MACH_OPLUS_SDM710
	int i, j;
	unsigned long status[NUM_IRQ_REGS] = {0};
#endif

	irq = get_irq_bit(linux_irq);
	if (!wcd9xxx_spmi_irq_valid(irq)) {
		pr_warn("%s: unknown Linux IRQ %d\n", __func__, linux_irq);
		return IRQ_NONE;
	}

	if (unlikely(wcd9xxx_spmi_lock_sleep() == false)) {
		pr_err("Failed to hold suspend\n");
		return IRQ_NONE;
	}

#ifdef CONFIG_MACH_OPLUS_SDM710
	/*
	 * The OP46B1 PM660L child IRQ is the source of truth.  Its
	 * codec-side INT_LATCHED_STS cannot be relied on for this level-style
	 * interrupt, so the CAF latch gate would drop the event.  Honor the
	 * software mask and dispatch the callback identified by the Linux IRQ.
	 */
	spin_lock_irqsave(&map.irq_lock, flags);
	if (map.mask[BIT_BYTE(irq)] & BYTE_BIT_MASK(irq)) {
		spin_unlock_irqrestore(&map.irq_lock, flags);
		pr_debug("%s: irq %d is software-masked\n", __func__, irq);
		goto out;
	}
	handler = READ_ONCE(map.handler[irq]);
	spin_unlock_irqrestore(&map.irq_lock, flags);

	if (handler) {
		if (irq == MSM89XX_IRQ_MBHC_HS_DET)
			pr_info_ratelimited("%s: dispatch MBHC irq=%d linux_irq=%d\n",
					__func__, irq, linux_irq);
		handler(irq, data);
	} else {
		pr_warn("%s: no handler registered for irq %d\n", __func__, irq);
	}
#else
	status[BIT_BYTE(irq)] |= BYTE_BIT_MASK(irq);
	for (i = 0; i < NUM_IRQ_REGS; i++) {
		status[i] |= snd_soc_read(map.codec,
				BIT_BYTE(irq) * 0x100 +
				MSM89XX_PMIC_DIGITAL_INT_LATCHED_STS);
		status[i] &= ~map.mask[i];
	}
	for (i = 0; i < MAX_NUM_IRQS; i++) {
		j = get_order_irq(i);
		if ((status[BIT_BYTE(j)] & BYTE_BIT_MASK(j)) &&
			((map.handled[BIT_BYTE(j)] &
			BYTE_BIT_MASK(j)) == 0)) {
			handler = READ_ONCE(map.handler[j]);
			if (!handler)
				continue;
			handler(irq, data);
			map.handled[BIT_BYTE(j)] |=
					BYTE_BIT_MASK(j);
		}
	}
	spin_lock_irqsave(&map.irq_lock, flags);
	map.handled[BIT_BYTE(irq)] &= ~BYTE_BIT_MASK(irq);
	spin_unlock_irqrestore(&map.irq_lock, flags);
#endif

out:
	wcd9xxx_spmi_unlock_sleep();

	return IRQ_HANDLED;
}

enum wcd9xxx_spmi_pm_state wcd9xxx_spmi_pm_cmpxchg(
		enum wcd9xxx_spmi_pm_state o,
		enum wcd9xxx_spmi_pm_state n)
{
	enum wcd9xxx_spmi_pm_state old;

	mutex_lock(&map.pm_lock);
	old = map.pm_state;
	if (old == o)
		map.pm_state = n;
	pr_debug("%s: map.pm_state = %d\n", __func__, map.pm_state);
	mutex_unlock(&map.pm_lock);
	return old;
}
EXPORT_SYMBOL(wcd9xxx_spmi_pm_cmpxchg);

int wcd9xxx_spmi_suspend(pm_message_t pmesg)
{
	int ret = 0;

	pr_debug("%s: enter\n", __func__);
	/*
	 * pm_qos_update_request() can be called after this suspend chain call
	 * started. thus suspend can be called while lock is being held
	 */
	mutex_lock(&map.pm_lock);
	if (map.pm_state == WCD9XXX_PM_SLEEPABLE) {
		pr_debug("%s: suspending system, state %d, wlock %d\n",
			 __func__, map.pm_state,
			 map.wlock_holders);
		map.pm_state = WCD9XXX_PM_ASLEEP;
	} else if (map.pm_state == WCD9XXX_PM_AWAKE) {
		/*
		 * unlock to wait for pm_state == WCD9XXX_PM_SLEEPABLE
		 * then set to WCD9XXX_PM_ASLEEP
		 */
		pr_debug("%s: waiting to suspend system, state %d, wlock %d\n",
			 __func__, map.pm_state,
			 map.wlock_holders);
		mutex_unlock(&map.pm_lock);
		if (!(wait_event_timeout(map.pm_wq,
					 wcd9xxx_spmi_pm_cmpxchg(
							WCD9XXX_PM_SLEEPABLE,
							WCD9XXX_PM_ASLEEP) ==
							WCD9XXX_PM_SLEEPABLE,
							HZ))) {
			pr_debug("%s: suspend failed state %d, wlock %d\n",
				 __func__, map.pm_state,
				 map.wlock_holders);
			ret = -EBUSY;
		} else {
			pr_debug("%s: done, state %d, wlock %d\n", __func__,
				 map.pm_state,
				 map.wlock_holders);
		}
		mutex_lock(&map.pm_lock);
	} else if (map.pm_state == WCD9XXX_PM_ASLEEP) {
		pr_warn("%s: system is already suspended, state %d, wlock %dn",
			__func__, map.pm_state,
			map.wlock_holders);
	}
	mutex_unlock(&map.pm_lock);

	return ret;
}
EXPORT_SYMBOL(wcd9xxx_spmi_suspend);

int wcd9xxx_spmi_resume(void)
{
	int ret = 0;

	pr_debug("%s: enter\n", __func__);
	mutex_lock(&map.pm_lock);
	if (map.pm_state == WCD9XXX_PM_ASLEEP) {
		pr_debug("%s: resuming system, state %d, wlock %d\n", __func__,
				map.pm_state,
				map.wlock_holders);
		map.pm_state = WCD9XXX_PM_SLEEPABLE;
	} else {
		pr_warn("%s: system is already awake, state %d wlock %d\n",
				__func__, map.pm_state,
				map.wlock_holders);
	}
	mutex_unlock(&map.pm_lock);
	wake_up_all(&map.pm_wq);

	return ret;
}
EXPORT_SYMBOL(wcd9xxx_spmi_resume);

bool wcd9xxx_spmi_lock_sleep(void)
{
	/*
	 * wcd9xxx_spmi_{lock/unlock}_sleep will be called by
	 * wcd9xxx_spmi_irq_thread
	 * and its subroutines only motly.
	 * but btn0_lpress_fn is not wcd9xxx_spmi_irq_thread's subroutine and
	 * It can race with wcd9xxx_spmi_irq_thread.
	 * So need to embrace wlock_holders with mutex.
	 */
	mutex_lock(&map.pm_lock);
	if (map.wlock_holders++ == 0) {
		pr_debug("%s: holding wake lock\n", __func__);
		pm_qos_update_request(&map.pm_qos_req,
				      msm_cpuidle_get_deep_idle_latency());
		pm_stay_awake(&map.spmi[0]->dev);
	}
	mutex_unlock(&map.pm_lock);
	pr_debug("%s: wake lock counter %d\n", __func__,
			map.wlock_holders);
	pr_debug("%s: map.pm_state = %d\n", __func__, map.pm_state);

	if (!wait_event_timeout(map.pm_wq,
				((wcd9xxx_spmi_pm_cmpxchg(
					WCD9XXX_PM_SLEEPABLE,
					WCD9XXX_PM_AWAKE)) ==
					WCD9XXX_PM_SLEEPABLE ||
					(wcd9xxx_spmi_pm_cmpxchg(
						 WCD9XXX_PM_SLEEPABLE,
						 WCD9XXX_PM_AWAKE) ==
						 WCD9XXX_PM_AWAKE)),
					msecs_to_jiffies(
					WCD9XXX_SYSTEM_RESUME_TIMEOUT_MS))) {
		pr_warn("%s: system didn't resume within %dms, s %d, w %d\n",
			__func__,
			WCD9XXX_SYSTEM_RESUME_TIMEOUT_MS, map.pm_state,
			map.wlock_holders);
		wcd9xxx_spmi_unlock_sleep();
		return false;
	}
	wake_up_all(&map.pm_wq);
	pr_debug("%s: leaving pm_state = %d\n", __func__, map.pm_state);
	return true;
}
EXPORT_SYMBOL(wcd9xxx_spmi_lock_sleep);

void wcd9xxx_spmi_unlock_sleep(void)
{
	mutex_lock(&map.pm_lock);
	if (--map.wlock_holders == 0) {
		pr_debug("%s: releasing wake lock pm_state %d -> %d\n",
			 __func__, map.pm_state, WCD9XXX_PM_SLEEPABLE);
		/*
		 * if wcd9xxx_spmi_lock_sleep failed, pm_state would be still
		 * WCD9XXX_PM_ASLEEP, don't overwrite
		 */
		if (likely(map.pm_state == WCD9XXX_PM_AWAKE))
			map.pm_state = WCD9XXX_PM_SLEEPABLE;
		pm_qos_update_request(&map.pm_qos_req,
				PM_QOS_DEFAULT_VALUE);
		pm_relax(&map.spmi[0]->dev);
	}
	mutex_unlock(&map.pm_lock);
	pr_debug("%s: wake lock counter %d\n", __func__,
			map.wlock_holders);
	pr_debug("%s: map.pm_state = %d\n", __func__, map.pm_state);
	wake_up_all(&map.pm_wq);
}
EXPORT_SYMBOL(wcd9xxx_spmi_unlock_sleep);

void wcd9xxx_spmi_set_codec(struct snd_soc_codec *codec)
{
	map.codec = codec;
}

void wcd9xxx_spmi_set_dev(struct platform_device *spmi, int i)
{
	if (spmi && i >= 0 && i < NUM_IRQ_REGS)
		map.spmi[i] = spmi;
}

int wcd9xxx_spmi_irq_init(void)
{
	int i = 0;

	spin_lock_init(&map.irq_lock);
	for (; i < MAX_NUM_IRQS; i++) {
		map.mask[BIT_BYTE(i)] |= BYTE_BIT_MASK(i);
		map.linuxirq[i] = -1;
		WRITE_ONCE(map.handler[i], NULL);
	}
	mutex_init(&map.pm_lock);
	map.wlock_holders = 0;
	map.pm_state = WCD9XXX_PM_SLEEPABLE;
	init_waitqueue_head(&map.pm_wq);
	pm_qos_add_request(&map.pm_qos_req,
				PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	return 0;
}

MODULE_DESCRIPTION("MSM8x16 SPMI IRQ driver");
MODULE_LICENSE("GPL v2");
