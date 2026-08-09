/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_USB_OPLUS_DWC3_H
#define __LINUX_USB_OPLUS_DWC3_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_OPLUS_CHARGER) && IS_REACHABLE(CONFIG_USB_DWC3_MSM)
int oplus_dwc3_set_sink_only(bool sink_only);
#else
static inline int oplus_dwc3_set_sink_only(bool sink_only)
{
	return sink_only ? -ENODEV : 0;
}
#endif

#endif /* __LINUX_USB_OPLUS_DWC3_H */
