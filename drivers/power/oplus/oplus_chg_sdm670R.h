/* SPDX-License-Identifier: GPL-2.0 */
/*
 * OPLUS sdm670R (OP46B1 / SDM710 + PMI632) platform glue types.
 *
 * struct qcom_pmic is the oplus wrapper hung off oplus_chg_chip->pmic_spmi;
 * it carries the qcom charger pointer (oplus_chip->pmic_spmi.chg), a struct
 * smb_charger * obtained from qcom's struct smb2 during probe_attach.
 *
 * Deliberately does NOT include framework/oplus_charger.h to avoid a cycle
 * (that header includes this one under CONFIG_OPLUS_SDM670R_CHARGER).
 */
#ifndef __OPLUS_CHG_SDM670R_H
#define __OPLUS_CHG_SDM670R_H

#include <linux/types.h>
#include <linux/bitops.h>

/* Pointer-only forward declarations (full defs from includer's scope). */
struct smb_charger;
struct oplus_chg_chip;
struct qpnp_vadc_chip;

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1),
};

struct qcom_pmic {
	struct smb_charger	*chg;
	struct qpnp_vadc_chip	*pm660_vadc_dev;
	struct qpnp_vadc_chip	*pm660_usbtemp_vadc_dev;

	/* for compile */
	bool			otg_pulse_skip_dis;
	int			pulse_cnt;
	unsigned int		therm_lvl_sel;
	bool			psy_registered;
	int			usb_online;

	/* copied from msm8976_pmic */
	int			bat_charging_state;
	bool			suspending;
	bool			aicl_suspend;
	bool			usb_hc_mode;
	int			usb_hc_count;
	bool			hc_mode_flag;
};

/*
 * extern declarations for oplus_smb_hook_impl.c — functions made non-static
 * in oplus_sdm670R_charger.c and oplus framework functions called from hooks.
 * Grouped by source unit for maintainability.
 */

/* ---- oplus_sdm670R_charger.c (previously static, now external) ---- */
void smbchg_aicl_enable(bool enable);
bool oplus_usbtemp_check_is_support(void);
int oplus_usbtemp_thread_init(void);
void oplus_usbtemp_thread_deinit(void);
int oplus_power_supply_init(struct oplus_chg_chip *chip);
void init_proc_dump_registers_mask(void);
void smbchg_enter_shipmode(struct oplus_chg_chip *chip);
int oplus_chg_get_fv(struct oplus_chg_chip *chip);
int oplus_chg_get_charging_current(struct oplus_chg_chip *chip);
int oplus_chg_parse_custom_dt(struct oplus_chg_chip *chip);
int oplus_chg_2uart_pinctrl_init(struct oplus_chg_chip *chip);
int get_current_time(unsigned long *now_tm_sec);
bool oplus_get_use_present_status(void);
int oplus_get_usb_status(void);

/* work handlers (used by post_smblib_init hook for INIT_WORK) */
void oplus_chg_monitor_work(struct work_struct *work);
void typec_disable_cmd_work(struct work_struct *work);
void oplus_divider_set_work(struct work_struct *work);

/* ---- oplus_sdm670R_charger.c (already non-static globals) ---- */
extern struct oplus_chg_operations smb2_chg_ops;
extern bool fg_oplus_set_input_current;
extern int usb_status;
extern bool usb_online_status;
void smbchg_set_chargerid_switch_val(int value);
void oplus_wake_up_usbtemp_thread(void);
bool oplus_chg_is_usb_present(void);
int oplus_tbatt_power_off_task_init(struct oplus_chg_chip *chip);
void switch_usb_state(int usb_state);

/* ---- oplus framework (oplus_charger.c etc., declared in their headers) ---- */
int oplus_chg_parse_svooc_dt(struct oplus_chg_chip *chip);
int oplus_chg_parse_charger_dt(struct oplus_chg_chip *chip);
int oplus_chg_init(struct oplus_chg_chip *chip);
int oplus_chg_get_prop_batt_health(struct oplus_chg_chip *chip);
void oplus_chg_soc_update_when_resume(unsigned long sleep_tm_sec);
void oplus_chg_set_allow_switch_to_fastchg(bool allow);
bool oplus_gauge_check_chip_is_null(void);
bool oplus_gauge_ic_chip_is_null(void);
bool oplus_vooc_check_chip_is_null(void);
bool oplus_adapter_check_chip_is_null(void);
void oplus_vooc_reset_mcu(void);
void oplus_vooc_switch_mode(int mode);
void oplus_vooc_reset_fastchg_after_usbout(void);
bool oplus_vooc_get_fastchg_started(void);
bool qpnp_is_power_off_charging(void);

/* kernel framework */
struct qpnp_vadc_chip;
struct qpnp_vadc_chip *qpnp_get_vadc(struct device *dev, const char *name);

#endif /* __OPLUS_CHG_SDM670R_H */
