/* SPDX-License-Identifier: GPL-2.0 */
/*
 * OPLUS charging hooks for qcom smb-lib / qpnp-smb2 (OP46B1 / SDM710 + PMI632).
 *
 * This header defines the qcom->oplus reverse callback table (struct
 * oplus_smb_hook). The restored qcom smb-lib.c / qpnp-smb2.c invoke these
 * hooks at every original VENDOR_EDIT injection point through
 * oplus_hook_call() / oplus_hook_eval(). The oplus side implements the table
 * and stores it on struct smb_charger.oplus_hook during probe.
 *
 * With CONFIG_OPLUS_CHARGER disabled every hook compiles away to a no-op and
 * the qcom layer behaves bit-for-bit like the upstream original.
 *
 * Dependency direction (no cycles):
 *   smb-lib.h  -> includes this file (struct smb_charger owns an oplus_hook *)
 *   oplus_chg_sdm670R.h -> forward-declares struct smb_charger (struct qcom_pmic
 *                          holds a struct smb_charger *chg; no struct smb2 dep)
 *   oplus_smb_hook_impl.c -> implements the table, includes the above + framework
 */
#ifndef __OPLUS_SMB_HOOK_H
#define __OPLUS_SMB_HOOK_H

#include <linux/types.h>

/* Forward declarations only — full definitions come from the includer's scope. */
struct smb_charger;
struct oplus_chg_chip;
struct platform_device;
struct smb_irq_data;
union power_supply_propval;

#ifdef CONFIG_OPLUS_CHARGER

/* Re-evaluate VOOC Sink Only versus the normal DRP Type-C role. */
int oplus_smb_update_otg_policy(void);

/* Notify Android when the VOOC-backed USB properties change. */
void oplus_smb_notify_usb_changed(void);

enum oplus_usb_plugin_phase {
	OPLUS_USB_PLUGIN_PRE,
	OPLUS_USB_PLUGIN_RISING,
	OPLUS_USB_PLUGIN_FALLING,
	OPLUS_USB_PLUGIN_POST,
};

/**
 * struct oplus_smb_hook - qcom->oplus reverse callbacks.
 * Grouped by the qcom function whose VENDOR_EDIT injection points they serve.
 * Any member may be NULL; callers must gate on it (oplus_hook_call does).
 */
struct oplus_smb_hook {
	/* === smb2_probe multi-stage (oplus init interleaves with qcom steps) === */
	/* Create oplus_chip, parse svooc dt, dependency checks, set chg_ops,
	 * g_oplus_chip, acquire vadc — runs BEFORE qcom chip kzalloc. */
	struct oplus_chg_chip *(*probe_early)(struct platform_device *pdev);
	/* Link oplus_chip->pmic_spmi.chg = chg, set chg->pre_current_ma. */
	void (*probe_attach)(struct oplus_chg_chip *oc, struct smb_charger *chg);
	/* Register oplus ac/usb/batt psy (oplus_power_supply_init). Return non-zero
	 * to make qcom skip its dc/usb/batt psy registration. */
	int  (*probe_psy_init)(struct oplus_chg_chip *oc);
	/* Power-on charge reset, parse_charger_dt, oplus_chg_init, main psy FV/Icc. */
	int  (*probe_chg_init)(struct oplus_chg_chip *oc);
	/* Wake update work, tbatt task, proc dump, usbtemp thread and the
	 * charger-side typec-disable workaround. */
	int  (*probe_finalize)(struct oplus_chg_chip *oc);
	/* Replace qcom RRADC batt_health read (E27). */
	int  (*batt_health)(struct oplus_chg_chip *oc, int *health);

	/* === USB insertion / typec (B/C injection aggregation) === */
	void (*usb_plugin_locked)(struct smb_charger *chg, bool vbus_rising,
				  unsigned int phase);
	void (*usb_plugin_hard_reset)(struct smb_charger *chg, bool vbus_rising,
				      unsigned int phase);
	void (*usb_source_change)(struct smb_charger *chg, bool apsd_done);
	int  (*usb_source_change_rerun)(struct smb_charger *chg);
	int  (*usb_source_change_guard)(struct smb_charger *chg);

	/* === register init (D-class aggregation) === */
	int  (*post_smb2_init_hw)(struct smb_charger *chg);
	void (*post_smblib_init)(struct smb_charger *chg);

	/* === shutdown / pm === */
	/* @shipmode_phase is false before qcom cleanup, true after it. */
	void (*shutdown)(struct smb_charger *chg, bool shipmode_phase);
	int  (*pm_resume)(struct smb_charger *chg);
	int  (*pm_suspend)(struct smb_charger *chg);
	void (*remove)(struct smb_charger *chg);

	/* === smblib runtime VENDOR_EDIT fixups === */

	/* smblib_handle_apsd_done (sdm670R.c:4548): vote 500mA when
	 * fg_oplus_set_input_current==false. Side effect. */
	void (*apsd_done)(struct smb_charger *chg);

	/* smblib_handle_typec_insertion (sdm670R.c:5058): smbchg_aicl_enable(true).
	 * Side effect. */
	void (*typec_insertion)(struct smb_charger *chg);

	/* smblib_get_apsd_result (sdm670R.c:540): HVDCP2 result mapping. Return
	 * non-zero to suppress the unconditional HVDCP2 remap (oplus only remaps
	 * for the exact QC2.0 bit pattern). */
	int  (*apsd_suppress_hvdcp2_remap)(struct smb_charger *chg,
					   u8 result_bit);

	/* smblib_set_usb_suspend (sdm670R.c:655): usb_status forces suspend.
	 * Return non-zero to force suspend=true. */
	int  (*set_usb_suspend_force)(struct smb_charger *chg, bool suspend);

	/* smblib_update_usb_type (sdm670R.c:841): set usb_psy_desc.type alongside
	 * real_charger_type. Side effect. */
	void (*update_usb_type)(struct smb_charger *chg, int charger_type);

	/* smblib_rerun_apsd_if_required (sdm670R.c:1038): skip rerun for non
	 * USB/CDP types. Return non-zero to return 0 early. */
	int  (*rerun_apsd_suppress)(struct smb_charger *chg);

	/* set_sdp_current (sdm670R.c:1143): override icl_options for SDP. Side
	 * effect via *icl_options (oplus: <=150mA -> 0, else USB51_MODE_BIT). */
	void (*sdp_current_options)(struct smb_charger *chg, int icl_ua,
				    u8 *icl_options);

	/* smblib_set_icl_current (sdm670R.c:1218): else-branch SDP current
	 * 500000 vs 100000. Side effect via *sdp_ua. */
	void (*icl_hc_sdp_current)(struct smb_charger *chg, int *sdp_ua);

	/* smblib_hvdcp_enable_vote_callback (sdm670R.c:1544,1556): force disable
	 * HVDCP. Return non-zero if oplus handled (caller sets *val=0 and forces
	 * hvdcp_enable=0). */
	int  (*hvdcp_enable_fixup)(struct smb_charger *chg, int hvdcp_enable,
				   u8 *val);

	/* smblib_get_prop_from_bms (sdm670R.c:2377): return 50% when bms_psy is
	 * NULL instead of -EINVAL. Return non-zero if oplus handled. */
	int  (*bms_missing_default)(struct smb_charger *chg,
				    union power_supply_propval *val);

	/* smblib_get_prop_usb_online (sdm670R.c:2783): custom online status.
	 * Return non-zero if oplus handled (filled val). */
	int  (*usb_online_status)(struct smb_charger *chg,
				  union power_supply_propval *val);
	int  (*usb_get_property)(struct smb_charger *chg, int psp,
				 union power_supply_propval *val);
	int  (*usb_use_present_status)(struct smb_charger *chg);

	/* Force Sink Only while the VOOC D+/D- path is active. */
	int  (*force_typec_sink)(struct smb_charger *chg);

	/* smblib_handle_dc_plugin (sdm670R.c:5266): NULL-guard dc_psy.
	 * Return non-zero to skip power_supply_changed(). */
	int  (*dc_plugin_guard)(struct smb_charger *chg);

	/* smblib_handle_switcher_power_ok (sdm670R.c:5333): replace weak-charger /
	 * boost-back storm logic. Return non-zero if oplus handled the storm
	 * branch. */
	int  (*switcher_power_ok_storm)(struct smb_charger *chg,
					struct smb_irq_data *irq_data);
};

/* Singleton table implemented by oplus_smb_hook_impl.c. */
extern const struct oplus_smb_hook oplus_smb_hooks;

/*
 * Fire-and-forget hook. @chg must point to a struct smb_charger (which owns
 * ->oplus_hook). Becomes a no-op when no hook is registered.
 */
#define oplus_hook_call(chg, fn, ...) \
	do { \
		if ((chg)->oplus_hook && (chg)->oplus_hook->fn) \
			(chg)->oplus_hook->fn((chg), ##__VA_ARGS__); \
	} while (0)

/*
 * Evaluating hook for conditional-replacement points. Sets @ret to the hook's
 * return value (0 when no hook). Non-zero @ret means "oplus handled it"; qcom
 * should skip its original branch and fall through otherwise.
 */
#define oplus_hook_eval(ret, chg, fn, ...) \
	({ \
		(ret) = 0; \
		if ((chg)->oplus_hook && (chg)->oplus_hook->fn) \
			(ret) = (chg)->oplus_hook->fn((chg), ##__VA_ARGS__); \
	})

#else /* !CONFIG_OPLUS_CHARGER */

/* Empty stub so struct smb_charger can still name the member type. */
struct oplus_smb_hook { };

#define oplus_hook_call(chg, fn, ...)		do { } while (0)
#define oplus_hook_eval(ret, chg, fn, ...)	((ret) = 0)

#endif /* CONFIG_OPLUS_CHARGER */

#endif /* __OPLUS_SMB_HOOK_H */
