// SPDX-License-Identifier: GPL-2.0
/*
 * OPLUS smb-lib / qpnp-smb2 hook implementations (OP46B1 / SDM710 + PMI632).
 *
 * Each callback here reproduces the original VENDOR_EDIT injection logic that
 * lived inline in oplus_battery_sdm670R.c, calling into the oplus platform
 * business functions kept in oplus_sdm670R_charger.c. The singleton table
 * oplus_smb_hooks is stored on chg->oplus_hook during smb2_probe.
 */
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/pmic-voter.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/pinctrl/consumer.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/usb/oplus_dwc3.h>

#include <soc/oplus/oplus_project.h>
#include <soc/oplus/boot_mode.h>

#include "oplus_smb_hook.h"
#include "oplus_chg_sdm670R.h"
#include "oplus_charger.h"
#include "oplus_vooc.h"

#include "../supply/qcom/smb-reg.h"
#include "../supply/qcom/smb-lib.h"

/* Defined in oplus_sdm670R_charger.c — the cross-context bridge. */
extern struct oplus_chg_chip *g_oplus_chip;

#define OPLUS_VOOC_ADAPTER_CURRENT_MAX_UA	4000000

/* PM resume/suspend RTC sleep-time tracking (file-scope static). */
static unsigned long oplus_suspend_tm_sec = 0;

static int oplus_get_usb_input_present(struct smb_charger *chg, bool *present)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0)
		return rc;

	*present = !!(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

/* VOOC owns the only explicit role override; all other sessions stay DRP. */
static int oplus_set_pmic_role(struct smb_charger *chg, bool sink_only)
{
	u8 desired = sink_only ? UFP_EN_CMD_BIT : 0;
	u8 stat;
	int rc;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't read Type-C power role rc=%d\n", rc);
		return rc;
	}
	if ((stat & TYPEC_POWER_ROLE_CMD_MASK) == desired)
		return 0;

	rc = smblib_masked_write(chg,
				 TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, desired);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set Type-C role to %s rc=%d\n",
			sink_only ? "Sink Only" : "DRP", rc);
		return rc;
	}

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't verify Type-C role rc=%d\n", rc);
		return rc;
	}
	if ((stat & TYPEC_POWER_ROLE_CMD_MASK) != desired) {
		dev_err(chg->dev,
			"Type-C role verify failed: wanted=0x%02x read=0x%02x\n",
			desired, stat);
		return -EIO;
	}

	dev_info(chg->dev, "OPLUS effective Type-C role=%s reg=0x%02x\n",
		 sink_only ? "Sink Only" : "DRP", stat);
	return 0;
}

/* The caller must serialize charger register access with chg->lock. */
static int oplus_apply_sink_policy(struct smb_charger *chg)
{
	bool sink_only = oplus_vooc_get_oppo_power_active();
	int rc;

	rc = oplus_set_pmic_role(chg, sink_only);
	if (rc < 0)
		return rc;

	rc = oplus_dwc3_set_sink_only(sink_only);
	if (rc == -ENODEV)
		rc = 0;
	if (rc < 0)
		return rc;

	return 0;
}

static int oplus_force_typec_sink(struct smb_charger *chg)
{
	(void)chg;
	return oplus_vooc_get_oppo_power_active();
}

int oplus_smb_update_otg_policy(void)
{
	struct oplus_chg_chip *oc = READ_ONCE(g_oplus_chip);
	struct smb_charger *chg;
	int rc;

	if (!oc || !oc->pmic_spmi.chg)
		return -ENODEV;

	chg = oc->pmic_spmi.chg;
	mutex_lock(&chg->lock);
	if (oc != READ_ONCE(g_oplus_chip) ||
	    chg->oplus_hook != &oplus_smb_hooks)
		rc = -ENODEV;
	else
		rc = oplus_apply_sink_policy(chg);
	mutex_unlock(&chg->lock);

	return rc;
}

void oplus_smb_notify_usb_changed(void)
{
	struct oplus_chg_chip *oc = READ_ONCE(g_oplus_chip);
	struct smb_charger *chg;

	if (!oc || !oc->pmic_spmi.chg)
		return;

	chg = oc->pmic_spmi.chg;
	if (chg->usb_psy)
		power_supply_changed(chg->usb_psy);
}

/* ===================== smb2_probe multi-stage ===================== */

/*
 * sdm670R.c:11733-11812: devm_kzalloc oplus_chip, parse_svooc_dt, dependency
 * checks (vooc/gauge/charger_ic/adapter), set chg_ops, g_oplus_chip, acquire
 * pm660 vadc. Runs BEFORE qcom chip kzalloc.
 */
static struct oplus_chg_chip *oplus_probe_early(struct platform_device *pdev)
{
	struct oplus_chg_chip *oplus_chip;
	int rc;

	oplus_chip = devm_kzalloc(&pdev->dev, sizeof(*oplus_chip), GFP_KERNEL);
	if (!oplus_chip)
		return ERR_PTR(-ENOMEM);

	oplus_chip->dev = &pdev->dev;
	rc = oplus_chg_parse_svooc_dt(oplus_chip);
	if (rc < 0)
		return ERR_PTR(rc);

	if (oplus_chip->vbatt_num == 1) {
		if (oplus_gauge_check_chip_is_null()) {
			chg_err("gauge chip null, will do after bettery init.\n");
			return ERR_PTR(-EPROBE_DEFER);
		}
		oplus_chip->chg_ops = &smb2_chg_ops;
	} else {
		chg_err("multi-cell config (vbatt_num=%d) not supported\n",
			oplus_chip->vbatt_num);
		return ERR_PTR(-ENOTSUPP);
	}
	/* sdm670R.c:11782-11812: acquire pm660 vadc devices */
	if (of_find_property(oplus_chip->dev->of_node,
			     "qcom,pm660chg-vadc", NULL)) {
		oplus_chip->pmic_spmi.pm660_vadc_dev =
			qpnp_get_vadc(oplus_chip->dev, "pm660chg");
		if (IS_ERR(oplus_chip->pmic_spmi.pm660_vadc_dev)) {
			rc = PTR_ERR(oplus_chip->pmic_spmi.pm660_vadc_dev);
			oplus_chip->pmic_spmi.pm660_vadc_dev = NULL;
			if (rc != -EPROBE_DEFER)
				chg_err("Couldn't get vadc rc=%d\n", rc);
			else {
				chg_err("Couldn't get vadc, try again...\n");
				return ERR_PTR(-EPROBE_DEFER);
			}
		}
	}

	if (of_find_property(oplus_chip->dev->of_node,
			     "qcom,pm660usbtemp-vadc", NULL)) {
		oplus_chip->pmic_spmi.pm660_usbtemp_vadc_dev =
			qpnp_get_vadc(oplus_chip->dev, "pm660usbtemp");
		if (IS_ERR(oplus_chip->pmic_spmi.pm660_usbtemp_vadc_dev)) {
			rc = PTR_ERR(oplus_chip->pmic_spmi.pm660_usbtemp_vadc_dev);
			oplus_chip->pmic_spmi.pm660_usbtemp_vadc_dev = NULL;
			if (rc != -EPROBE_DEFER)
				chg_err("Couldn't get usbtemp vadc rc=%d\n", rc);
			else {
				chg_err("Couldn't get usbtemp vadc, try again...\n");
				return ERR_PTR(-EPROBE_DEFER);
			}
		}
	}

	chg_debug("SMB2_Probe Start----\n");

	return oplus_chip;
}

/* sdm670R.c:11763-11780: oplus_chip->pmic_spmi.chg = chg;
 * chg->pre_current_ma = -1 */
static void oplus_probe_attach(struct oplus_chg_chip *oc,
			       struct smb_charger *chg)
{
	oc->pmic_spmi.chg = chg;
	g_oplus_chip = oc;
	chg->oplus_hook = &oplus_smb_hooks;
	chg->pre_current_ma = -1;
}

/*
 * sdm670R.c:11886-11919 (E24/E25/E26): oplus_power_supply_init (ac/usb/batt
 * psy). Return non-zero so qcom skips smb2_init_dc_psy / smb2_init_usb_psy /
 * smb2_init_batt_psy.
 */
static int oplus_probe_psy_init(struct oplus_chg_chip *oc)
{
	int rc;

	rc = oplus_power_supply_init(oc);
	if (rc < 0) {
		pr_err("Couldn't initialize oplus psy rc=%d\n", rc);
		return rc;
	}
	return 1; /* non-zero: qcom skips its dc/usb/batt psy registration */
}

/*
 * sdm670R.c:11920-11950: power-on charge reset, parse_charger_dt,
 * oplus_chg_init(oplus_chip), main psy FV/Icc.
 */
static int oplus_probe_chg_init(struct oplus_chg_chip *oc)
{
	struct smb_charger *chg = oc->pmic_spmi.chg;
	struct power_supply *main_psy = NULL;
	union power_supply_propval pval = {0, };
	int rc;

	/* power-on charge reset */
	if (oplus_chg_is_usb_present()) {
		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
					 CHARGING_ENABLE_CMD_BIT, 0);
		if (rc < 0)
			pr_err("Couldn't disable at bootup rc=%d\n", rc);
		msleep(100);
		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
					 CHARGING_ENABLE_CMD_BIT,
					 CHARGING_ENABLE_CMD_BIT);
		if (rc < 0)
			pr_err("Couldn't enable at bootup rc=%d\n", rc);
	}

	/* DT parse + init */
	rc = oplus_chg_parse_custom_dt(oc);
	if (rc < 0)
		return rc;
	rc = oplus_chg_parse_charger_dt(oc);
	if (rc < 0)
		return rc;
	rc = oplus_chg_2uart_pinctrl_init(oc);
	if (rc < 0)
		return rc;
	rc = oplus_chg_init(oc);
	if (rc < 0)
		return rc;

	/* set main psy FV / Icc */
	main_psy = power_supply_get_by_name("main");
	if (main_psy) {
		pval.intval = 1000 * oplus_chg_get_fv(oc);
		power_supply_set_property(main_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
		pval.intval = 1000 * oplus_chg_get_charging_current(oc);
		power_supply_set_property(main_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				&pval);
		power_supply_put(main_psy);
	}

	return 0;
}

/*
 * sdm670R.c:11964-12037: wake_update_work, tbatt task,
 * init_proc_dump_registers_mask, usbtemp_thread_init, typec_disable schedule.
 * The VOOC Sink Only/ordinary DRP policy is reasserted at the matching
 * post-IRQ point before the remaining OPLUS services are started.
 */
static int oplus_probe_finalize(struct oplus_chg_chip *oc)
{
	struct smb_charger *chg = oc->pmic_spmi.chg;
	union power_supply_propval val = {0, };
	int rc;

	/*
	 * Re-assert DRP, or VOOC Sink Only if that session is already active.
	 */
	rc = oplus_apply_sink_policy(chg);
	if (rc < 0)
		return rc;

	/* wake update work + tbatt power-off task */
	oplus_chg_wake_update_work();
	if (qpnp_is_power_off_charging() == false) {
		rc = oplus_tbatt_power_off_task_init(oc);
		if (rc < 0)
			return rc;
	}

	/* init proc_dump_registers_mask */
	init_proc_dump_registers_mask();

	/* usbtemp thread */
	if (oplus_usbtemp_check_is_support() == true) {
		rc = oplus_usbtemp_thread_init();
		if (rc < 0)
			return rc;
	}

	/* typec_disable schedule if usb present */
	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't get usb present rc=%d\n", rc);
	} else if (val.intval) {
		schedule_delayed_work(&chg->typec_disable_cmd_work,
				      msecs_to_jiffies(1000));
	}

	return 0;
}

/* sdm670R.c:12005-12014 (E27): oplus_chg_get_prop_batt_health replaces RRADC. */
static int oplus_batt_health(struct oplus_chg_chip *oc, int *health)
{
	*health = oplus_chg_get_prop_batt_health(oc);
	return 1; /* handled */
}

/* ===================== USB insertion / typec ===================== */

/*
 * sdm670R.c:4091-4210 (8 injections): aicl_enable, vooc reset,
 * chg_monitor/divider/dpdm work scheduling, fake_typec handling, switch_usb.
 */
static void oplus_usb_plugin_removed(struct smb_charger *chg,
				     bool reset_apsd_rerun)
{
	oplus_vooc_reset_fastchg_after_usbout();
	if (oplus_vooc_get_fastchg_started() == false && g_oplus_chip) {
		smbchg_set_chargerid_switch_val(0);
		g_oplus_chip->chargerid_volt = 0;
		g_oplus_chip->chargerid_volt_got = false;
		g_oplus_chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		oplus_chg_wake_update_work();
	}
	chg->pd_sdp = false;
	chg->pre_current_ma = -1;
	if (reset_apsd_rerun)
		chg->uusb_apsd_rerun_done = false;
}

static void oplus_usb_plugin_post(struct smb_charger *chg, bool vbus_rising)
{
	if (vbus_rising) {
		cancel_delayed_work_sync(&chg->chg_monitor_work);
		schedule_delayed_work(&chg->chg_monitor_work,
				      round_jiffies_relative(
					      msecs_to_jiffies(5000)));
		oplus_wake_up_usbtemp_thread();
		cancel_delayed_work_sync(&chg->divider_set_work);
		schedule_delayed_work(&chg->divider_set_work, 0);
	} else {
		fg_oplus_set_input_current = false;
		cancel_delayed_work_sync(&chg->chg_monitor_work);
		cancel_delayed_work_sync(&chg->divider_set_work);
		schedule_delayed_work(&chg->divider_set_work, 0);
	}

	if (chg->fake_typec_insertion == true && !vbus_rising) {
		chg->fake_typec_insertion = false;
		chg->typec_mode = POWER_SUPPLY_TYPEC_NONE;
	}

}

static void oplus_usb_plugin_locked(struct smb_charger *chg, bool vbus_rising,
				    unsigned int phase)
{
	int rc;

	switch (phase) {
	case OPLUS_USB_PLUGIN_PRE:
		/*
		 * This runs before qcom checks the cached DFP state. Ordinary
		 * charging selects DRP here; OPPO Power stays Sink Only.
		 */
		if (vbus_rising) {
			rc = oplus_apply_sink_policy(chg);
			if (rc < 0)
				dev_err(chg->dev,
					"Couldn't apply Type-C role policy on USB attach rc=%d\n",
					rc);
		}
		break;
	case OPLUS_USB_PLUGIN_RISING:
		smbchg_aicl_enable(true);
		schedule_delayed_work(&chg->typec_disable_cmd_work,
				      msecs_to_jiffies(500));
		break;
	case OPLUS_USB_PLUGIN_FALLING:
		oplus_usb_plugin_removed(chg, true);
		rc = oplus_apply_sink_policy(chg);
		if (rc < 0)
			dev_err(chg->dev,
				"Couldn't apply Type-C role policy on USB removal rc=%d\n",
				rc);
		break;
	case OPLUS_USB_PLUGIN_POST:
		oplus_usb_plugin_post(chg, vbus_rising);
		rc = oplus_apply_sink_policy(chg);
		if (rc < 0)
			dev_err(chg->dev,
				"Couldn't finalize Type-C role policy on USB event rc=%d\n",
				rc);
		break;
	}
}

/*
 * sdm670R.c:3936-4015 (5 injections): hard-reset vooc/chg_monitor handling.
 */
static void oplus_usb_plugin_hard_reset(struct smb_charger *chg,
					bool vbus_rising,
					unsigned int phase)
{
	int rc;

	if (phase == OPLUS_USB_PLUGIN_FALLING && !vbus_rising)
		oplus_usb_plugin_removed(chg, false);
	else if (phase == OPLUS_USB_PLUGIN_POST) {
		oplus_usb_plugin_post(chg, vbus_rising);
		rc = oplus_apply_sink_policy(chg);
		if (rc < 0)
			dev_err(chg->dev,
				"Couldn't apply Type-C role policy during PD hard reset rc=%d\n",
				rc);
	}
}

/* sdm670R.c:4597-4605: ignore source changes for dual-cell sink sessions. */
static int oplus_usb_source_change_guard(struct smb_charger *chg)
{
	if (g_oplus_chip &&
	    chg->typec_mode == POWER_SUPPLY_TYPEC_SINK &&
	    g_oplus_chip->vbatt_num == 2) {
		pr_info("%s: typec_mode is sink, ignoring source change\n",
			__func__);
		return 1;
	}

	return 0;
}

/* sdm670R.c:4594-4650: APSD rerun condition and update-work wakeup. */
static int oplus_usb_source_change_rerun(struct smb_charger *chg)
{
	u8 result;
	int rc;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &result);
	if (rc < 0)
		return rc;

	return !!(result & (CDP_CHARGER_BIT | SDP_CHARGER_BIT));
}

static void oplus_usb_source_change(struct smb_charger *chg, bool apsd_done)
{
	if (apsd_done)
		oplus_chg_wake_update_work();
}

/* ===================== register init (D-class) ===================== */

/*
 * sdm670R.c:9556-9774 (8 injections in smb2_init_hw): BAT_UVLO, FG iterm,
 * AICL rerun, 0x1380/0x1365/0x1363/0x1052/0x1670 etc.
 * Called after qcom's smb2_init_hw so these writes override the defaults.
 */
static int oplus_post_smb2_init_hw(struct smb_charger *chg)
{
	int rc;

	/* BAT_UVLO (sdm670R.c:9557) */
	smblib_masked_write(chg, BAT_UVLO_THRESHOLD_CFG_REG,
			    BAT_UVLO_THRESHOLD_MASK, 0x3);

	/* AICL configuration: enable rerun (sdm670R.c:9601-9607) */
	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
		 SUSPEND_ON_COLLAPSE_USBIN_BIT |
				 USBIN_AICL_START_AT_MAX_BIT |
				 USBIN_AICL_ADC_EN_BIT |
				 USBIN_AICL_RERUN_EN_BIT,
		 USBIN_AICL_RERUN_EN_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure AICL rc=%d\n", rc);
		return rc;
	}

	/* override HVDCP enable vote (sdm670R.c:9665-9666) */
	vote(chg->hvdcp_enable_votable, MICRO_USB_VOTER, false, 0);

	/* disable FG default iterm (sdm670R.c:9719-9730) */
	rc = smblib_masked_write(chg, FG_UPDATE_CFG_2_SEL_REG,
				 IBT_LT_CHG_TERM_THRESH_SEL_BIT, 1);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't disable FG iterm override rc=%d\n",
			rc);

	rc = smblib_masked_write(chg, CHGR_CFG2_REG, I_TERM_BIT, 1);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't disable PM660 iterm override rc=%d\n",
			rc);

	/* custom register tweaks (sdm670R.c:9756-9770) */
	smblib_masked_write(chg, 0x1380, 0x03, 0x3);
	smblib_masked_write(chg, 0x1365, 0x03, 0x3);
	smblib_masked_write(chg, 0x1363, 0x20, 0);
	smblib_masked_write(chg, 0x1052, 0x02, 0);
	smblib_masked_write(chg, 0x1053, 0x40, 0);
	smblib_masked_write(chg, 0x1670, 0xff, 0);

	/* reset input-current flag (sdm670R.c:9773) */
	fg_oplus_set_input_current = false;

	return 0;
}

/*
 * Initialize the OPLUS monitor/divider/typec work items after smblib_init.
 */
static void oplus_post_smblib_init(struct smb_charger *chg)
{
	INIT_DELAYED_WORK(&chg->chg_monitor_work, oplus_chg_monitor_work);
	INIT_DELAYED_WORK(&chg->typec_disable_cmd_work,
			  typec_disable_cmd_work);
	INIT_DELAYED_WORK(&chg->divider_set_work, oplus_divider_set_work);
	chg->oplus_works_initialized = true;
}

static void oplus_remove(struct smb_charger *chg)
{
	if (chg->oplus_works_initialized) {
		cancel_delayed_work_sync(&chg->chg_monitor_work);
		cancel_delayed_work_sync(&chg->typec_disable_cmd_work);
		cancel_delayed_work_sync(&chg->divider_set_work);
		chg->oplus_works_initialized = false;
	}

	oplus_usbtemp_thread_deinit();
	remove_proc_subtree("d_reg_mask", NULL);
	if (g_oplus_chip && g_oplus_chip->pmic_spmi.chg == chg)
		oplus_chg_deinit(g_oplus_chip);

	if (g_oplus_chip && g_oplus_chip->pmic_spmi.chg == chg)
		g_oplus_chip = NULL;
	chg->oplus_hook = NULL;
}

/* ===================== shutdown / pm ===================== */

/*
 * sdm670R.c:12085-12128 (2 VENDOR_EDIT blocks): oplus shutdown sequence.
 * Block 1 (vooc reset) runs before qcom register writes and block 2
 * (shipmode) runs afterwards, selected by @shipmode_phase.
 */
static void oplus_shutdown(struct smb_charger *chg, bool shipmode_phase)
{
	if (!shipmode_phase && g_oplus_chip) {
		oplus_vooc_reset_mcu();
		smbchg_set_chargerid_switch_val(0);
		oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
		msleep(30);
	}

	if (shipmode_phase && g_oplus_chip && g_oplus_chip->enable_shipmode) {
		msleep(1000);
		smbchg_enter_shipmode(g_oplus_chip);
	}
}

/* sdm670R.c:11463-11507: usb_temp_adc pinctrl restore, RTC sleep, soc update. */
static int oplus_pm_resume(struct smb_charger *chg)
{
	struct oplus_chg_chip *oc = g_oplus_chip;
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc;

	if (!oc)
		return 0;

	if (oplus_usbtemp_check_is_support() == true) {
		if (IS_ERR_OR_NULL(oc->normalchg_gpio.usb_temp_adc)) {
			chg_err("get usb_temp_adc fail\n");
			return -EINVAL;
		} else {
			pinctrl_select_state(oc->normalchg_gpio.pinctrl,
					     oc->normalchg_gpio.usb_temp_adc);
		}
		if (get_project() == 19691 || get_project() == 19651) {
			if (IS_ERR_OR_NULL(oc->normalchg_gpio.usb_temp_adc_12)) {
				chg_err("get usb_temp_adc_12 fail\n");
				return -EINVAL;
			} else {
				pinctrl_select_state(
					oc->normalchg_gpio.pinctrl,
					oc->normalchg_gpio.usb_temp_adc_12);
			}
		}
	}

	rc = get_current_time(&resume_tm_sec);
	if (rc || oplus_suspend_tm_sec == (unsigned long)-1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else if (resume_tm_sec >= oplus_suspend_tm_sec) {
		sleep_time = resume_tm_sec - oplus_suspend_tm_sec;
	} else {
		sleep_time = 0;
	}

	oplus_chg_soc_update_when_resume(sleep_time);

	return 0;
}

/* sdm670R.c:11509-11539: usb_temp_adc suspend pinctrl, record suspend time. */
static int oplus_pm_suspend(struct smb_charger *chg)
{
	struct oplus_chg_chip *oc = g_oplus_chip;

	if (!oc)
		return 0;

	if (oplus_usbtemp_check_is_support() == true) {
		if (IS_ERR_OR_NULL(
			    oc->normalchg_gpio.usb_temp_adc_suspend)) {
			chg_err("get usb_temp_adc_suspend fail\n");
			return -EINVAL;
		} else {
			pinctrl_select_state(
				oc->normalchg_gpio.pinctrl,
				oc->normalchg_gpio.usb_temp_adc_suspend);
		}
		if (get_project() == 19691 || get_project() == 19651) {
			if (IS_ERR_OR_NULL(
				    oc->normalchg_gpio
					    .usb_temp_adc_suspend_12)) {
				chg_err("get usb_temp_adc_suspend_12 fail\n");
				return -EINVAL;
			} else {
				pinctrl_select_state(
					oc->normalchg_gpio.pinctrl,
					oc->normalchg_gpio
						.usb_temp_adc_suspend_12);
			}
		}
	}

	if (get_current_time(&oplus_suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		oplus_suspend_tm_sec = -1;
	}

	return 0;
}

/* ===================== smblib runtime VENDOR_EDIT fixups ===================== */

/*
 * sdm670R.c:4548: vote 500mA on usb_icl when fg_oplus_set_input_current
 * == false.
 */
static void oplus_apsd_done(struct smb_charger *chg)
{
	if (fg_oplus_set_input_current == false)
		vote(chg->usb_icl_votable, USB_PSY_VOTER, true, 500000);
}

/* sdm670R.c:5058: smbchg_aicl_enable(true) on non-sink typec insertion. */
static void oplus_typec_insertion(struct smb_charger *chg)
{
	smbchg_aicl_enable(true);
}

/*
 * sdm670R.c:540: HVDCP2 result mapping: only remap when result_bit ==
 * (DCP_CHARGER_BIT | QC_2P0_BIT). Return non-zero to suppress the remap.
 */
static int oplus_apsd_suppress_hvdcp2_remap(struct smb_charger *chg,
					    u8 result_bit)
{
	if (result_bit != (DCP_CHARGER_BIT | QC_2P0_BIT))
		return 1;
	return 0;
}

/* sdm670R.c:655: usb_status global forces suspend. Return non-zero to force. */
static int oplus_set_usb_suspend_force(struct smb_charger *chg, bool suspend)
{
	if (oplus_get_usb_status() ||
	    (g_oplus_chip && READ_ONCE(g_oplus_chip->input_suspend)))
		return 1;
	return 0;
}

/* sdm670R.c:841: mirror apsd pst into chg->usb_psy_desc.type. */
static void oplus_update_usb_type(struct smb_charger *chg, int charger_type)
{
	chg->usb_psy_desc.type = charger_type;
}

/*
 * sdm670R.c:1038: skip APSD rerun for types other than UNKNOWN/USB/CDP.
 * Return non-zero to return 0 early.
 */
static int oplus_rerun_apsd_suppress(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0)
		return 0;

	stat &= APSD_RESULT_STATUS_MASK;

	if (stat != 0 && stat != SDP_CHARGER_BIT && stat != CDP_CHARGER_BIT)
		return 1;

	return 0;
}

/*
 * sdm670R.c:1143: collapse SDP icl_options: <=150mA -> 0, else
 * USB51_MODE_BIT.
 */
static void oplus_sdp_current_options(struct smb_charger *chg, int icl_ua,
				     u8 *icl_options)
{
	if (icl_ua <= 150000)
		*icl_options = 0;
	else
		*icl_options = USB51_MODE_BIT;
}

/*
 * sdm670R.c:1218: non-SDP set_icl path uses 500000 for the SDP preset
 * instead of 100000.
 */
static void oplus_icl_hc_sdp_current(struct smb_charger *chg, int *sdp_ua)
{
	*sdp_ua = 500000;
}

/*
 * sdm670R.c:1544,1556: force HVDCP disabled: hvdcp_enable=0, val=0.
 * Return non-zero if oplus handled (caller clears hvdcp_enable and sets val=0).
 */
static int oplus_hvdcp_enable_fixup(struct smb_charger *chg, int hvdcp_enable,
				   u8 *val)
{
	*val = 0;
	return 1;
}

/*
 * sdm670R.c:2377: report 50% soc placeholder when bms_psy is NULL instead
 * of -EINVAL. Return non-zero if handled.
 */
static int oplus_bms_missing_default(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	if (!chg->bms_psy) {
		val->intval = 50;
		return 1;
	}
	return 0;
}

/*
 * sdm670R.c:2783: derive usb online from USBIN rt-status when the
 * usb_online_status flag is set. Return non-zero if handled.
 */
static int oplus_usb_online_status(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	u8 stat;
	int rc;

	if (usb_online_status == true) {
		rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
		if (rc < 0) {
			dev_err(chg->dev,
				"usb_online_status: Couldn't read USBIN_RT_STS rc=%d\n",
				rc);
			return rc;
		}
		val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
		return 1;
	}
	return 0;
}

static int oplus_smb_usb_get_property(struct smb_charger *chg, int psp,
				      union power_supply_propval *val)
{
	bool input_present;

	if (!g_oplus_chip)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if ((!oplus_vooc_get_fastchg_started() &&
		     !oplus_vooc_get_fastchg_ing()) ||
		    READ_ONCE(g_oplus_chip->input_suspend) ||
		    chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN)
			return -EOPNOTSUPP;
		if (oplus_get_usb_input_present(chg, &input_present) < 0 ||
		    !input_present)
			return -EOPNOTSUPP;
		/*
		 * Classic VOOC on OP46B1 is a 5 V / 4 A source.  The PMIC input
		 * path is suspended while VOOC is active, so its settled
		 * current cannot describe the adapter capability to Android.
		 */
		val->intval = OPLUS_VOOC_ADAPTER_CURRENT_MAX_UA;
		return 0;
	case POWER_SUPPLY_PROP_FAST_CHG_TYPE:
		if ((oplus_vooc_get_fastchg_started() ||
		     oplus_vooc_get_fastchg_dummy_started()) &&
		    g_oplus_chip->vbatt_num == 1 &&
		    oplus_vooc_get_fast_chg_type() ==
				FASTCHG_CHARGER_TYPE_UNKOWN)
			val->intval = CHARGER_SUBTYPE_FASTCHG_VOOC;
		else
			val->intval = oplus_vooc_get_fast_chg_type();
		return 0;
	case POWER_SUPPLY_PROP_USB_STATUS:
		val->intval = oplus_get_usb_status();
		return 0;
	case POWER_SUPPLY_PROP_USBTEMP_VOLT_L:
		val->intval = g_oplus_chip->usbtemp_volt_l;
		return 0;
	case POWER_SUPPLY_PROP_USBTEMP_VOLT_R:
		val->intval = g_oplus_chip->usbtemp_volt_r;
		return 0;
	default:
		return -EINVAL;
	}
}

static int oplus_usb_use_present_status(struct smb_charger *chg)
{
	/*
	 * VOOC bypasses/suspends the normal PMIC charging path.  Its ONLINE
	 * result therefore drops even though VBUS remains physically present.
	 * Keep the standard USB supply online so QTI/Lineage Health continues
	 * to report the charger throughout the VOOC session.
	 */
	return READ_ONCE(g_oplus_chip->input_suspend) ||
	       oplus_get_use_present_status() ||
	       oplus_vooc_get_fastchg_started() ||
	       oplus_vooc_get_fastchg_ing();
}

/*
 * A PD source that advertises USB Communications Capable must keep D+/D-
 * on the AP/PHY. If the charger framework already classified it as DCP and
 * started the VOOC probe, undo that probe before USB enumeration begins.
 */
static void oplus_pd_sdp_changed(struct smb_charger *chg, bool pd_sdp)
{
	struct oplus_chg_chip *oc = READ_ONCE(g_oplus_chip);

	if (!oc || !pd_sdp || oc->pmic_spmi.chg != chg)
		return;
	if (oc->charger_type != POWER_SUPPLY_TYPE_USB_DCP ||
	    chg->real_charger_type != POWER_SUPPLY_TYPE_USB_PD)
		return;

	chg_err("PD source has USB comm, restore D+/D- to AP\n");
	oplus_chg_set_chargerid_switch_val(0);
	oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
	oplus_vooc_reset_mcu();
	oplus_chg_clear_chargerid_info();
	oc->charger_type = POWER_SUPPLY_TYPE_USB;
	oc->real_charger_type = POWER_SUPPLY_TYPE_USB_PD;
	oplus_smb_update_otg_policy();
	oplus_chg_wake_update_work();
	if (chg->usb_psy)
		power_supply_changed(chg->usb_psy);
}

/* sdm670R.c:5266: guard power_supply_changed when dc_psy is NULL.
 * Return non-zero to skip the call. */
static int oplus_dc_plugin_guard(struct smb_charger *chg)
{
	if (!chg->dc_psy)
		return 1;
	return 0;
}

/*
 * sdm670R.c:5333: replace weak-charger/boost-back storm block with a
 * conditional 0mA vote for non-CDP/non-USB types. Return non-zero if handled.
 */
static int oplus_switcher_power_ok_storm(struct smb_charger *chg,
					struct smb_irq_data *irq_data)
{
	if (printk_ratelimit())
		dev_err(chg->dev,
			"Reverse boost detected: voting 0mA to suspend input\n");
	if (chg->real_charger_type != POWER_SUPPLY_TYPE_USB_CDP &&
	    chg->real_charger_type != POWER_SUPPLY_TYPE_USB)
		vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
	return 1;
}

/* ===================== the table ===================== */

const struct oplus_smb_hook oplus_smb_hooks = {
	.probe_early			= oplus_probe_early,
	.probe_attach			= oplus_probe_attach,
	.probe_psy_init			= oplus_probe_psy_init,
	.probe_chg_init			= oplus_probe_chg_init,
	.probe_finalize			= oplus_probe_finalize,
	.batt_health			= oplus_batt_health,

	.usb_plugin_locked		= oplus_usb_plugin_locked,
	.usb_plugin_hard_reset		= oplus_usb_plugin_hard_reset,
	.usb_source_change		= oplus_usb_source_change,
	.usb_source_change_rerun	= oplus_usb_source_change_rerun,
	.usb_source_change_guard	= oplus_usb_source_change_guard,

	.post_smb2_init_hw		= oplus_post_smb2_init_hw,
	.post_smblib_init		= oplus_post_smblib_init,

	.shutdown			= oplus_shutdown,
	.pm_resume			= oplus_pm_resume,
	.pm_suspend			= oplus_pm_suspend,
	.remove				= oplus_remove,

	.apsd_done			= oplus_apsd_done,
	.typec_insertion			= oplus_typec_insertion,
	.apsd_suppress_hvdcp2_remap	= oplus_apsd_suppress_hvdcp2_remap,
	.set_usb_suspend_force		= oplus_set_usb_suspend_force,
	.update_usb_type			= oplus_update_usb_type,
	.rerun_apsd_suppress		= oplus_rerun_apsd_suppress,
	.sdp_current_options		= oplus_sdp_current_options,
	.icl_hc_sdp_current		= oplus_icl_hc_sdp_current,
	.hvdcp_enable_fixup		= oplus_hvdcp_enable_fixup,
	.bms_missing_default		= oplus_bms_missing_default,
	.usb_online_status		= oplus_usb_online_status,
	.usb_get_property		= oplus_smb_usb_get_property,
	.usb_use_present_status		= oplus_usb_use_present_status,
	.pd_sdp_changed			= oplus_pd_sdp_changed,
	.force_typec_sink		= oplus_force_typec_sink,
	.dc_plugin_guard			= oplus_dc_plugin_guard,
	.switcher_power_ok_storm	= oplus_switcher_power_ok_storm,
};
