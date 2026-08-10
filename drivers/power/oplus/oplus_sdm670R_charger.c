/* SPDX-License-Identifier: GPL-2.0 */
/*
 * OPLUS platform business for sdm670R (OP46B1 / SDM710 + PMI632).
 *
 * Mechanically extracted from the merged sdm670R.c (qcom smb-lib + qpnp-smb2
 * + oplus business). This file holds only the OPLUS business helpers
 * charger-IC ops, vooc/adapter/usbtemp/ship/shortc/divider glue,
 * the smb2_chg_ops instance, oplus-modified smb2_batt_get_prop and AC/batt
 * power_supply descriptors, oplus power_supply_init, DT parsing, and the
 * dev_pm_ops handlers. The QCOM smblib and smb2 originals live in qcom/smb-lib.c
 * and qcom/qpnp-smb2.c.
 *
 * VENDOR_EDIT is #defined below so the oplus code paths are active (in the
 * original oplus build this flag is supplied globally). It does not affect the
 * qcom units, which contain no VENDOR_EDIT sections.
 *
 * The original source was a merged QCOM/OPLUS translation unit. Interfaces
 * used across the split are explicitly declared by smb-lib.h and the OPLUS
 * platform headers.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/qpnp/qpnp-revid.h>

#include <soc/oplus/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/oplus_project.h>

#include "../supply/qcom/smb-lib.h"
#include "../supply/qcom/smb-reg.h"

#include "oplus_chg_sdm670R.h"
#include "oplus_smb_hook.h"

#include "oplus_charger.h"
#include "oplus_gauge.h"
#include "oplus_vooc.h"
#include "oplus_short.h"
#include "oplus_adapter.h"
#include "oplus_short_ic.h"
#include "oplus_bq27541.h"
#include "op_charge.h"


/* ===== globals and forward declarations ===== */
struct oplus_chg_chip *g_oplus_chip = NULL;
struct smb_charger *g_smb_chip = NULL;
void smbchg_set_chargerid_switch_val(int value);
void oplus_wake_up_usbtemp_thread(void);
void smbchg_aicl_enable(bool enable);
int qpnp_get_prop_charger_voltage_now(void);
bool oplus_usbtemp_check_is_support(void);
int oplus_tbatt_power_off_task_init(struct oplus_chg_chip *chip);
static int smbchg_charging_disble(void);
bool oplus_chg_is_usb_present(void);
#define OPLUS_CHG_MONITOR_INTERVAL round_jiffies_relative(msecs_to_jiffies(5000))
#define OPLUS_DIVIDER_WORK_MODE_AUTO			1
#define OPLUS_DIVIDER_WORK_MODE_FIXED		0
#define USBIN_25MA	25000

static struct task_struct *oplus_usbtemp_kthread;
DECLARE_WAIT_QUEUE_HEAD(oplus_usbtemp_wq);
bool fg_oplus_set_input_current = false;
int usb_status = 0;
bool usb_online_status = false;
static bool use_present_status;

bool oplus_get_use_present_status(void)
{
	return use_present_status;
}

int __attribute__((weak)) oplus_set_divider_work_mode(int work_mode)
{
    return 0;
}
void __attribute__((weak)) switch_usb_state(int usb_state) {return;}

/* ===== oplus divider work handler ===== */
static bool divider_in_auto_mode = true;
void oplus_divider_set_work(struct work_struct *work)
{
	int rc;
	u8 stat;
	bool vbus_rising;
	struct smb_charger *chg = container_of(work, struct smb_charger,
								divider_set_work.work);
	if (g_oplus_chip && g_oplus_chip->vbatt_num != 2) {
		return;
	}
	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	dev_dbg(chg->dev, "oplus_divider_set_work vbus_rising=%d\n",
		vbus_rising);
	if (vbus_rising) {
		rc = oplus_set_divider_work_mode(OPLUS_DIVIDER_WORK_MODE_FIXED);
		divider_in_auto_mode = false;
	} else {
		rc = oplus_set_divider_work_mode(OPLUS_DIVIDER_WORK_MODE_AUTO);
		if (rc == 0) {
			divider_in_auto_mode = true;
		}
	}

}
/* ===== typec disable command work ===== */
void typec_disable_cmd_work(struct work_struct *work)
{
	int rc = 0;
	struct smb_charger *chg = container_of(work, struct smb_charger, typec_disable_cmd_work.work);

	if (smblib_get_prop_typec_mode(chg) != POWER_SUPPLY_TYPEC_NONE) {
		printk(KERN_ERR "!!! %s: active t-c module\n", __func__);
		return;
	}

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, TYPEC_DISABLE_CMD_BIT, TYPEC_DISABLE_CMD_BIT);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't write TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG rc=%d\n", rc);

	msleep(100);

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, TYPEC_DISABLE_CMD_BIT, 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't write TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG rc=%d\n", rc);

	printk(KERN_ERR "!!! %s: re-active t-c module\n", __func__);

	msleep(200);
	if (smblib_get_prop_typec_mode(chg) == POWER_SUPPLY_TYPEC_NONE) {
		printk(KERN_ERR "!!! %s: fake typec plug\n", __func__);
		rc = smblib_masked_write(chg, TYPE_C_CFG_REG, APSD_START_ON_CC_BIT, 0);
		if (rc < 0)
			dev_err(chg->dev, "Couldn't enable APSD_START_ON_CC rc=%d\n", rc);
		msleep(600);
		chg->fake_typec_insertion = true;
		chg->typec_mode = POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
		power_supply_changed(chg->usb_psy);
	}

	/* The FSM restart must not override the effective OPLUS power role. */
	rc = oplus_smb_update_otg_policy();
	if (rc < 0 && rc != -ENODEV)
		dev_err(chg->dev, "Couldn't restore OTG policy rc=%d\n", rc);

	return;
}

/* ===== oplus charge monitor (fv/vbatt-full/monitor work) ===== */
static int oplus_chg_get_fv_monitor(struct oplus_chg_chip *chip)
{
	int default_fv = 0;

	if (!chip)
		return 0;

	default_fv = chip->limits.temp_cold_vfloat_mv;

	switch(chip->tbatt_status) {
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
		case BATTERY_STATUS__HIGH_TEMP:
			break;
		case BATTERY_STATUS__COLD_TEMP:
			default_fv = chip->limits.temp_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
			default_fv = chip->limits.temp_little_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__COOL_TEMP:
			default_fv = chip->limits.temp_cool_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			default_fv = chip->limits.temp_little_cool_vfloat_mv;
			break;
		case BATTERY_STATUS__NORMAL:
			default_fv = chip->limits.temp_normal_vfloat_mv;
			break;
		case BATTERY_STATUS__WARM_TEMP:
			default_fv = chip->limits.temp_warm_vfloat_mv;
			break;
		default:
			break;
	}
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	if (oplus_short_c_batt_is_prohibit_chg(chip) && default_fv > chip->limits.short_c_bat_vfloat_mv)
		default_fv = chip->limits.short_c_bat_vfloat_mv;
#endif
	return default_fv;
}

static int oplus_chg_get_vbatt_full_vol_sw(struct oplus_chg_chip *chip)
{
	int default_fv = 0;

	if (!chip)
		return 0;

	default_fv = chip->limits.cold_vfloat_sw_limit;

	switch(chip->tbatt_status) {
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
		case BATTERY_STATUS__HIGH_TEMP:
			break;
		case BATTERY_STATUS__COLD_TEMP:
			default_fv = chip->limits.cold_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
			default_fv = chip->limits.little_cold_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__COOL_TEMP:
			default_fv = chip->limits.cool_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			default_fv = chip->limits.little_cool_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__NORMAL:
			default_fv = chip->limits.temp_normal_vfloat_mv;
			break;
		case BATTERY_STATUS__WARM_TEMP:
			default_fv = chip->limits.warm_vfloat_sw_limit;
			break;
		default:
			break;
	}
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	if (oplus_short_c_batt_is_prohibit_chg(chip) && default_fv > chip->limits.short_c_bat_vfloat_sw_limit)
		default_fv = chip->limits.short_c_bat_vfloat_sw_limit;
#endif
	return default_fv;
}

/* When charger voltage is setting to < 4.3V and then resume to 5V, cannot charge, so... */
void oplus_chg_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							chg_monitor_work.work);
	struct oplus_chg_chip *chip = g_oplus_chip;
	int boot_mode = get_boot_mode();
	static int counts = 0;
	int rechg_vol;
	int rc;
	u8 stat;

	if (!chip || !chip->charger_exist || !chip->batt_exist || !chip->mmi_chg){
		counts = 0;
		goto rerun_work;
	}
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB || chg->real_charger_type == POWER_SUPPLY_TYPE_USB_CDP)
		return;
	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN)
		return;
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	if (oplus_short_c_batt_is_disable_rechg(chip)){
		counts = 0;
		goto rerun_work;
	}
#endif
	if (oplus_vooc_get_fastchg_started() == true || chip->charger_volt < 4400){
		counts = 0;
		goto rerun_work;
	}
	if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP)
		rechg_vol = oplus_chg_get_fv_monitor(chip) - 300;
	else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP)
		rechg_vol = oplus_chg_get_fv_monitor(chip) - 200;
	else
		rechg_vol = oplus_chg_get_fv_monitor(chip) - 100;
	if ((chip->batt_volt > rechg_vol - 10) && chip->batt_full){
		counts = 0;
		goto rerun_work;
	}else if (chip->batt_volt > oplus_chg_get_vbatt_full_vol_sw(chip) - 10){
		counts = 0;
		goto rerun_work;
	}

	if (chip->icharging >= 0) {
		counts++;
	} else if (chip->icharging < 0 && (chip->icharging * -1) <= chip->limits.iterm_ma / 2) {
		counts++;
	} else {
		counts = 0;
	}
	if (counts > 10)
		counts = 10;

	if (counts >= (chip->batt_full ? 8 : 3)) {//because rechg counts=6
		rc = smblib_read(chg, BATTERY_CHARGER_STATUS_8_REG, &stat);
		if (rc < 0) {
			printk(KERN_ERR "oplus_chg_monitor_work: Couldn't get BATTERY_CHARGER_STATUS_8_REG status rc=%d\n", rc);
			goto rerun_work;
		}
		if (get_client_vote(chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
				&& get_effective_result(chg->usb_icl_votable) <= USBIN_25MA) {
			printk(KERN_ERR "oplus_chg_monitor_work: boost back\n");
			if (chg->wa_flags & BOOST_BACK_WA)
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
		}
		if (chip->charging_state == CHARGING_STATUS_FAIL) {//for TEMP > 55 or < -3
			counts = 0;
			goto rerun_work;
		}
		if (stat & PRE_TERM_BIT) {
			usb_online_status = true;
			printk(KERN_ERR "oplus_chg_monitor_work: PRE_TERM_BIT is set[0x%x], clear it\n", stat);
			rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 1);
			if (rc < 0) {
				printk(KERN_ERR "oplus_chg_monitor_work: Couldn't set USBIN_SUSPEND_BIT rc=%d\n", rc);
				goto rerun_work;
			}
			msleep(50);
			if (!usb_status && !READ_ONCE(chip->input_suspend))
				rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0);
			if (rc < 0) {
				printk(KERN_ERR "oplus_chg_monitor_work: Couldn't clear USBIN_SUSPEND_BIT rc=%d\n", rc);
				goto rerun_work;
			}
			msleep(10);
			rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, RESTART_AICL_BIT, RESTART_AICL_BIT);
			if (rc < 0) {
				printk(KERN_ERR "oplus_chg_monitor_work: Couldn't set RESTART_AICL_BIT rc=%d\n", rc);
				goto rerun_work;
			}
			printk(KERN_ERR "oplus_chg_monitor_work: ichg[%d], fv[%d]\n", chip->icharging, oplus_chg_get_fv_monitor(chip));
		}
		counts = 0;
	}

rerun_work:
	usb_online_status = false;
	schedule_delayed_work(&chg->chg_monitor_work, OPLUS_CHG_MONITOR_INTERVAL);
}

/* ===== oplus 2uart / chargerid / ship / shortc / usbtemp gpio+init ===== */
int oplus_chg_2uart_pinctrl_init(struct oplus_chg_chip *chip)
{
	struct smb_charger *chg = NULL;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return -EINVAL;
	}

	if (chip->vbatt_num != 2) {
		return 0;
	}

	chg = chip->pmic_spmi.chg;

	chg->chg_2uart_pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chg->chg_2uart_pinctrl)) {
		chg_err("get 2uart chg_2uart_pinctrl fail\n");
		return -EINVAL;
	}

	chg->chg_2uart_default = pinctrl_lookup_state(chg->chg_2uart_pinctrl, "chg_qupv3_se12_2uart_default");
	if (IS_ERR_OR_NULL(chg->chg_2uart_default)) {
		chg_err("get chg_2uart_default fail\n");
		return -EINVAL;
	}

	chg->chg_2uart_sleep = pinctrl_lookup_state(chg->chg_2uart_pinctrl, "chg_qupv3_se12_2uart_sleep");
	if (IS_ERR_OR_NULL(chg->chg_2uart_sleep)) {
		chg_err("get chg_2uart_sleep fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chg->chg_2uart_pinctrl, chg->chg_2uart_default);

	return 0;
}
 static int oplus_chg_set_2uart_pinctrl_chgID(struct oplus_chg_chip *chip)
{
	struct smb_charger *chg = NULL;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return -EINVAL;
	}

	if (chip->vbatt_num != 2) {
		return 0;
	}
	chg = chip->pmic_spmi.chg;

	if (IS_ERR_OR_NULL(chg->chg_2uart_pinctrl) || IS_ERR_OR_NULL(chg->chg_2uart_sleep)) {
		chg_err("get 2uart chg_2uart_pinctrl fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chg->chg_2uart_pinctrl, chg->chg_2uart_sleep);
	return 0;
}

static int oplus_chg_set_2uart_pinctrl_default(struct oplus_chg_chip *chip)
{
	struct smb_charger *chg = NULL;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return -EINVAL;
	}

	if (chip->vbatt_num != 2) {
		return 0;
	}
	chg = chip->pmic_spmi.chg;

	if (IS_ERR_OR_NULL(chg->chg_2uart_pinctrl) || IS_ERR_OR_NULL(chg->chg_2uart_default)) {
		chg_err("get 2uart chg_2uart_pinctrl fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chg->chg_2uart_pinctrl, chg->chg_2uart_default);
	return 0;
}
int smbchg_get_chargerid_volt(void)
{
	int rc = 0;
	int chargerid_volt = 0;
	struct qpnp_vadc_result results;
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct smb_charger *chg = chip->pmic_spmi.chg;

	if (!chip->pmic_spmi.pm660_vadc_dev) {
		chg_err("pm660_vadc_dev NULL\n");
		return 0;
	}
	if (chip->vbatt_num == 2) {
		oplus_chg_set_2uart_pinctrl_chgID(chip);
		msleep(10);
	}
	if (chg->charger_id_num == 7) {
		rc = qpnp_vadc_read(chip->pmic_spmi.pm660_vadc_dev, P_MUX10_1_1, &results);
	} else {
		rc = qpnp_vadc_read(chip->pmic_spmi.pm660_vadc_dev, P_MUX3_1_1, &results);
	}
	if (rc) {
		chg_err("unable to read pm660_vadc_dev charger_id rc = %d\n", rc);
		return 0;
	}
	chargerid_volt = (int)results.physical / 1000;
	chg_err("chargerid_volt: %d\n", chargerid_volt);
	if (chip->vbatt_num == 2) {
		oplus_chg_set_2uart_pinctrl_default(chip);
	}
	return chargerid_volt;
}


static int smbchg_chargerid_switch_gpio_init(struct oplus_chg_chip *chip)
{
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_active =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)) {
		chg_err("get chargerid_switch_active fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_sleep =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)) {
		chg_err("get chargerid_switch_sleep fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_default =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_default");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_default)) {
		chg_err("get chargerid_switch_default fail\n");
		return -EINVAL;
	}

	if (chip->normalchg_gpio.chargerid_switch_gpio > 0) {
		gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
	}
	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.chargerid_switch_default);

	return 0;
}

void smbchg_set_chargerid_switch_val(int value)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (oplus_vooc_get_adapter_update_real_status() == ADAPTER_FW_NEED_UPDATE
		|| oplus_vooc_get_btb_temp_over() == true) {
		chg_err("adapter update or btb_temp_over, return\n");
		return;
	}
#if 0
	if (chip->pmic_spmi.not_support_1200ma && !value && !is_usb_present(chip)) {
	/* BugID 879716 : Solve some situatuion ChargerID is not 0 mV when usb is not present */
		chip->chargerid_volt = 0;
		chip->chargerid_volt_got = false;
	}
#endif
	if (value) {
		gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 1);
		pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.chargerid_switch_default);
	} else {
		gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
		pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.chargerid_switch_default);
	}
	chg_err("set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio));
}

int smbchg_get_chargerid_switch_val(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio);
}

static int oplus_ship_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	chip->normalchg_gpio.ship_active =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "ship_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active)) {
		chg_err("get ship_active fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.ship_sleep =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "ship_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
		chg_err("get ship_sleep fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_sleep);

	return 0;
}

static bool oplus_ship_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.ship_gpio))
		return true;

	return false;
}

#define PWM_COUNT	5
void smbchg_enter_shipmode(struct oplus_chg_chip *chip)
{
	int i = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return;
	}

	if (oplus_ship_check_is_gpio(chip) == true) {
		chg_debug("select gpio control\n");
		if (!IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
			pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.ship_sleep);
		}
		for (i = 0; i < PWM_COUNT; i++) {
			//gpio_direction_output(chip->normalchg_gpio.ship_gpio, 1);
			pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_active);
			mdelay(3);
			//gpio_direction_output(chip->normalchg_gpio.ship_gpio, 0);
			pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_sleep);
			mdelay(3);
		}
		chg_debug("power off after 15s\n");
	}
}

static int oplus_shortc_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	chip->normalchg_gpio.shortc_active =
		pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "shortc_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.shortc_active)) {
		chg_err("get shortc_active fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.shortc_active);

	/*chg->fcc_stepper_enable = of_property_read_bool(node,
					"qcom,fcc-stepping-enable");*/

	return 0;
}

static bool oplus_shortc_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.shortc_gpio))
		return true;

	return false;
}

#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
static bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;
	struct oplus_chg_chip *chip = g_oplus_chip;
	return true;
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return shortc_hw_status;
	}

	if (oplus_shortc_check_is_gpio(chip) == true) {
		shortc_hw_status = !!(gpio_get_value(chip->normalchg_gpio.shortc_gpio));
	}
	return shortc_hw_status;
}
#else
static bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;

	return shortc_hw_status;
}
#endif /* CONFIG_OPLUS_SHORT_HW_CHECK */
static bool oplus_usbtemp_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.dischg_gpio))
		return true;

	return false;
}

static int oplus_usbtemp_init(struct oplus_chg_chip *chip)
{
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.pinctrl fail\n");
		return -EINVAL;
	}
	chip->normalchg_gpio.usb_temp_adc_suspend =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "usb_temp_adc_suspend");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.usb_temp_adc_suspend)) {
		chg_err("get usb_temp_adc_suspend fail\n");
		return -EINVAL;
	}
	chip->normalchg_gpio.usb_temp_adc =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "usb_temp_adc");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.usb_temp_adc)) {
		chg_err("get usb_temp_adc fail\n");
		return -EINVAL;
	} else {
        pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.usb_temp_adc);
	}

	if(get_project() == 19691 || get_project() == 19651) {
		chip->normalchg_gpio.usb_temp_adc_suspend_12 =
				pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "usb_temp_adc_suspend_12");
	   if (IS_ERR_OR_NULL(chip->normalchg_gpio.usb_temp_adc_suspend_12)) {
		   chg_err("get usb_temp_adc_suspend_12 fail\n");
		   return -EINVAL;
	   }
	   chip->normalchg_gpio.usb_temp_adc_12 =
			   pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "usb_temp_adc_12");
	   if (IS_ERR_OR_NULL(chip->normalchg_gpio.usb_temp_adc_12)) {
		   chg_err("get usb_temp_adc_12 fail\n");
		   return -EINVAL;
	   } else {
		   pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.usb_temp_adc_12);
	   }
	}
	return 0;
}

bool oplus_usbtemp_check_is_support(void)
{
	if(oplus_usbtemp_check_is_gpio(g_oplus_chip) == true)
		return true;

	chg_err("dischg return false\n");

	return false;
}

static int oplus_get_usbtemp_volt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	int usbtemp_volt = 0;
	struct qpnp_vadc_result results;

	if (!chip->pmic_spmi.pm660_usbtemp_vadc_dev) {
		chg_err("usbtemp_vadc_dev NULL\n");
		return 0;
	}

	if(get_project() == 19651) {
		/* just for 19651 gpio,other project need change channal  */
		rc = qpnp_vadc_read(chip->pmic_spmi.pm660_usbtemp_vadc_dev, P_MUX4_1_1, &results);
		if (rc) {
			chg_err("unable to read usbtemp_vadc_dev gpio03 rc = %d\n", rc);
			return 0;
		}
		chip->usbtemp_volt_r = (int)results.physical / 1000;
		rc = qpnp_vadc_read(chip->pmic_spmi.pm660_usbtemp_vadc_dev, P_MUX10_1_1, &results);
		if (rc) {
			chg_err("unable to read usbtemp_vadc_dev gpio12 rc = %d\n", rc);
		}
		if (!chip->usbtemp_volt_l)
			chip->usbtemp_volt_l = chip->usbtemp_volt_r;
		else
			chip->usbtemp_volt_l = (int)results.physical / 1000;

		return chip->usbtemp_volt_r < chip->usbtemp_volt_l ? chip->usbtemp_volt_r : chip->usbtemp_volt_l;
	} else if (get_project() == 19691) {
		/* just for 19691 gpio,other project need change channal  */
		rc = qpnp_vadc_read(chip->pmic_spmi.pm660_usbtemp_vadc_dev, P_MUX4_1_1, &results);
		if (rc) {
			chg_err("unable to read usbtemp_vadc_dev gpio03 rc = %d\n", rc);
			return 0;
		}
		chip->usbtemp_volt_r = (int)results.physical / 1000;
		rc = qpnp_vadc_read(chip->pmic_spmi.pm660_usbtemp_vadc_dev, P_MUX5_1_1, &results);
		if (rc) {
			chg_err("unable to read usbtemp_vadc_dev gpio12 rc = %d\n", rc);
		}

		/*if(pcb_version >= 1)*/
			chip->usbtemp_volt_l = (int)results.physical / 1000;/*for evt and later*/
		/*else */
		/*	chip->usbtemp_volt_l = chip->usbtemp_volt_r;for T0*/
		return chip->usbtemp_volt_r < chip->usbtemp_volt_l ? chip->usbtemp_volt_r : chip->usbtemp_volt_l;
	} else {
		rc = qpnp_vadc_read(chip->pmic_spmi.pm660_usbtemp_vadc_dev, P_MUX5_1_1, &results);
		if (rc) {
			chg_err("unable to read usbtemp_vadc_dev VADC_AMUX1_GPIO_PU2 rc = %d\n", rc);
			return 0;
		}

		usbtemp_volt = (int)results.physical / 1000;
		chip->usbtemp_volt_l = usbtemp_volt;
		chip->usbtemp_volt_r = usbtemp_volt;

		return usbtemp_volt;
	}
	return usbtemp_volt;
}

static int oplus_dischg_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("chip NULL\n");
		return EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get dischg_pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.dischg_enable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_enable");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
		chg_err("get dischg_enable fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.dischg_disable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_disable");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_disable)) {
		chg_err("get dischg_disable fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);

	return 0;
}

#define USB_40C	40
#define USB_50C	50
#define USB_55C	53
#define USB_57C	57
#define USB_100C	100
#define VBUS_VOLT_THRESHOLD	400
#define USB_VBUS_SHORT_DISABLE_VOLT		509
#define USB_VBUS_SHORT_ENABLE_VOLT		392
#define MIN_MONITOR_INTERVAL	50 /*50ms*/
#define MAX_MONITOR_INTERVAL	200 /*200ms*/
#define RETRY_CNT_DELAY         5 /*ms*/
#define VBUS_MONITOR_INTERVAL	3000 /*3s*/
#define HIGH_TEMP_SHORT_CHECK_TIMEOUT 1000 /*ms*/
static void get_usb_temp(struct oplus_chg_chip *chip)
{
	int i = 0;

	for (i = ARRAY_SIZE(con_volt_18097) - 1; i >= 0; i--) {
		if (con_volt_18097[i] >= chip->usbtemp_volt_r)
			break;
		else if (i == 0)
			break;
	}
	chip->usb_temp_r = con_temp_18097[i]+1;

	for (i = ARRAY_SIZE(con_volt_18097) - 1; i >= 0; i--) {
		if (con_volt_18097[i] >= chip->usbtemp_volt_l)
			break;
		else if (i == 0)
			break;
	}
	if(!chip->usb_temp_l)
		chip->usb_temp_l = chip->usb_temp_r; /*for T0*/
	else
		chip->usb_temp_l = con_temp_18097[i]+1; /*for evt and later*/
	/*WR for temperature value filter, use last value*/

	/*chg_err("usbtemp: %d, %d\n", chip->usb_temp_r, chip->usb_temp_l);*/
}

#define USB_TEMP_HIGH	0x01 /*bit0*/
#define USB_RESERVE1	0x02 /*bit1*/
#define USB_RESERVE2	0x04 /*bit2*/
#define USB_RESERVE3	0x08 /*bit3*/
#define USB_RESERVE4	0x10 /*bit4*/
#define USB_DONOT_USE	0x80000000 /*bit31*/

static void oplus_set_usb_status(int status)
{
	usb_status = usb_status | status;
}


int oplus_get_usb_status(void)
{
	return usb_status;
}
static int oplus_usbtemp_monitor_main(void *data)
{
	int delay = 0;
	static bool dischg_flag = false;
	struct smb_charger *chg = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	static int count = 0;
	static int total_count = 0;
	static int last_usb_temp_r = 25;
	static int current_temp_r = 25;
	static int last_usb_temp_l = 25;
	static int current_temp_l = 25;
	int vbus_volt = 0, usbtemp_print = 0;
	int retry_cnt = 3, i = 0;
	int count_r = 1, count_l = 1;
	#ifndef CONFIG_HIGH_TEMP_VERSION
	union power_supply_propval pval;
	#endif
	chg = chip->pmic_spmi.chg;
	#ifdef CONFIG_HIGH_TEMP_VERSION
	dischg_flag = true;
	pr_err("%s:dischg_flag = true; \n", __func__);
	#endif

	while (!kthread_should_stop() && dischg_flag == false) {
		oplus_get_usbtemp_volt(chip);
		get_usb_temp(chip);
		vbus_volt = qpnp_get_prop_charger_voltage_now();

		/*chg_err("dischg vbus_volt...[%d]\n", vbus_volt);*/
		if ((chip->usb_temp_r < USB_40C)&&(chip->usb_temp_l < USB_40C)) {
			delay = MAX_MONITOR_INTERVAL;
	total_count = 10;
	} else {
		delay = MIN_MONITOR_INTERVAL;
		total_count = 30;
	}
		if (chip->usb_temp_r > chip->usb_temp_l)
			chip->usb_temp_l = chip->usb_temp_r;
		if (usbtemp_print != chip->usb_temp_l)
			chg_err("dischg usbtemp...[%d]\n", usbtemp_print);
		usbtemp_print = chip->usb_temp_l;
		if (((chip->usb_temp_r < USB_50C)&&(chip->usb_temp_l < USB_50C)) && vbus_volt < VBUS_VOLT_THRESHOLD) {
			delay = VBUS_MONITOR_INTERVAL;
		}
		if ((USB_57C <= chip->usb_temp_r && chip->usb_temp_r < USB_100C)
				|| (USB_57C <= chip->usb_temp_l && chip->usb_temp_l < USB_100C)) {
			if (dischg_flag == false && vbus_volt > 3000) {
				for (i = 1; i < retry_cnt; i++) {
					mdelay(RETRY_CNT_DELAY);
					oplus_get_usbtemp_volt(chip);
					get_usb_temp(chip);
					if (chip->usb_temp_r >= USB_57C)
						count_r++;
					if (chip->usb_temp_l >= USB_57C)
						count_l++;
				}

				if (count_r >= retry_cnt || count_l >= retry_cnt) {
					if (!IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
						dischg_flag = true;
						chg_err("dischg enable1...[%d,%d]\n", chip->usb_temp_r, chip->usb_temp_l);
						oplus_set_usb_status(USB_TEMP_HIGH);
						#ifndef CONFIG_HIGH_TEMP_VERSION
						/*chip->chg_ops->charger_suspend();*/
						if (oplus_vooc_get_fastchg_started() == true) {
							oplus_chg_set_chargerid_switch_val(0);
							oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
							oplus_vooc_reset_mcu();
						} else if (chg->real_charger_type) {
							smbchg_charging_disble();
						}
						usleep_range(10000, 10000); /*wait for turn-off fastchg MOS*/
						/*chip->chg_ops->charger_suspend();*/
						pval.intval = 1;
						power_supply_set_property(chg->batt_psy, POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
						usleep_range(20000, 20000); /*wait for turn-off fastchg MOS*/
						#endif
						#ifdef CONFIG_HIGH_TEMP_VERSION
						chg_err(" CONFIG_HIGH_TEMP_VERSION enable here,do not set vbus down \n");
						pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);
						#else
						chg_err(" CONFIG_HIGH_TEMP_VERSION disabled \n");
						pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_enable);
						#endif
					}
				}
				count_r = 1;
				count_l = 1;
			}
			count = 0;
			msleep(delay);
		} else if ((((chip->usb_temp_r - chip->temperature/10) > 10) && (chip->usb_temp_r < USB_100C)) ||
					(((chip->usb_temp_l - chip->temperature/10) > 10) && (chip->usb_temp_l < USB_100C))) {
			if (dischg_flag == false && vbus_volt > 3000) {
				if (count <= total_count) {
					if (count == 0) {
						last_usb_temp_r = chip->usb_temp_r;
						last_usb_temp_l = chip->usb_temp_l;
					} else {
						current_temp_r = chip->usb_temp_r;
						current_temp_l = chip->usb_temp_l;
					}
					if ((current_temp_r - last_usb_temp_r) >= 3 || (current_temp_l - last_usb_temp_l) >= 3) {
						for (i = 1; i < retry_cnt; i++) {
							mdelay(RETRY_CNT_DELAY);
							oplus_get_usbtemp_volt(chip);
							get_usb_temp(chip);
							if ((chip->usb_temp_r - last_usb_temp_r) >= 3)
								count_r++;
							if ((chip->usb_temp_l - last_usb_temp_l) >= 3)
								count_l++;
						}
						current_temp_r = chip->usb_temp_r;
						current_temp_l = chip->usb_temp_l;
						if (count_r >= retry_cnt || count_l >= retry_cnt) {
							if (!IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
								dischg_flag = true;
								chg_err("dischg enable3...,current_temp=[%d,%d],last_usb_temp=[%d,%d],last_usb_tempcount =%d\n",
									current_temp_r, current_temp_l, last_usb_temp_r, last_usb_temp_l, count);
								oplus_set_usb_status(USB_TEMP_HIGH);
								#ifndef CONFIG_HIGH_TEMP_VERSION
								/*chip->chg_ops->charger_suspend();*/
								if (oplus_vooc_get_fastchg_started() == true) {
									oplus_chg_set_chargerid_switch_val(0);
									oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
									oplus_vooc_reset_mcu();
								} else if (chg->real_charger_type) {
									smbchg_charging_disble();
								}
								usleep_range(10000, 10000);/*wait for turn-off fastchg MOS*/
								/*chip->chg_ops->charger_suspend();*/
								pval.intval = 1;
								power_supply_set_property(chg->batt_psy, POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
								usleep_range(20000, 20000);/*wait for turn-off fastchg MOS*/
								#endif

								chg_err("dischg enable3...,current_temp_l=%d,last_usb_temp_l=%d,current_temp_r=%d,last_usb_temp_r =%d\n",
									current_temp_l, last_usb_temp_l, current_temp_r, last_usb_temp_r);
								#ifdef CONFIG_HIGH_TEMP_VERSION
								chg_err(" CONFIG_HIGH_TEMP_VERSION enable here,do not set vbus down \n");
								pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);
								#else
								chg_err(" CONFIG_HIGH_TEMP_VERSION disabled \n");
								pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_enable);
								#endif
							}
						}
						count_r = 1;
						count_l = 1;
					}
					count++;
					if (count > total_count) {
						count = 0;
					}
				}
			}
			msleep(delay);
		} else {
			count = 0;
			msleep(delay);
			wait_event_interruptible(oplus_usbtemp_wq,
				qpnp_get_prop_charger_voltage_now() > 3000 ||
				kthread_should_stop());
			if (kthread_should_stop())
				break;
		}
	}

	return 0;
}

int oplus_usbtemp_thread_init(void)
{
	int rc;

	if (oplus_usbtemp_kthread)
		return 0;

	oplus_usbtemp_kthread =
			kthread_run(oplus_usbtemp_monitor_main, 0, "usbtemp_kthread");
	if (IS_ERR(oplus_usbtemp_kthread)) {
		rc = PTR_ERR(oplus_usbtemp_kthread);
		oplus_usbtemp_kthread = NULL;
		chg_err("failed to create oplus_usbtemp_kthread: %d\n", rc);
		return rc;
	}
	return 0;
}

void oplus_usbtemp_thread_deinit(void)
{
	if (!oplus_usbtemp_kthread)
		return;

	wake_up_interruptible(&oplus_usbtemp_wq);
	kthread_stop(oplus_usbtemp_kthread);
	oplus_usbtemp_kthread = NULL;
}

void oplus_wake_up_usbtemp_thread(void)
{
	if (oplus_usbtemp_check_is_support() == true){
		wake_up_interruptible(&oplus_usbtemp_wq);
	}
}

/* ===== oplus custom DT parsing ===== */
int oplus_chg_parse_custom_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;
	if (!node) {
			pr_err("device tree node missing\n");
			return -EINVAL;
	}

	if (g_oplus_chip) {
		g_oplus_chip->normalchg_gpio.chargerid_switch_gpio =
				of_get_named_gpio(node, "qcom,chargerid_switch-gpio", 0);
		if (g_oplus_chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
			chg_err("Couldn't read chargerid_switch-gpio rc = %d, chargerid_switch_gpio:%d\n",
					rc, g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
		} else {
			if (gpio_is_valid(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio)) {
				rc = devm_gpio_request(chip->dev,
					g_oplus_chip->normalchg_gpio.chargerid_switch_gpio,
					"charging-switch1-gpio");
				if (rc) {
					chg_err("unable to request chargerid_switch_gpio:%d\n", g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
				} else {
					smbchg_chargerid_switch_gpio_init(g_oplus_chip);
				}
			}
			chg_err("chargerid_switch_gpio:%d\n", g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
		}
	}
	if (g_oplus_chip) {
		g_oplus_chip->normalchg_gpio.dischg_gpio = of_get_named_gpio(node, "qcom,dischg-gpio", 0);
		if (g_oplus_chip->normalchg_gpio.dischg_gpio <= 0) {
			chg_err("Couldn't read qcom,dischg-gpio rc=%d, qcom,dischg-gpio:%d\n",
				rc, g_oplus_chip->normalchg_gpio.dischg_gpio);
		} else {
			if (oplus_usbtemp_check_is_support() == true) {
				if (gpio_is_valid(g_oplus_chip->normalchg_gpio.dischg_gpio)) {
					rc = devm_gpio_request(chip->dev,
						g_oplus_chip->normalchg_gpio.dischg_gpio,
						"dischg-gpio");
					if (rc) {
						chg_err("unable to request dischg-gpio:%d\n", g_oplus_chip->normalchg_gpio.dischg_gpio);
					} else {
						oplus_dischg_gpio_init(g_oplus_chip);
					}
				}
			}
			chg_err("dischg-gpio:%d\n", g_oplus_chip->normalchg_gpio.dischg_gpio);
		}

		if(get_project() == 19691 || get_project() == 19651) {
			oplus_usbtemp_init(g_oplus_chip);
		}
	}

	if (g_oplus_chip) {
		g_oplus_chip->normalchg_gpio.ship_gpio =
				of_get_named_gpio(node, "qcom,ship-gpio", 0);
		if (g_oplus_chip->normalchg_gpio.ship_gpio <= 0) {
			chg_err("Couldn't read qcom,ship-gpio rc = %d, qcom,ship-gpio:%d\n",
					rc, g_oplus_chip->normalchg_gpio.ship_gpio);
		} else {
			if (oplus_ship_check_is_gpio(g_oplus_chip) == true) {
				rc = devm_gpio_request(chip->dev,
					g_oplus_chip->normalchg_gpio.ship_gpio,
					"ship-gpio");
				if (rc) {
					chg_err("unable to request ship-gpio:%d\n",
							g_oplus_chip->normalchg_gpio.ship_gpio);
				} else {
					oplus_ship_gpio_init(g_oplus_chip);
					if (rc)
						chg_err("unable to init ship-gpio:%d\n", g_oplus_chip->normalchg_gpio.ship_gpio);
				}
			}
			chg_err("ship-gpio:%d\n", g_oplus_chip->normalchg_gpio.ship_gpio);
		}
	}

	if (g_oplus_chip) {
		g_oplus_chip->normalchg_gpio.shortc_gpio =
				of_get_named_gpio(node, "qcom,shortc-gpio", 0);
		if (g_oplus_chip->normalchg_gpio.shortc_gpio <= 0) {
			chg_err("Couldn't read qcom,shortc-gpio rc = %d, qcom,shortc-gpio:%d\n",
					rc, g_oplus_chip->normalchg_gpio.shortc_gpio);
		} else {
			if (oplus_shortc_check_is_gpio(g_oplus_chip) == true) {
				rc = devm_gpio_request(chip->dev,
					g_oplus_chip->normalchg_gpio.shortc_gpio,
					"shortc-gpio");
				if (rc) {
					chg_err("unable to request shortc-gpio:%d\n",
							g_oplus_chip->normalchg_gpio.shortc_gpio);
				} else {
					oplus_shortc_gpio_init(g_oplus_chip);
					if (rc)
						chg_err("unable to init ship-gpio:%d\n", g_oplus_chip->normalchg_gpio.ship_gpio);
				}
			}
			chg_err("shortc-gpio:%d\n", g_oplus_chip->normalchg_gpio.shortc_gpio);
		}
	}
	return rc;

}

/* ===== oplus AC power supply (ac_props / smb2_ac_get_property / ac_psy_desc / smb2_init_ac_psy) ===== */
/*************************
 * AC PSY REGISTRATION *
 *************************/
static enum power_supply_property ac_props[] = {
 /*oplus own ac props*/
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb2_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int rc = 0;

    rc = oplus_ac_get_property(psy, psp, val);

	return rc;
}

static const struct power_supply_desc ac_psy_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ac_props,
	.num_properties = ARRAY_SIZE(ac_props),
	.get_property = smb2_ac_get_property,
};

static int smb2_init_ac_psy(struct oplus_chg_chip *chip)
{
	struct power_supply_config ac_cfg = {};
	struct smb_charger *chg = chip->pmic_spmi.chg;

	ac_cfg.drv_data = chip;
	ac_cfg.of_node = chg->dev->of_node;
	chg->ac_psy = devm_power_supply_register(chg->dev,
						  &ac_psy_desc,
						  &ac_cfg);
	if (IS_ERR(chg->ac_psy)) {
		pr_err("Couldn't register AC power supply\n");
		return PTR_ERR(chg->ac_psy);
	}

	return 0;
}

/* ===== oplus-modified battery power supply cluster =====
 * (smb2_batt_props / smb2_batt_get_prop / set_prop / prop_is_writeable / batt_psy_desc / smb2_init_batt_psy)
 */
static enum power_supply_property smb2_batt_props[] = {
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_QNOVO,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_QNOVO,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SW_JEITA_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_PARALLEL_DISABLE,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_DIE_HEALTH,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	/* oplus own battery props */
        POWER_SUPPLY_PROP_VOLTAGE_MIN,
        POWER_SUPPLY_PROP_CHARGE_NOW,
        POWER_SUPPLY_PROP_AUTHENTICATE,
        POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
        POWER_SUPPLY_PROP_CHARGE_TECHNOLOGY,
        POWER_SUPPLY_PROP_FAST_CHARGE,
        POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE,
        POWER_SUPPLY_PROP_BATTERY_FCC,
        POWER_SUPPLY_PROP_BATTERY_SOH,
        POWER_SUPPLY_PROP_BATTERY_CC,
        POWER_SUPPLY_PROP_BATTERY_RM,
        POWER_SUPPLY_PROP_BATTERY_NOTIFY_CODE,
        POWER_SUPPLY_PROP_ADAPTER_FW_UPDATE,
        POWER_SUPPLY_PROP_VOOCCHG_ING,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
	POWER_SUPPLY_PROP_CHIP_SOC,
#endif
	POWER_SUPPLY_PROP_CYCLE_COUNT,
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
        POWER_SUPPLY_PROP_CHARGERID_VOLT,
#endif
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
        POWER_SUPPLY_PROP_SHIP_MODE,
#endif
	POWER_SUPPLY_PROP_FCC_STEPPER_ENABLE,
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
        POWER_SUPPLY_PROP_SHORT_C_LIMIT_CHG,
        POWER_SUPPLY_PROP_SHORT_C_LIMIT_RECHG,
        POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
        POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
#else
        POWER_SUPPLY_PROP_SHORT_C_BATT_UPDATE_CHANGE,
        POWER_SUPPLY_PROP_SHORT_C_BATT_IN_IDLE,
        POWER_SUPPLY_PROP_SHORT_C_BATT_CV_STATUS,
#endif//CONFIG_OPLUS_SHORT_USERSPACE
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
        POWER_SUPPLY_PROP_SHORT_C_HW_FEATURE,
        POWER_SUPPLY_PROP_SHORT_C_HW_STATUS,
#endif
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
        POWER_SUPPLY_PROP_SHORT_C_IC_OTP_STATUS,
        POWER_SUPPLY_PROP_SHORT_C_IC_VOLT_THRESH,
        POWER_SUPPLY_PROP_SHORT_C_IC_OTP_VALUE,
#endif
        POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_COOL_DOWN,
};

static int smb2_batt_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct smb_charger *chg = power_supply_get_drvdata(psy);
	union power_supply_propval usb_present = {0, };
	int rc = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/*
		 * Keep the native QCOM state machine for normal charging and
		 * discharging. VOOC bypasses the PMIC charger after
		 * negotiation, so only an active session needs the OPLUS status
		 * override.
		 */
		if (oplus_vooc_get_fastchg_started() && g_oplus_chip &&
		    chg->real_charger_type != POWER_SUPPLY_TYPE_UNKNOWN &&
		    !READ_ONCE(g_oplus_chip->input_suspend) &&
		    !smblib_get_prop_usb_present(chg, &usb_present) &&
		    usb_present.intval)
			rc = oplus_battery_get_property(psy, psp, val);
		else
			rc = smblib_get_prop_batt_status(chg, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		rc = smblib_get_prop_input_suspend(chg, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rc = smblib_get_prop_batt_charge_type(chg, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		rc = smblib_get_prop_system_temp_level(chg, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		rc = smblib_get_prop_system_temp_level_max(chg, val);
		break;
        case POWER_SUPPLY_PROP_CHARGER_TEMP:
        case POWER_SUPPLY_PROP_CHARGER_TEMP_MAX:
            val->intval = -1;
            break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		rc = smblib_get_prop_input_current_limited(chg, val);
		break;
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
		val->intval = chg->step_chg_enabled;
		break;
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
		val->intval = chg->sw_jeita_enabled;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = get_client_vote(chg->fv_votable,
				BATT_PROFILE_VOTER);
		break;
	case POWER_SUPPLY_PROP_CHARGE_QNOVO_ENABLE:
		rc = smblib_get_prop_charge_qnovo_enable(chg, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_QNOVO:
		val->intval = get_client_vote_locked(chg->fv_votable,
				QNOVO_VOTER);
		break;
	case POWER_SUPPLY_PROP_CURRENT_QNOVO:
		val->intval = get_client_vote_locked(chg->fcc_votable,
				QNOVO_VOTER);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = get_client_vote(chg->fcc_votable,
					      BATT_PROFILE_VOTER);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = get_client_vote(chg->fcc_votable,
					      FG_ESR_VOTER);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		rc = smblib_get_prop_batt_charge_done(chg, val);
		break;
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
		val->intval = get_client_vote(chg->pl_disable_votable,
					      USER_VOTER);
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		/* Not in ship mode as long as device is active */
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_DIE_HEALTH:
		if (chg->die_health == -EINVAL)
			rc = smblib_get_prop_die_health(chg, val);
		else
			val->intval = chg->die_health;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chg->pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		/*
		 * The external BQ27411 already supplies remaining capacity in
		 * mAh.  Report that value instead of estimating it from the
		 * smoothed UI SOC and design capacity.
		 */
		val->intval = g_oplus_chip ? g_oplus_chip->batt_rm * 1000 : 0;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		/* Unsupported external gauges report zero here. */
		val->intval = g_oplus_chip ? g_oplus_chip->batt_cc : 0;
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smblib_get_prop_input_current_settled(chg, val);
		break;

	case POWER_SUPPLY_PROP_FCC_STEPPER_ENABLE:
		val->intval = chg->fcc_stepper_enable;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		if (g_oplus_chip && (g_oplus_chip->ui_soc == 0)) {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
			chg_err("bat pro POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL, should shutdown!!!\n");
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (g_oplus_chip) {
			val->intval = g_oplus_chip->batt_capacity_mah * 1000;
		}
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = 0;
		break;
	default:
//#ifdef VENDOR_EDIT
         /*oplus own battery props*/
		rc = oplus_battery_get_property(psy, psp, val);
//#endif
	}

	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}

	return 0;
}

static int smb2_batt_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int rc = 0;
	struct smb_charger *chg = power_supply_get_drvdata(psy);
	bool old_input_suspend;
	bool suspend;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		rc = smblib_set_prop_batt_status(chg, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		if (!g_oplus_chip) {
			rc = -ENODEV;
			break;
		}
		suspend = !!val->intval;
		old_input_suspend = READ_ONCE(g_oplus_chip->input_suspend);
		rc = oplus_chg_set_input_suspend(g_oplus_chip, suspend);
		if (rc < 0)
			break;
		rc = smblib_set_prop_input_suspend(chg, val);
		if (rc < 0) {
			oplus_chg_set_input_suspend(g_oplus_chip,
						    old_input_suspend);
			if (!old_input_suspend)
				oplus_chg_wake_update_work();
			break;
		}
		if (!suspend)
			oplus_chg_wake_update_work();
		oplus_smb_notify_usb_changed();
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		rc = smblib_set_prop_system_temp_level(chg, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = smblib_set_prop_batt_capacity(chg, val);
		break;
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
		vote(chg->pl_disable_votable, USER_VOTER, (bool)val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		chg->batt_profile_fv_uv = val->intval;
		vote(chg->fv_votable, BATT_PROFILE_VOTER, true, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_QNOVO_ENABLE:
		rc = smblib_set_prop_charge_qnovo_enable(chg, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_QNOVO:
		vote(chg->fv_votable, QNOVO_VOTER,
			(val->intval >= 0), val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_QNOVO:
		vote(chg->pl_disable_votable, PL_QNOVO_VOTER,
			val->intval != -EINVAL && val->intval < 2000000, 0);
		if (val->intval == -EINVAL) {
			vote(chg->fcc_votable, BATT_PROFILE_VOTER,
					true, chg->batt_profile_fcc_ua);
			vote(chg->fcc_votable, QNOVO_VOTER, false, 0);
		} else {
			vote(chg->fcc_votable, QNOVO_VOTER, true, val->intval);
			vote(chg->fcc_votable, BATT_PROFILE_VOTER, false, 0);
		}
		break;
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
		chg->step_chg_enabled = !!val->intval;
		break;
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
		if (chg->sw_jeita_enabled != (!!val->intval)) {
			rc = smblib_disable_hw_jeita(chg, !!val->intval);
			if (rc == 0)
				chg->sw_jeita_enabled = !!val->intval;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		chg->batt_profile_fcc_ua = val->intval;
		vote(chg->fcc_votable, BATT_PROFILE_VOTER, true, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval)
			vote(chg->fcc_votable, FG_ESR_VOTER, true, val->intval);
		else
			vote(chg->fcc_votable, FG_ESR_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		/* Not in ship mode as long as the device is active */
		if (!val->intval)
			break;
		if (chg->pl.psy)
			power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_SET_SHIP_MODE, val);
		rc = smblib_set_prop_ship_mode(chg, val);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		rc = smblib_rerun_aicl(chg);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		rc = smblib_dp_dm(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		rc = smblib_set_prop_input_current_limited(chg, val);
		break;
	case POWER_SUPPLY_PROP_DIE_HEALTH:
		chg->die_health = val->intval;
		power_supply_changed(chg->batt_psy);
		break;
	default:
	/*oplus own battery props*/
		rc = oplus_battery_set_property(psy, prop, val);
	}

	return rc;
}

static int smb2_batt_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
        int rc = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SW_JEITA_ENABLED:
	case POWER_SUPPLY_PROP_DIE_HEALTH:
		return 1;
	default:
	/*oplus own battery props*/
            rc = oplus_battery_property_is_writeable(psy, psp);
		break;
	}

	return rc;
}

static const struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = smb2_batt_props,
	.num_properties = ARRAY_SIZE(smb2_batt_props),
	.get_property = smb2_batt_get_prop,
	.set_property = smb2_batt_set_prop,
	.property_is_writeable = smb2_batt_prop_is_writeable,
};

static int smb2_init_batt_psy(struct oplus_chg_chip *chip)
{
	struct power_supply_config batt_cfg = {};
	struct smb_charger *chg = chip->pmic_spmi.chg;
	int rc = 0;

	batt_cfg.drv_data = chg;
	batt_cfg.of_node = chg->dev->of_node;
	chg->batt_psy = power_supply_register(chg->dev,
						   &batt_psy_desc,
						   &batt_cfg);
	if (IS_ERR(chg->batt_psy)) {
		pr_err("Couldn't register battery power supply\n");
		return PTR_ERR(chg->batt_psy);
	}

	return rc;
}

/* ===== oplus power supply init (registers ac/batt/usb) ===== */
/*oplus own battery props*/
int oplus_power_supply_init(struct oplus_chg_chip *chip)
{
    int rc = 0;

    rc = smb2_init_ac_psy(chip);
    if (rc < 0) {
        pr_err("Couldn't initialize ac psy rc=%d\n", rc);
        return rc;
    }
//kong
    rc = smb2_init_batt_psy(chip);
    if (rc < 0) {
        pr_err("Couldn't initialize batt psy rc=%d\n", rc);
        return rc;
    }

    rc = smb2_init_usb_psy_for_oplus(chip->pmic_spmi.chg);
    if (rc < 0) {
        pr_err("Couldn't initialize usb psy rc=%d\n", rc);
        return rc;
    }

    return rc;
}

/* ===== proc dump_registers_mask ===== */
static bool d_reg_mask = false;
static ssize_t dump_registers_mask_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	char mask[16];

	if (count > 16) {
		return -EFAULT;
	}

	if (copy_from_user(&mask, buff, count)) {
		printk(KERN_ERR "dump_registers_mask_write error.\n");
		return -EFAULT;
	}

	if (strncmp(mask, "dump808", 7) == 0) {
		d_reg_mask = true;
		printk(KERN_ERR "dump registers mask enable.\n");
	} else {
		d_reg_mask = false;
		return -EFAULT;
	}

	return count;
}

static const struct file_operations dump_registers_mask_fops = {
	.write = dump_registers_mask_write,
	.llseek = noop_llseek,
};

void init_proc_dump_registers_mask(void)
{
	if (!proc_create("d_reg_mask", S_IWUSR | S_IWGRP | S_IWOTH, NULL, &dump_registers_mask_fops)) {
		printk(KERN_ERR "proc_create dump_registers_mask_fops fail\n");
	}
}

/* ===== charger-IC ops implementations + dump_regs + rtc + pm_ops + smb2_chg_ops instance =====
 * dump_regs, smbchg_*, oplus_chg_*, opchg_*, qpnp_get_*, oplus_chg_get_*,
 * get_current_time, smb2_pm_resume/suspend, smb2_pm_ops, smb2_chg_ops
 */
//static int get_boot_mode(void);
static int smbchg_usb_suspend_disable(void);
static int smbchg_usb_suspend_enable(void);
static int smbchg_charging_enble(void);

static void dump_regs(void)
{
	int i;
	int j;
	int rc;
	u8 stat;
	int base[] = {0x1000, 0x1100, 0x1200, 0x1300, 0x1400, 0x1600, 0x1800, 0x1900};
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip || !d_reg_mask)
		return;

	chg = g_oplus_chip->pmic_spmi.chg;
	if (!chg)
		return;

	pr_err("================= %s: begin ======================\n", __func__);

	for (j = 0; j < 8; j++) {
		for (i = 0; i < 255; i++) {
			rc = smblib_read(chg, base[j] + i, &stat);
			if (rc < 0) {
				pr_err("Couldn't read %x rc=%d\n", base[j] + i, rc);
			} else {
				pr_err("%x : %x\n", base[j] + i, stat);
			}
		}

		msleep(1000);
	}

	pr_err("================= %s: end ======================\n", __func__);

	d_reg_mask = false;
}

static int smbchg_kick_wdt(void)
{
	return 0;
}

static int oplus_chg_hw_init(void)
{
	int boot_mode = get_boot_mode();

	if (boot_mode != MSM_BOOT_MODE__RF && boot_mode != MSM_BOOT_MODE__WLAN) {
		smbchg_usb_suspend_disable();
	} else {
		smbchg_usb_suspend_enable();
	}
	smbchg_charging_enble();

	return 0;
}

static int smbchg_set_fastchg_current_raw(int current_ma)
{
	int rc = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	rc = vote(chip->pmic_spmi.chg->fcc_votable, DEFAULT_VOTER,
			true, current_ma * 1000);
	if (rc < 0)
		chg_err("Couldn't vote fcc_votable[%d], rc=%d\n", current_ma, rc);

	return rc;
}

static void smbchg_set_aicl_point(int vol)
{
	// do nothing
}

void smbchg_aicl_enable(bool enable)
{
	int rc = 0;
	u8 aicl_op;
	struct oplus_chg_chip *chip = g_oplus_chip;

	rc = smblib_masked_write(chip->pmic_spmi.chg, USBIN_AICL_OPTIONS_CFG_REG,
			USBIN_AICL_EN_BIT, enable ? USBIN_AICL_EN_BIT : 0);
	if (rc < 0)
		chg_err("Couldn't write USBIN_AICL_OPTIONS_CFG_REG rc=%d\n", rc);
	rc = smblib_read(chip->pmic_spmi.chg, 0x1380, &aicl_op);
	if (!rc)
		chg_err("AICL_OPTIONS 0x1380 = 0x%02x\n", aicl_op); //dump 0x1380
}
static void smbchg_usbin_collapse_irq_enable(bool enable)
{
	static bool collapse_en = true;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (enable && !collapse_en){
		enable_irq(chip->pmic_spmi.chg->irq_info[USBIN_COLLAPSE_IRQ].irq);
	}else if (!enable && collapse_en){
		disable_irq(chip->pmic_spmi.chg->irq_info[USBIN_COLLAPSE_IRQ].irq);
	}
	collapse_en = enable;
}
static void smbchg_rerun_aicl(void)
{
	smbchg_aicl_enable(false);
	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_aicl_enable(true);
}

static bool  oplus_chg_is_normal_mode(void)
{
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN)
		return false;
	return true;
}

static bool oplus_chg_is_suspend_status(void)
{
	int rc = 0;
	u8 stat;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip)
		return false;

	chg = g_oplus_chip->pmic_spmi.chg;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		printk(KERN_ERR "oplus_chg_is_suspend_status: Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return false;
	}

	return (bool)(stat & USBIN_SUSPEND_STS_BIT);
}

static void oplus_chg_clear_suspend(void)
{
	int rc;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip)
		return;

	chg = g_oplus_chip->pmic_spmi.chg;

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 1);
	if (rc < 0) {
		printk(KERN_ERR "oplus_chg_monitor_work: Couldn't set USBIN_SUSPEND_BIT rc=%d\n", rc);
	}
	msleep(50);
	if (!usb_status && !READ_ONCE(g_oplus_chip->input_suspend))
		rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0);
	if (rc < 0) {
		printk(KERN_ERR "oplus_chg_monitor_work: Couldn't clear USBIN_SUSPEND_BIT rc=%d\n", rc);
	}
}

static void oplus_chg_check_clear_suspend(void)
{
	use_present_status = true;
	oplus_chg_clear_suspend();
	use_present_status = false;
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1350, 1500, 1750, 2000, 3000,
};

static int oplus_chg_set_input_current(int current_ma)
{
	int rc = 0, i = 0, n = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	u8 stat = 0;
    int pre_current = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	}

	if (chip->pmic_spmi.chg->pre_current_ma == current_ma)
		return rc;
	else {
		pre_current = chip->pmic_spmi.chg->pre_current_ma;
		chip->pmic_spmi.chg->pre_current_ma = current_ma;
	}

	chg_debug( "usb input max current limit=%d setting %02x, pre_current[%d]\n", current_ma, i, pre_current);

	fg_oplus_set_input_current = true;

	if (chip->batt_volt > 4100 )
		aicl_point = 4550;
	else
		aicl_point = 4500;

	smbchg_aicl_enable(false);
    if (pre_current > current_ma){
		for (n = sizeof(usb_icl)/sizeof(int); n > 0; n--){
			if (pre_current > usb_icl[n]){
				break;
			}
		}
		chg_debug( "downTo: usb input max current limit=%d setting %d\n", current_ma, n);
		if (usb_icl[n] > 1200){
			rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[n] * 1000);
			msleep(90);
			n--;

			if (usb_icl[n] >= 1200){
				rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[n] * 1000);
				msleep(90);
				n--;

				if (usb_icl[n] >= 1200){
					rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[n] * 1000);
					msleep(90);
					n--;
				}
			}
		}
	}

	smbchg_usbin_collapse_irq_enable(false);
	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		chg_debug( "use 500 here\n");
		goto aicl_boost_back;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		chg_debug( "use 500 here\n");
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point) {
		chg_debug( "use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 1;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 1;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1350 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(130);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 2;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point) {
		i = i - 2;
		goto aicl_pre_step;
	}

	i = 5; /* 1500 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(120);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 3;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 3;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 3;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point) {
		i = i - 3; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 2; //We use 1.2A here
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 6; /* 1750 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(120);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 3;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 3;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 3;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point) {
		i = i - 3; //1.2
		goto aicl_pre_step;
	}

	i = 7; /* 2000 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 2;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point) {
		i =  i - 2;//1.5
		goto aicl_pre_step;
	} else if (current_ma < 3000)
		goto aicl_end;

	i = 8; /* 3000 */
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.chg->usb_icl_votable) <= USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 1;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, usb_icl[i], aicl_point);
	smbchg_rerun_aicl();
	smbchg_usbin_collapse_irq_enable(true);
	goto aicl_return;
aicl_end:
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, usb_icl[i], aicl_point);
	smbchg_rerun_aicl();
	smbchg_usbin_collapse_irq_enable(true);
	goto aicl_return;
aicl_boost_back:
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_boost_back\n", chg_vol, i, usb_icl[i], aicl_point);
	if (chip->pmic_spmi.chg->wa_flags & BOOST_BACK_WA)
		vote(chip->pmic_spmi.chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	smbchg_rerun_aicl();
	smbchg_usbin_collapse_irq_enable(true);
	goto aicl_return;
aicl_suspend:
	rc = vote(chip->pmic_spmi.chg->usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_suspend\n", chg_vol, i, usb_icl[i], aicl_point);
	oplus_chg_check_clear_suspend();
	smbchg_rerun_aicl();
	smbchg_usbin_collapse_irq_enable(true);
	goto aicl_return;
aicl_return:
	/*FORCE icl 500mA for AUDIO_ADAPTER combo cable*/
	if (chip->pmic_spmi.chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
		chg_debug( "AUDIO ADAPTER MODE\n");
		rc = smblib_read(chip->pmic_spmi.chg, USBIN_LOAD_CFG_REG, &stat);
		if (rc < 0) {
			chg_debug( "read USBIN_LOAD_CFG_REG, failed rc=%d\n", rc);
		}
		if ((bool)(stat& ICL_OVERRIDE_AFTER_APSD_BIT)) {
			rc = smblib_write(chip->pmic_spmi.chg, USBIN_CURRENT_LIMIT_CFG_REG, 0x14);
			if (rc < 0) {
				chg_debug( "Couldn't write USBIN_CURRENT_LIMIT_CFG_REG rc=%d\n", rc);
			} else {
				chg_debug( "FORCE icl 500\n");
			}
		}
	}
	return rc;
}

static int smbchg_float_voltage_set(int vfloat_mv)
{
	int rc = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	rc = vote(chip->pmic_spmi.chg->fv_votable, BATT_PROFILE_VOTER/*DEFAULT_VOTER*/,
			true, vfloat_mv * 1000);
	if (rc < 0)
		chg_err("Couldn't vote fv_votable[%d], rc=%d\n", vfloat_mv, rc);

	return rc;
}

static int smbchg_term_current_set(int term_current)
{
	int rc = 0;
	u8 val_raw = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (term_current < 0 || term_current > 750)
		term_current = 150;

	val_raw = term_current / 50;
	rc = smblib_masked_write(chip->pmic_spmi.chg, TCCC_CHARGE_CURRENT_TERMINATION_CFG_REG,
			TCCC_CHARGE_CURRENT_TERMINATION_SETTING_MASK, val_raw);
	if (rc < 0)
		chg_err("Couldn't write TCCC_CHARGE_CURRENT_TERMINATION_CFG_REG rc=%d\n", rc);

	return rc;
}

static int smbchg_charging_enble(void)
{
	int rc = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	rc = vote(chip->pmic_spmi.chg->chg_disable_votable, DEFAULT_VOTER,
			false, 0);
	if (rc < 0)
		chg_err("Couldn't enable charging, rc=%d\n", rc);

	return rc;
}

static int smbchg_charging_disble(void)
{
	int rc = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	rc = vote(chip->pmic_spmi.chg->chg_disable_votable, DEFAULT_VOTER,
			true, 0);
	if (rc < 0)
		chg_err("Couldn't disable charging, rc=%d\n", rc);

	chip->pmic_spmi.chg->pre_current_ma = -1;

	return rc;
}

static int smbchg_get_charge_enable(void)
{
	int rc = 0;
	u8 temp = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	rc = smblib_read(chip->pmic_spmi.chg, CHARGING_ENABLE_CMD_REG, &temp);
	if (rc < 0) {
		chg_err("Couldn't read CHARGING_ENABLE_CMD_REG rc=%d\n", rc);
		return 0;
	}
	rc = temp & CHARGING_ENABLE_CMD_BIT;

	return rc;
}

static int smbchg_usb_suspend_enable(void)
{
	int rc = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	rc = smblib_set_usb_suspend(chip->pmic_spmi.chg, true);
	if (rc < 0)
		chg_err("Couldn't write enable to USBIN_SUSPEND_BIT rc=%d\n", rc);

	chip->pmic_spmi.chg->pre_current_ma = -1;

	return rc;
}

static int smbchg_usb_suspend_disable(void)
{
	int rc = 0;
	int boot_mode = get_boot_mode();
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		chg_err("RF/WLAN, suspending...\n");
		rc = smblib_set_usb_suspend(chip->pmic_spmi.chg, true);
		if (rc < 0)
			chg_err("Couldn't write enable to USBIN_SUSPEND_BIT rc=%d\n", rc);
		return rc;
	}

	rc = smblib_set_usb_suspend(chip->pmic_spmi.chg, false);
	if (rc < 0)
		chg_err("Couldn't write disable to USBIN_SUSPEND_BIT rc=%d\n", rc);

	return rc;
}

static int smbchg_set_rechg_vol(int rechg_vol)
{
	return 0;
}

static int smbchg_reset_charger(void)
{
	return 0;
}

static int smbchg_read_full(void)
{
	int rc = 0;
	u8 stat = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!oplus_chg_is_usb_present())
		return 0;

	rc = smblib_read(chip->pmic_spmi.chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		chg_err("Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n", rc);
		return 0;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (stat == TERMINATE_CHARGE || stat == INHIBIT_CHARGE)
		return 1;
	return 0;
}

static int oplus_set_chging_term_disable(void)
{
	return 0;
}

static bool qcom_check_charger_resume(void)
{
	return true;
}

bool smbchg_need_to_check_ibatt(void)
{
	return false;
}

static int smbchg_get_chg_current_step(void)
{
	return 25;
}

int opchg_get_real_charger_type(void)
{
    struct smb_charger *chg = NULL;
    struct oplus_chg_chip *chip = g_oplus_chip;

    if (!chip) {
        return POWER_SUPPLY_TYPE_UNKNOWN;
    }

    chg = chip->pmic_spmi.chg;

    return chg->real_charger_type;
}

/*
 * A PD port is a USB SDP (data-capable host) when either the source
 * capability advertises USB Communications Capable, or BC1.2 still found
 * SDP before PD disabled APSD.  Such a port must never be handed to VOOC.
 */
static bool oplus_pd_sdp_detected(struct smb_charger *chg)
{
	u8 stat;

	if (chg->pd_sdp)
		return true;

	if (smblib_read(chg, APSD_RESULT_STATUS_REG, &stat) < 0)
		return false;

	return (stat & APSD_RESULT_STATUS_MASK) == SDP_CHARGER_BIT;
}

int opchg_get_charger_type(void)
{
	u8 apsd_stat;
	int rc;
	struct smb_charger *chg = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip)
		return POWER_SUPPLY_TYPE_UNKNOWN;

	chg = chip->pmic_spmi.chg;

	/* reset for fastchg to normal */
	if (chip->charger_type == POWER_SUPPLY_TYPE_UNKNOWN)
		chg->pre_current_ma = -1;

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		chg_err("Couldn't read APSD_STATUS rc=%d\n", rc);
		return POWER_SUPPLY_TYPE_UNKNOWN;
	}
	chg_debug("APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT)) {
		if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_PD &&
		    oplus_pd_sdp_detected(chg))
			return POWER_SUPPLY_TYPE_USB;
		return POWER_SUPPLY_TYPE_UNKNOWN;
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB
			|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_CDP
			|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
		oplus_chg_soc_update();
	}

	if (POWER_SUPPLY_TYPE_UNKNOWN == chg->real_charger_type) {
		smblib_update_usb_type(chg);
		chg_debug("update_usb_type: get_charger_type=%d\n", chg->real_charger_type);
	}

	if (POWER_SUPPLY_TYPE_USB_PD == chg->real_charger_type) {
		if (oplus_pd_sdp_detected(chg))
			return POWER_SUPPLY_TYPE_USB;
		return POWER_SUPPLY_TYPE_USB_DCP;
	}
	return chg->real_charger_type;
}


int qpnp_get_prop_charger_voltage_now(void)
{
	int val = 0;
	struct smb_charger *chg = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip)
		return 0;

	//if (!oplus_chg_is_usb_present())
	//	return 0;

	chg = chip->pmic_spmi.chg;
	if (!chg->iio.usbin_v_chan || PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan))
		return PTR_ERR(chg->iio.usbin_v_chan);

	iio_read_channel_processed(chg->iio.usbin_v_chan, &val);

	if (val < 2000 * 1000)
		chg->pre_current_ma = -1;

	return val / 1000;
}

static int oplus_chg_get_ibus(void)
{
	struct smb_charger *chg = NULL;
	union power_supply_propval val;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int rc;

	if (!chip) {
		chg_err("fail to init oplus_chip\n");
		return 0;
	}

	chg = chip->pmic_spmi.chg;

	rc = power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get INPUT_CURRENT_NOW, rc=%d\n", rc);
		return rc;
	}

	return val.intval;
}
bool oplus_chg_is_usb_present(void)
{
	int rc = 0;
	u8 stat = 0;
	bool vbus_rising = false;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip)
		return false;

	rc = smblib_read(chip->pmic_spmi.chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		chg_err("Couldn't read USB_INT_RT_STS, rc=%d\n", rc);
		return false;
	}
	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (vbus_rising == false && oplus_vooc_get_fastchg_started() == true) {
		if (qpnp_get_prop_charger_voltage_now() > 2000) {
			chg_err("USBIN_PLUGIN_RT_STS_BIT low but fastchg started true and chg vol > 2V\n");
			vbus_rising = true;
		}
	}
	if (vbus_rising == false && (oplus_vooc_get_fastchg_started() == true && (chip->vbatt_num == 2))) {
			chg_err("USBIN_PLUGIN_RT_STS_BIT low but fastchg started true and SVOOC\n");
			vbus_rising = true;
	}

	if (vbus_rising == false)
		chip->pmic_spmi.chg->pre_current_ma = -1;

	return vbus_rising;
}


int qpnp_get_battery_voltage(void)
{
	return 3800;//Not use anymore
}
#if 0
static int get_boot_mode(void)
{
	return 0;
}
#endif
int smbchg_get_boot_reason(void)
{
	return 0;
}

int oplus_chg_get_shutdown_soc(void)
{
	return 0;
}

int oplus_chg_backup_soc(int backup_soc)
{
	return 0;
}

static int smbchg_get_aicl_level_ma(void)
{
	return 0;
}

static int smbchg_force_tlim_en(bool enable)
{
	return 0;
}

static int smbchg_system_temp_level_set(int lvl_sel)
{
	return 0;
}

static int smbchg_set_prop_flash_active(enum skip_reason reason, bool disable)
{
	return 0;
}

static int smbchg_dp_dm(int val)
{
	return 0;
}

static int smbchg_calc_max_flash_current(void)
{
	return 0;
}

int oplus_chg_get_fv(struct oplus_chg_chip *chip)
{
	int flv = chip->limits.temp_normal_vfloat_mv;
	int batt_temp = chip->temperature;

	if (batt_temp > chip->limits.hot_bat_decidegc) {//53C
		//default
	} else if (batt_temp >= chip->limits.warm_bat_decidegc) {//45C
		flv = chip->limits.temp_warm_vfloat_mv;
	} else if (batt_temp >= chip->limits.normal_bat_decidegc) {//16C
		flv = chip->limits.temp_normal_vfloat_mv;
	} else if (batt_temp >= chip->limits.little_cool_bat_decidegc) {//12C
		flv = chip->limits.temp_little_cool_vfloat_mv;
	} else if (batt_temp >= chip->limits.cool_bat_decidegc) {//5C
		flv = chip->limits.temp_cool_vfloat_mv;
	} else if (batt_temp >= chip->limits.little_cold_bat_decidegc) {//0C
		flv = chip->limits.temp_little_cold_vfloat_mv;
	} else if (batt_temp >= chip->limits.cold_bat_decidegc) {//-3C
		flv = chip->limits.temp_cold_vfloat_mv;
	} else {
		//default
	}

	return flv;
}

int oplus_chg_get_charging_current(struct oplus_chg_chip *chip)
{
	int charging_current = 0;
	int batt_temp = chip->temperature;

	if (batt_temp > chip->limits.hot_bat_decidegc) {//53C
		charging_current = 0;
	} else if (batt_temp >= chip->limits.warm_bat_decidegc) {//45C
		charging_current = chip->limits.temp_warm_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.normal_bat_decidegc) {//16C
		charging_current = chip->limits.temp_normal_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.little_cool_bat_decidegc) {//12C
		charging_current = chip->limits.temp_little_cool_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.cool_bat_decidegc) {//5C
		if (chip->batt_volt > 4180)
			charging_current = chip->limits.temp_cool_fastchg_current_ma_low;
		else
			charging_current = chip->limits.temp_cool_fastchg_current_ma_high;
	} else if (batt_temp >= chip->limits.little_cold_bat_decidegc) {//0C
		charging_current = chip->limits.temp_little_cold_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.cold_bat_decidegc) {//-3C
		charging_current = chip->limits.temp_cold_fastchg_current_ma;
	} else {
		charging_current = 0;
	}

	return charging_current;
}

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPLUS_RTC_DET_SUPPORT */

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int oplus_chg_get_dyna_aicl_result(void)
{
	struct power_supply *usb_psy = NULL;
	union power_supply_propval pval = {0, };
	int result = 1000;

	usb_psy = power_supply_get_by_name("usb");
	if (usb_psy) {
		power_supply_get_property(usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
				&pval);
		result = pval.intval / 1000;
		power_supply_put(usb_psy);
	}

	return result;
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

struct oplus_chg_operations  smb2_chg_ops = {
	.dump_registers = dump_regs,
	.kick_wdt = smbchg_kick_wdt,
	.hardware_init = oplus_chg_hw_init,
	.charging_current_write_fast = smbchg_set_fastchg_current_raw,
	.set_aicl_point = smbchg_set_aicl_point,
	.input_current_write = oplus_chg_set_input_current,
	.float_voltage_write = smbchg_float_voltage_set,
	.term_current_set = smbchg_term_current_set,
	.charging_enable = smbchg_charging_enble,
	.charging_disable = smbchg_charging_disble,
	.get_charging_enable = smbchg_get_charge_enable,
	.charger_suspend = smbchg_usb_suspend_enable,
	.charger_unsuspend = smbchg_usb_suspend_disable,
	.set_rechg_vol = smbchg_set_rechg_vol,
	.reset_charger = smbchg_reset_charger,
	.read_full = smbchg_read_full,
	.set_charging_term_disable = oplus_set_chging_term_disable,
	.check_charger_resume = qcom_check_charger_resume,
	.get_chargerid_volt = smbchg_get_chargerid_volt,
	.set_chargerid_switch_val = smbchg_set_chargerid_switch_val,
	.get_chargerid_switch_val = smbchg_get_chargerid_switch_val,
	.need_to_check_ibatt = smbchg_need_to_check_ibatt,
	.get_chg_current_step = smbchg_get_chg_current_step,
	.get_charger_type = opchg_get_charger_type,
	.get_real_charger_type = opchg_get_real_charger_type,
	.get_charger_volt = qpnp_get_prop_charger_voltage_now,
	.get_ibus = oplus_chg_get_ibus,
	.check_chrdet_status = oplus_chg_is_usb_present,
	.get_instant_vbatt = qpnp_get_battery_voltage,
	.get_boot_mode = get_boot_mode,
	.get_boot_reason = smbchg_get_boot_reason,
	.get_rtc_soc = oplus_chg_get_shutdown_soc,
	.set_rtc_soc = oplus_chg_backup_soc,
	.get_aicl_ma = smbchg_get_aicl_level_ma,
	.rerun_aicl = smbchg_rerun_aicl,
	.tlim_en = smbchg_force_tlim_en,
	.set_system_temp_level = smbchg_system_temp_level_set,
	.otg_pulse_skip_disable = smbchg_set_prop_flash_active,
	.set_dp_dm = smbchg_dp_dm,
	.calc_flash_current = smbchg_calc_max_flash_current,
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oplus_chg_get_dyna_aicl_result,
#endif
	.get_shortc_hw_gpio_status = oplus_chg_get_shortc_hw_gpio_status,
};

/* ===== pm660l_bob regulator mode get/set =====
 * NOTE: guarded by #ifndef VENDOR_EDIT in the original; with VENDOR_EDIT defined
 * above this block is compiled out (dead). Kept verbatim for completeness.
 */
