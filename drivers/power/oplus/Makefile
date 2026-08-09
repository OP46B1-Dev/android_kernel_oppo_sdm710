# SPDX-License-Identifier: GPL-2.0
#
# OPLUS charging stack (sdm670R / OP46B1 / SDM710 + PMI632).
# Hooks into the qcom QPNP SMB2 charger (drivers/power/supply/qcom/) via
# function-pointer callbacks (oplus_smb_hook); selected by CONFIG_OPLUS_CHARGER.

obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_charger.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_gauge.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_vooc.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_adapter.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_short.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_short_ic.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_bq27541.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_sha1_hmac.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_debug_info.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_smb_hook_impl.o
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus_sdm670R_charger.o
obj-$(CONFIG_OPLUS_CHARGER)	+= vooc_ic/
obj-$(CONFIG_OPLUS_CHARGER)	+= adapter_ic/

# A few sources use angle-bracket includes of same-directory headers
# (e.g. oplus_bq27541.c <oplus_bq27541.h>); add this dir to the search path.
subdir-ccflags-y		+= -I$(src)
