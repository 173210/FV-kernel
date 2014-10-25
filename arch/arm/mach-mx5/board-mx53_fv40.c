/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fec.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/mpr.h>
#include <linux/fsl_devices.h>
#include <linux/ahci_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/android_pmem.h>
#include <linux/pwm_backlight.h>
#include <linux/mxcfb.h>
#include <linux/ipu.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mfd/da9052/da9052.h>
#include <video/platform_lcd.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/memblock.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/ipu-v3.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx53.h>
#include <mach/ahci_sata.h>
#include <mach/imx_rfkill.h>
#include <mach/mxc_asrc.h>
#include <mach/mxc_dvfs.h>
#include <linux/mfd/tc35894.h>
#include <linux/input/matrix_keypad.h>
#include <mach/check_fuse.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include "crm_regs.h"
#include "devices-imx53.h"
#include "devices.h"
#include "usb.h"
#include "android.h"
#include "pmic.h"

#include <linux/power_supply.h>

#include "mx53_fv40_gpio_conf.c"
/* MX53 FV40 GPIO PIN configurations */
#define MX53_FV40_KEY_RESET	IMX_GPIO_NR(1, 2)
#define MX53_FV40_SATA_CLK_GPEN	IMX_GPIO_NR(1, 4)
#define MX53_FV40_PMIC_FAULT	IMX_GPIO_NR(1, 5)

#define MX53_UPGRADE_LED_RED    IMX_GPIO_NR(4, 0)
#define MX53_UPGRADE_LED_GREEN    IMX_GPIO_NR(4, 3)

#define MX53_FV40_SYS_ON_OFF_CTL	IMX_GPIO_NR(1, 7)
#define MX53_FV40_PMIC_ON_OFF_REQ	IMX_GPIO_NR(1, 8)

#define MX53_FV40_FEC_INT	IMX_GPIO_NR(2, 4)
#define MX53_FV40_HEADPHONE_DEC	IMX_GPIO_NR(2, 5)
#define MX53_FV40_ZIGBEE_INT	IMX_GPIO_NR(2, 6)
#define MX53_FV40_ZIGBEE_RESET_B	IMX_GPIO_NR(2, 7)
#define MX53_FV40_GPS_RESET_B	IMX_GPIO_NR(2, 12)
#define MX53_FV40_WAKEUP_ZIGBEE	IMX_GPIO_NR(2, 13)
#define MX53_FV40_UI2		IMX_GPIO_NR(2, 14)
#define MX53_FV40_UI1		IMX_GPIO_NR(2, 15)
#define MX53_FV40_FEC_PWR_EN	IMX_GPIO_NR(2, 16)
#define MX53_FV40_LID_OPN_CLS_SW	IMX_GPIO_NR(2, 23)
#define MX53_FV40_GPS_PPS	IMX_GPIO_NR(2, 24)
#define MX53_FV40_ECSPI1_CS0	IMX_GPIO_NR(2, 30)

#define MX53_FV40_DCDC1V8_EN	IMX_GPIO_NR(3, 1)
#define MX53_FV40_AUD_AMP_STBY_B	IMX_GPIO_NR(3, 2)
#define MX53_FV40_SATA_PWR_EN	IMX_GPIO_NR(3, 3)
#define MX53_FV40_TPM_OSC_EN	IMX_GPIO_NR(3, 4)
#define MX53_FV40_WLAN_PD	IMX_GPIO_NR(3, 5)
#define MX53_FV40_WiFi_BT_PWR_EN	IMX_GPIO_NR(3, 10)
#define MX53_FV40_RECOVERY_MODE_SW	IMX_GPIO_NR(3, 11)
#define MX53_FV40_USB_OTG_OC	IMX_GPIO_NR(3, 12)
#define MX53_FV40_SD1_CD         IMX_GPIO_NR(3, 13)
#define MX53_FV40_USB_HUB_RESET_B	IMX_GPIO_NR(3, 14)
#define MX53_FV40_eCOMPASS_INT	IMX_GPIO_NR(3, 15)
#define MX53_FV40_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define MX53_FV40_CAP_TCH_INT1	IMX_GPIO_NR(3, 20)
#define MX53_FV40_BT_PRIORITY	IMX_GPIO_NR(3, 21)
#define MX53_FV40_ALS_INT	IMX_GPIO_NR(3, 22)
#define MX53_FV40_SD1_PW	IMX_GPIO_NR(3, 23)
#define MX53_FV40_TPM_INT	IMX_GPIO_NR(3, 26)
#define MX53_FV40_MODEM_WKUP	IMX_GPIO_NR(3, 27)
#define MX53_FV40_BT_RESET	IMX_GPIO_NR(3, 28)
#define MX53_FV40_TPM_RST_B	IMX_GPIO_NR(3, 29)
#define MX53_FV40_CHRG_OR_CMOS	IMX_GPIO_NR(3, 30)
#define MX53_FV40_CAP_TCH_INT0	IMX_GPIO_NR(3, 31)

#define MX53_FV40_MODEM_DISABLE_B	IMX_GPIO_NR(4, 10)
#define MX53_FV40_SD1_WP         IMX_GPIO_NR(4, 11)
#define MX53_FV40_DCDC5V_BB_EN	IMX_GPIO_NR(4, 14)
#define MX53_FV40_WLAN_HOST_WAKE	IMX_GPIO_NR(4, 15)

#define MX53_FV40_HDMI_RESET_B   IMX_GPIO_NR(5, 0)
#define MX53_FV40_MODEM_RESET_B  IMX_GPIO_NR(5, 2)
#define MX53_FV40_KEY_INT	IMX_GPIO_NR(5, 4)

#define MX53_FV40_CAP_TCH_FUN0	IMX_GPIO_NR(6, 6)
#define MX53_FV40_CSI0_RST       IMX_GPIO_NR(6, 9)
#define MX53_FV40_CSI0_PWN       IMX_GPIO_NR(6, 10)
#define MX53_FV40_OSC_CKIH1_EN	IMX_GPIO_NR(6, 11)
#define MX53_FV40_HDMI_INT	IMX_GPIO_NR(6, 12)
#define MX53_FV40_LCD_PWR_EN	IMX_GPIO_NR(6, 13)
#define MX53_FV40_ACCL_INT1_IN	IMX_GPIO_NR(6, 15)
#define MX53_FV40_ACCL_INT2_IN	IMX_GPIO_NR(6, 16)
#define MX53_FV40_AC_IN		IMX_GPIO_NR(6, 17)
#define MX53_FV40_PWR_GOOD	IMX_GPIO_NR(6, 18)

#define MX53_FV40_CABC_EN0	IMX_GPIO_NR(7, 2)
#define MX53_FV40_DOCK_DECTECT	IMX_GPIO_NR(7, 3)
#define FV40_FEC_PHY_RST		IMX_GPIO_NR(7, 6)
#define MX53_FV40_USER_DEG_CHG_NONE	IMX_GPIO_NR(7, 7)
#define MX53_FV40_OTG_VBUS	IMX_GPIO_NR(7, 8)
#define MX53_FV40_DEVELOP_MODE_SW	IMX_GPIO_NR(7, 9)
#define MX53_FV40_CABC_EN1	IMX_GPIO_NR(7, 10)
#define MX53_FV40_PMIC_INT	IMX_GPIO_NR(7, 11)
#define MX53_FV40_CAP_TCH_FUN1	IMX_GPIO_NR(7, 13)

#define MX53_FV40_MRS_DETECT	IMX_GPIO_NR(2, 23) /* GPIO_2_23 */
#define MX53_FV40_WLAN_RESET	IMX_GPIO_NR(3, 28)	/* GPIO_3_28 */
#define MX53_FV40_WLAN_RESET_QA1 IMX_GPIO_NR(3, 15)	/*GPIO_3_15 */
#define MX53_FV40_LCD_PWR_EN_QA1 IMX_GPIO_NR(3, 28)	/* GPIO_3_28 */

#define MX53_FV40_LCD_STANDBY	IMX_GPIO_NR(2, 25) /* GPIO_2_25 */
#define MX53_FV40_LED_CTL1	IMX_GPIO_NR(2, 25) /* GPIO_2_25 */

#define MX53_FV40_LED_CTL2	IMX_GPIO_NR(3, 3) /* GPIO_3_3 */
#define MX53_FV40_LED_CTL3	IMX_GPIO_NR(3, 4) /* GPIO_3_4 */
#define MX53_FV40_LED_CTL4	IMX_GPIO_NR(6, 18) /* GPIO_6_18 */

#define MX53_FV40_LED_CHARGER_NOW_OR_CMOS_RUN	IMX_GPIO_NR(3, 30) /* GPIO_3_30 */
#define MX53_FV40_LED_USER_DEBUG_OR_CHARGER_DONE	IMX_GPIO_NR(7, 7) /* GPIO_7_7 */

#define MX53_FV40_DI0_DISP_CLK	IMX_GPIO_NR(4, 16)
#define MX53_FV40_DI0_PIN15	IMX_GPIO_NR(4, 17)
#define MX53_FV40_DI0_PIN2	IMX_GPIO_NR(4, 18)
#define MX53_FV40_DI0_PIN3	IMX_GPIO_NR(4, 19)
#define MX53_FV40_DISP0_DAT0	IMX_GPIO_NR(4, 21)
#define MX53_FV40_DISP0_DAT1	IMX_GPIO_NR(4, 22)
#define MX53_FV40_DISP0_DAT2	IMX_GPIO_NR(4, 23)
#define MX53_FV40_DISP0_DAT3	IMX_GPIO_NR(4, 24)
#define MX53_FV40_DISP0_DAT4	IMX_GPIO_NR(4, 25)
#define MX53_FV40_DISP0_DAT5	IMX_GPIO_NR(4, 26)
#define MX53_FV40_DISP0_DAT6	IMX_GPIO_NR(4, 27)
#define MX53_FV40_DISP0_DAT7	IMX_GPIO_NR(4, 28)
#define MX53_FV40_DISP0_DAT8	IMX_GPIO_NR(4, 29)
#define MX53_FV40_DISP0_DAT9	IMX_GPIO_NR(4, 30)
#define MX53_FV40_DISP0_DAT10	IMX_GPIO_NR(4, 31)
#define MX53_FV40_DISP0_DAT11	IMX_GPIO_NR(5, 5)
#define MX53_FV40_DISP0_DAT12	IMX_GPIO_NR(5, 6)
#define MX53_FV40_DISP0_DAT13	IMX_GPIO_NR(5, 7)
#define MX53_FV40_DISP0_DAT14	IMX_GPIO_NR(5, 8)
#define MX53_FV40_DISP0_DAT15	IMX_GPIO_NR(5, 9)
#define MX53_FV40_DISP0_DAT16	IMX_GPIO_NR(5, 10)
#define MX53_FV40_DISP0_DAT17	IMX_GPIO_NR(5, 11)
#define MX53_FV40_DISP0_DAT18	IMX_GPIO_NR(5, 12)
#define MX53_FV40_DISP0_DAT19	IMX_GPIO_NR(5, 13)
#define MX53_FV40_DISP0_DAT20	IMX_GPIO_NR(5, 14)
#define MX53_FV40_DISP0_DAT21	IMX_GPIO_NR(5, 15)
#define MX53_FV40_DISP0_DAT22	IMX_GPIO_NR(5, 16)
#define MX53_FV40_DISP0_DAT23	IMX_GPIO_NR(5, 17)
#define GPIO1_18	IMX_GPIO_NR(1, 18)
#define GPIO1_20	IMX_GPIO_NR(1, 20)
#define GPIO1_16	IMX_GPIO_NR(1, 16)
#define GPIO1_17	IMX_GPIO_NR(1, 17)
#define GPIO1_19	IMX_GPIO_NR(1, 19)
#define GPIO1_21	IMX_GPIO_NR(1, 21)
#define GPIO1_11	IMX_GPIO_NR(1, 11)
#define GPIO1_10	IMX_GPIO_NR(1, 10)
#define GPIO1_15	IMX_GPIO_NR(1, 15)
#define GPIO1_14	IMX_GPIO_NR(1, 14)
#define GPIO1_13	IMX_GPIO_NR(1, 13)
#define GPIO1_12	IMX_GPIO_NR(1, 12)
#define GPIO2_8		IMX_GPIO_NR(2, 8)
#define GPIO2_9		IMX_GPIO_NR(2, 9)
#define GPIO2_10	IMX_GPIO_NR(2, 10)
#define GPIO2_11	IMX_GPIO_NR(2, 11)
#define GPIO2_0		IMX_GPIO_NR(2, 0)
#define GPIO2_1		IMX_GPIO_NR(2, 1)
#define GPIO2_2		IMX_GPIO_NR(2, 2)
#define GPIO2_3		IMX_GPIO_NR(2, 3)
#define GPIO7_5		IMX_GPIO_NR(7, 5)
#define GPIO7_4		IMX_GPIO_NR(7, 4)
#define GPIO7_6		IMX_GPIO_NR(7, 6)

static unsigned lcd_pwer_pin = MX53_FV40_LCD_PWR_EN;
unsigned wlan_reset_pin = MX53_FV40_WLAN_RESET;
unsigned wlan_pwr_pin = MX53_FV40_WiFi_BT_PWR_EN;
EXPORT_SYMBOL_GPL(wlan_reset_pin);
EXPORT_SYMBOL_GPL(wlan_pwr_pin);
#define TZIC_WAKEUP0_OFFSET	0x0E00
#define TZIC_WAKEUP1_OFFSET	0x0E04
#define TZIC_WAKEUP2_OFFSET	0x0E08
#define TZIC_WAKEUP3_OFFSET	0x0E0C
#define GPIO7_0_11_IRQ_BIT	(0x1<<11)
#define GPIO3_0_31_IRQ_BIT	(0x1<<23)
#define GPIO2_0_23_IRQ_BIT 	(0x1<<21)
#define GPIO5_0_4_IRQ_BIT	(0x1<<7)
#define GPIO1_0_8_IRQ_BIT 	(0x1<<18)

#define TC35894_IRQ_BASE 109 //reserverd irq


void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk, *sata_ref_clk;
static int fs_in_sdcard;

extern char *lp_reg_id;
extern char *gp_reg_id;
extern void mx5_cpu_regulator_init(void);
extern int mx53_fv40_init_da9052(void);

static iomux_v3_cfg_t mx53_fv40_pads[] = {
	/* unused GPIO config*/
	MX53_PAD_EIM_WAIT__GPIO5_0,
	MX53_PAD_EIM_RW__GPIO2_26,
	MX53_PAD_EIM_LBA__GPIO2_27,
	MX53_PAD_EIM_EB0__GPIO2_28,
	MX53_PAD_EIM_EB1__GPIO2_29,
	MX53_PAD_EIM_EB2__GPIO2_30,
	MX53_PAD_EIM_EB3__GPIO2_31,
	MX53_PAD_EIM_CS1__GPIO2_24,
	MX53_PAD_EIM_A16__GPIO2_22,
	MX53_PAD_EIM_A17__GPIO2_21,
	MX53_PAD_EIM_A18__GPIO2_20,
	MX53_PAD_EIM_A19__GPIO2_19,
	MX53_PAD_EIM_A20__GPIO2_18,
	MX53_PAD_EIM_A21__GPIO2_17,
	MX53_PAD_EIM_D16__GPIO3_16,
	MX53_PAD_EIM_D17__GPIO3_17,
	MX53_PAD_EIM_D18__GPIO3_18,
	MX53_PAD_EIM_D19__GPIO3_19,
	MX53_PAD_EIM_DA0__GPIO3_0,
	MX53_PAD_EIM_DA6__GPIO3_6,
	MX53_PAD_EIM_DA7__GPIO3_7,
	MX53_PAD_EIM_DA8__GPIO3_8,
	MX53_PAD_EIM_DA9__GPIO3_9,
	MX53_PAD_NANDF_ALE__GPIO6_8,
	MX53_PAD_NANDF_CLE__GPIO6_7,
	MX53_PAD_NANDF_CS1__GPIO6_14,
	MX53_PAD_CSI0_DAT4__GPIO5_22,
	MX53_PAD_CSI0_DAT5__GPIO5_23,
	MX53_PAD_CSI0_DAT6__GPIO5_24,
	MX53_PAD_CSI0_DAT7__GPIO5_25,
	MX53_PAD_CSI0_DAT12__GPIO5_30,
	MX53_PAD_CSI0_DAT13__GPIO5_31,
	MX53_PAD_CSI0_DAT14__GPIO6_0,
	MX53_PAD_CSI0_DAT15__GPIO6_1,
	MX53_PAD_CSI0_DAT16__GPIO6_2,
	MX53_PAD_CSI0_DAT17__GPIO6_3,
	MX53_PAD_CSI0_DAT18__GPIO6_4,
	MX53_PAD_CSI0_DAT19__GPIO6_5,
	MX53_PAD_CSI0_VSYNC__GPIO5_21,
	MX53_PAD_CSI0_MCLK__GPIO5_19,
	MX53_PAD_CSI0_PIXCLK__GPIO5_18,
	MX53_PAD_CSI0_DATA_EN__GPIO5_20,
	MX53_PAD_DI0_PIN4__GPIO4_20,
	MX53_PAD_KEY_ROW1__GPIO4_9,
	MX53_PAD_GPIO_11__GPIO4_1,
	MX53_PAD_GPIO_19__GPIO4_5,
	MX53_PAD_EIM_D20__GPIO3_20,
	MX53_PAD_EIM_D21__GPIO3_21,
	MX53_PAD_EIM_D22__GPIO3_22,
	MX53_PAD_EIM_D26__GPIO3_26,
	MX53_PAD_EIM_D27__GPIO3_27,
	MX53_PAD_NANDF_WE_B__GPIO6_12,
	MX53_PAD_NANDF_RE_B__GPIO6_13,
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	MX53_PAD_NANDF_RB0__GPIO6_10,
	MX53_PAD_PATA_DATA6__GPIO2_6,
	MX53_PAD_PATA_DATA7__GPIO2_7,
	MX53_PAD_PATA_DATA12__GPIO2_12,
	MX53_PAD_PATA_DATA13__GPIO2_13,
	MX53_PAD_PATA_DATA14__GPIO2_14,
	MX53_PAD_PATA_DATA15__GPIO2_15,
	MX53_PAD_PATA_DIOR__GPIO7_3,
	MX53_PAD_EIM_DA11__GPIO3_11,
	MX53_PAD_EIM_DA12__GPIO3_12,
	MX53_PAD_EIM_DA1__GPIO3_1,
	MX53_PAD_EIM_DA2__GPIO3_2,
	MX53_PAD_EIM_DA3__GPIO3_3,
	MX53_PAD_EIM_DA4__GPIO3_4,
	MX53_PAD_EIM_DA5__GPIO3_5,
	MX53_PAD_EIM_D29__GPIO3_29,
	MX53_PAD_LVDS0_TX3_P__GPIO7_22,
	MX53_PAD_LVDS0_CLK_P__GPIO7_24,
	MX53_PAD_LVDS0_TX2_P__GPIO7_26,
	MX53_PAD_LVDS0_TX1_P__GPIO7_28,
	MX53_PAD_LVDS0_TX0_P__GPIO7_30,
	MX53_PAD_LVDS1_TX3_P__GPIO6_22,
	MX53_PAD_LVDS1_TX2_P__GPIO6_24,
	MX53_PAD_LVDS1_CLK_P__GPIO6_26,
	MX53_PAD_LVDS1_TX1_P__GPIO6_28,
	MX53_PAD_LVDS1_TX0_P__GPIO6_30,
	MX53_PAD_GPIO_18__GPIO7_13,
	MX53_PAD_GPIO_14__GPIO4_4,
	MX53_PAD_PATA_DA_2__GPIO7_8,
	MX53_PAD_PATA_CS_0__GPIO7_9,
	MX53_PAD_PATA_CS_1__GPIO7_10,
	MX53_PAD_KEY_COL2__GPIO4_10,
	MX53_PAD_KEY_ROW2__GPIO4_11,
	MX53_PAD_NANDF_CS2__GPIO6_15,
	MX53_PAD_NANDF_CS3__GPIO6_16,
	MX53_PAD_PATA_DMACK__GPIO6_18,
	MX53_PAD_PATA_INTRQ__GPIO7_2,
	MX53_PAD_GPIO_12__GPIO4_2,
	MX53_PAD_KEY_COL4__GPIO4_14,
	MX53_PAD_KEY_ROW4__GPIO4_15,
	MX53_PAD_GPIO_4__GPIO1_4,
	MX53_PAD_EIM_A22__GPIO2_16,
	MX53_PAD_EIM_DA14__GPIO3_14,
	/* LCD_STBYB */
	MX53_PAD_EIM_OE__GPIO2_25,
	/*CAP_TCH_FUN0*/
	MX53_PAD_EIM_A23__GPIO6_6,
	/*MRS_OUT*/
	MX53_PAD_EIM_CS0__GPIO2_23,

	MX53_PAD_GPIO_0__GPIO1_0,

	/* BT: UART3*/
	MX53_PAD_EIM_D24__UART3_TXD_MUX,
	MX53_PAD_EIM_D25__UART3_RXD_MUX,

	/* KEY_INT */
	MX53_PAD_EIM_A24__GPIO5_4,
	/* FEC_nRST */
	MX53_PAD_EIM_A25__GPIO5_2,
	/* SD_PWR_ON*/
	MX53_PAD_EIM_D23__GPIO3_23,
	/* LCD_VCC_EN */
	MX53_PAD_EIM_D28__GPIO3_28,
	/* CHARGER_NOW_OR_CMOS_RUN */
	MX53_PAD_EIM_D30__GPIO3_30,
	/* CAP_TCH_INT0 */
	MX53_PAD_EIM_D31__GPIO3_31,
	/* WiFi_BT_PWR_EN */
	MX53_PAD_EIM_DA10__GPIO3_10,
	/* SD1_CD */
	MX53_PAD_EIM_DA13__GPIO3_13,
	/* WLAN_RESET */
	MX53_PAD_EIM_DA15__GPIO3_15,
	/* OSC_CKIH1_EN */
	MX53_PAD_NANDF_CS0__GPIO6_11,
	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
#if 0
	/* SD2 */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
#endif
	/* UART2 */
	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
	/* USER_DEBUG_OR_CHARGER_DONE */
	MX53_PAD_PATA_DA_1__GPIO7_7,
	/* SD3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	MX53_PAD_PATA_DA_0__ESDHC3_RST,

	/* HEADPHONE DET*/
	MX53_PAD_PATA_DATA5__GPIO2_5,
	/* Adapt_DET */
	MX53_PAD_PATA_DIOW__GPIO6_17,

	MX53_PAD_GPIO_1__PWM2_PWMO,
	/* KEY_RESET */
	MX53_PAD_GPIO_2__GPIO1_2,

	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,

	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,

	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,
	/* PMIC_FAULT */
	MX53_PAD_GPIO_5__GPIO1_5,
	/* SYS_ON_OFF_CTL */
	MX53_PAD_GPIO_7__GPIO1_7,
	/* PMIC_ON_OFF_REQ */
	MX53_PAD_GPIO_8__GPIO1_8,
	/* TP49 */
	MX53_PAD_GPIO_10__GPIO4_0,
	/* TP58 */
	MX53_PAD_GPIO_13__GPIO4_3,
	/* PMIC_INT */
	MX53_PAD_GPIO_16__GPIO7_11,
	/* TP60 */
	MX53_PAD_GPIO_17__GPIO7_12,
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_type, ev_code, act_low, descr, wake, debounce_ms) \
{                                                               \
	.gpio           = gpio_num,                             \
	.type           = ev_type,                               \
	.code           = ev_code,                              \
	.active_low     = act_low,                              \
	.desc           = "btn " descr,                         \
	.wakeup         = wake,                                 \
	.debounce_interval = debounce_ms,                       \
}

static struct gpio_keys_button fv40_buttons[] = {
	GPIO_BUTTON(MX53_FV40_PMIC_ON_OFF_REQ, EV_KEY, KEY_POWER, 1, "power", 0, 100),
//	GPIO_BUTTON(MX53_FV40_UI1, KEY_VOLUMEUP, 1, "volume-up", 0, 0),
//	GPIO_BUTTON(MX53_FV40_UI2, KEY_VOLUMEDOWN, 1, "volume-down", 0, 0),
	GPIO_BUTTON(MX53_FV40_MRS_DETECT, EV_SW, KEY_MRS, 1, "mag sensor", 1, 0),
};

static struct gpio_keys_platform_data fv40_button_data = {
	.buttons        = fv40_buttons,
	.nbuttons       = ARRAY_SIZE(fv40_buttons),
};

static struct platform_device fv40_button_device = {
	.name           = "gpio-keys",
	.id             = -1,
	.num_resources  = 0,
	.dev            = {
		.platform_data = &fv40_button_data,
		}
};

static void __init fv40_add_device_buttons(void)
{
	platform_device_register(&fv40_button_device);
}
#else
static void __init fv40_add_device_buttons(void) {}
#endif

#define GPIO_LED(desc, def_trigger, pin, act_low, keep, def_state)	\
{									\
	.name 			= desc,					\
	.default_trigger 	= def_trigger,				\
	.gpio 			= pin, 					\
	.active_low 		= act_low, 				\
	.retain_state_suspended = keep, 				\
	.default_state 		= def_state, 				\
}

#define LED_RED_PIN 	MX53_FV40_LED_CHARGER_NOW_OR_CMOS_RUN
#define LED_GREEN_PIN	MX53_FV40_LED_USER_DEBUG_OR_CHARGER_DONE

static struct gpio_led fv40_leds[] = {
	GPIO_LED("led_red", "da9052-bat-charging", LED_RED_PIN, 0, 1, 2),
	GPIO_LED("led_green", "da9052-bat-charging-or-full",  LED_GREEN_PIN, 0, 1, 2),
	GPIO_LED("pic-standby", NULL, MX53_FV40_SYS_ON_OFF_CTL, 0, 1, 1),
	GPIO_LED("upgrade_red", NULL, MX53_UPGRADE_LED_RED, 0, 1, 1),
	GPIO_LED("upgrade_green", NULL, MX53_UPGRADE_LED_GREEN, 0, 1, 1),

};

static struct gpio_led_platform_data fv40_leds_data = {
	.leds		= fv40_leds,
	.num_leds	= ARRAY_SIZE(fv40_leds),
};

static struct platform_device fv40_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &fv40_leds_data,
	}
};

static void __init fv40_add_device_leds(void)
{
	platform_device_register(&fv40_led_device);
}
static const struct imxuart_platform_data mx53_fv40_uart_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
	.dma_req_rx = MX53_DMA_REQ_UART3_RX,
	.dma_req_tx = MX53_DMA_REQ_UART3_TX,
};

static inline void mx53_fv40_init_uart(void)
{
	imx53_add_imx_uart(0, NULL);
	imx53_add_imx_uart(1, NULL);
	imx53_add_imx_uart(2, &mx53_fv40_uart_data);
}

static inline void mx53_fv40_fec_reset(void)
{
	int ret;

	/* reset FEC PHY */
	ret = gpio_request(FV40_FEC_PHY_RST, "fec-phy-reset");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_FEC_PHY_RESET: %d\n", ret);
		return;
	}
	gpio_direction_output(FV40_FEC_PHY_RST, 0);
	gpio_set_value(FV40_FEC_PHY_RST, 0);
	msleep(1);
	gpio_set_value(FV40_FEC_PHY_RST, 1);
}

static struct fec_platform_data mx53_fv40_fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static const struct imxi2c_platform_data mx53_fv40_i2c_data __initconst = {
	.bitrate = 320000,
};

extern void __iomem *tzic_base;
static void fv40_da9053_irq_wakeup_only_fixup(void)
{
	if (NULL == tzic_base) {
		pr_err("fail to map MX53_TZIC_BASE_ADDR\n");
		return;
	}
	__raw_writel(0, tzic_base + TZIC_WAKEUP0_OFFSET);
        __raw_writel(GPIO3_0_31_IRQ_BIT|GPIO2_0_23_IRQ_BIT|GPIO1_0_8_IRQ_BIT, tzic_base + TZIC_WAKEUP1_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP2_OFFSET);
	/* only enable irq wakeup for da9053 */
        __raw_writel(GPIO7_0_11_IRQ_BIT|GPIO5_0_4_IRQ_BIT, tzic_base + TZIC_WAKEUP3_OFFSET);
	pr_info("only da9053 irq is wakeup-enabled\n");
}

static void fv40_suspend_enter(void)
{
#if 1
	if(!power_supply_is_system_supplied()) {
		printk("close lcd power\n");
		gpio_request(lcd_pwer_pin, "lcd-pwr-en");
		gpio_direction_output(lcd_pwer_pin, 0);
		gpio_free(lcd_pwer_pin);
	}
        fv40_da9053_irq_wakeup_only_fixup();
        da9053_suspend_cmd_hw();
#else

	if (board_is_rev(IMX_BOARD_REV_4)) {
		fv40_da9053_irq_wakeup_only_fixup();
		da9053_suspend_cmd_sw();
	} else {
		if (da9053_get_chip_version() != DA9053_VERSION_BB && da9053_get_chip_version() != DA9053_VERSION_GSL)
			fv40_da9053_irq_wakeup_only_fixup();

		da9053_suspend_cmd_hw();
	}
#endif
}

static void fv40_suspend_exit(void)
{
	if (da9053_get_chip_version())
		da9053_restore_volt_settings();
}

static struct mxc_pm_platform_data fv40_pm_data = {
	.suspend_enter = fv40_suspend_enter,
	.suspend_exit = fv40_suspend_exit,
};

static iomux_v3_cfg_t mx53_fv40_sdio_pads[3][2][15] = {
	{
		/* SD1 */
		{
			MX53_PAD_SD1_CMD__ESDHC1_CMD,
			MX53_PAD_SD1_CLK__ESDHC1_CLK,
			MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
			MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
			MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
			MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
		}, {
			MX53_PAD_SD1_CMD__GPIO1_18,
			MX53_PAD_SD1_CLK__GPIO1_20,
			MX53_PAD_SD1_DATA0__GPIO1_16,
			MX53_PAD_SD1_DATA1__GPIO1_17,
			MX53_PAD_SD1_DATA2__GPIO1_19,
			MX53_PAD_SD1_DATA3__GPIO1_21,
		},
	}, {
		/* SD2 */
		{
			MX53_PAD_SD2_CMD__ESDHC2_CMD,
			MX53_PAD_SD2_CLK__ESDHC2_CLK,
			MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
			MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
			MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
			MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
		}, {
			MX53_PAD_SD2_CMD__GPIO1_11,
			MX53_PAD_SD2_CLK__GPIO1_10,
			MX53_PAD_SD2_DATA0__GPIO1_15,
			MX53_PAD_SD2_DATA1__GPIO1_14,
			MX53_PAD_SD2_DATA2__GPIO1_13,
			MX53_PAD_SD2_DATA3__GPIO1_12,
		},
	}, {
		/* SD3 */
		{
			MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
			MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
			MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
			MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
			MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
			MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
			MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
			MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
			MX53_PAD_PATA_IORDY__ESDHC3_CLK,
			MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
			MX53_PAD_PATA_DA_0__ESDHC3_RST,
		}, {
			MX53_PAD_PATA_DATA8__GPIO2_8,
			MX53_PAD_PATA_DATA9__GPIO2_9,
			MX53_PAD_PATA_DATA10__GPIO2_10,
			MX53_PAD_PATA_DATA11__GPIO2_11,
			MX53_PAD_PATA_DATA0__GPIO2_0,
			MX53_PAD_PATA_DATA1__GPIO2_1,
			MX53_PAD_PATA_DATA2__GPIO2_2,
			MX53_PAD_PATA_DATA3__GPIO2_3,
			MX53_PAD_PATA_IORDY__GPIO7_5,
			MX53_PAD_PATA_RESET_B__GPIO7_4,
			MX53_PAD_PATA_DA_0__GPIO7_6,
		},
	},
};

static u32 sdio_gpios[3][15] = {
	{
		GPIO1_18,
		GPIO1_20,
		GPIO1_16,
		GPIO1_17,
		GPIO1_19,
		GPIO1_21,
	}, {
		GPIO1_11,
		GPIO1_10,
		GPIO1_15,
		GPIO1_14,
		GPIO1_13,
		GPIO1_12,
	}, {
		GPIO2_8,
		GPIO2_9,
		GPIO2_10,
		GPIO2_11,
		GPIO2_0,
		GPIO2_1,
		GPIO2_2,
		GPIO2_3,
		GPIO7_5,
		GPIO7_4,
		GPIO7_6,
	},
};
unsigned wifi_status;
EXPORT_SYMBOL_GPL(wifi_status);
int sdio_set_power(unsigned int sdhcid, unsigned int state)
{
	int i;
	if((sdhcid == 1) && !wifi_status)
		return 0;
	mxc_iomux_v3_setup_multiple_pads(mx53_fv40_sdio_pads[sdhcid][!state],
					 ARRAY_SIZE(mx53_fv40_sdio_pads[sdhcid][!state]));
	if(!state) {
		for (i = 0; i < ARRAY_SIZE(sdio_gpios[sdhcid]); i++) {
			gpio_request(sdio_gpios[sdhcid][i], NULL);
			gpio_direction_output(sdio_gpios[sdhcid][i], 0);
			gpio_free(sdio_gpios[sdhcid][i]);
		}
	}
	return 0;
}
/* SDIO Card Slot */
static const struct esdhc_platform_data mx53_fv40_sd1_data __initconst = {
	.cd_gpio = MX53_FV40_SD1_CD,
//	.wp_gpio = MX53_FV40_SD1_WP,
};

/* SDIO Wifi */
static const struct esdhc_platform_data mx53_fv40_sd2_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
//	.non_remove = 1,
	.pmflag = 1,
	.set_power = sdio_set_power,
};

/* SDIO Internal eMMC */
static const struct esdhc_platform_data mx53_fv40_sd3_data __initconst = {
	.always_present = 1,
	.support_8bit = 1,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {};

static const u32 tc35894_keymap[TC_KPD_ROWS*TC_KPD_COLUMNS]= {
	/* 削除/BS     一括          Q     O      A       Z       SHIFT          / */
	KEY_BACKSPACE, KEY_YI_KUO, KEY_Q, KEY_O, KEY_A, KEY_Z, KEY_LEFTSHIFT, KEY_SLASH,
	/* メニュー  英英        W      P      S      X        CNTL       BACK */
	KEY_MENU, KEY_YIN_YIN, KEY_W, KEY_P, KEY_S, KEY_X, KEY_LEFTCTRL, KEY_BACK,
	/* 国語     お気に入り    E     L       D      C       スペル       . */
	KEY_GUO_YU, KEY_JPNONE, KEY_E, KEY_L, KEY_D, KEY_C, KEY_JPNTHREE, KEY_DOT,
	/* 英和         学習     R      Hypen     F       V      ESC */
	KEY_YIN_HE, KEY_LEARN, KEY_R, KEY_MINUS, KEY_F, KEY_V, KEY_BACK, 197,
	/* NUM_LOCK         T    チェツク ？   G       B     SPACE     DPAD_LEFT */
	KEY_NUMLOCK, 202, KEY_T, KEY_JPN_SYM, KEY_G, KEY_B, KEY_SPACE, KEY_LEFT,
	/*         Y      HOME      H       N      ENTER    DPAD_RIGHT */
	201, 203, KEY_Y, KEY_HOME, KEY_H, KEY_N, KEY_ENTER, KEY_RIGHT,
	/*          U      CAPS_LOCK     J      M     MENU     DPAD_UP */
	200, 204, KEY_U, KEY_CAPSLOCK, KEY_J, KEY_M, KEY_MENU, KEY_UP,
	/*  PLAY          和英       I      TAB      K     VOLUME_UP     VOLUME_DOWN    DPAD_DOWN */
	KEY_PLAYPAUSE, KEY_HE_YIN, KEY_I, KEY_TAB, KEY_K, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_DOWN,
};

static struct matrix_keymap_data tc35894_keymap_data = {
        .keymap = tc35894_keymap,
        .keymap_size = TC_KPD_ROWS*TC_KPD_COLUMNS,
};

static struct tc35894_keypad_platform_data tc35894_key_platform = {
	.keymap_data = &tc35894_keymap_data,
	.krow = 8,
	.kcol = 10,
	.enable_wakeup = 1,
	.no_autorepeat = 1,
};

static struct tc35894_platform_data tc35894_plat_data= {
        .irq_base = TC35894_IRQ_BASE,
        .keypad = &tc35894_key_platform,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "wm8960",
	 .addr = 0x1a,
	 },
        {
        .type = "tc35894",
        .addr = 0x45,
        .irq = gpio_to_irq(MX53_FV40_KEY_INT),
        .platform_data = &tc35894_plat_data,
        },

};

static iomux_v3_cfg_t mx53_fv40_lcd_pads[2][28] = {
	{
		/* GPIO */
		MX53_PAD_DI0_DISP_CLK__GPIO4_16,
		MX53_PAD_DI0_PIN15__GPIO4_17,
		MX53_PAD_DI0_PIN2__GPIO4_18,
		MX53_PAD_DI0_PIN3__GPIO4_19,
		MX53_PAD_DISP0_DAT0__GPIO4_21,
		MX53_PAD_DISP0_DAT1__GPIO4_22,
		MX53_PAD_DISP0_DAT2__GPIO4_23,
		MX53_PAD_DISP0_DAT3__GPIO4_24,
		MX53_PAD_DISP0_DAT4__GPIO4_25,
		MX53_PAD_DISP0_DAT5__GPIO4_26,
		MX53_PAD_DISP0_DAT6__GPIO4_27,
		MX53_PAD_DISP0_DAT7__GPIO4_28,
		MX53_PAD_DISP0_DAT8__GPIO4_29,
		MX53_PAD_DISP0_DAT9__GPIO4_30,
		MX53_PAD_DISP0_DAT10__GPIO4_31,
		MX53_PAD_DISP0_DAT11__GPIO5_5,
		MX53_PAD_DISP0_DAT12__GPIO5_6,
		MX53_PAD_DISP0_DAT13__GPIO5_7,
		MX53_PAD_DISP0_DAT14__GPIO5_8,
		MX53_PAD_DISP0_DAT15__GPIO5_9,
		MX53_PAD_DISP0_DAT16__GPIO5_10,
		MX53_PAD_DISP0_DAT17__GPIO5_11,
		MX53_PAD_DISP0_DAT18__GPIO5_12,
		MX53_PAD_DISP0_DAT19__GPIO5_13,
		MX53_PAD_DISP0_DAT20__GPIO5_14,
		MX53_PAD_DISP0_DAT21__GPIO5_15,
		MX53_PAD_DISP0_DAT22__GPIO5_16,
		MX53_PAD_DISP0_DAT23__GPIO5_17,
	}, {
		/* DISPLAY */
		MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
		MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
		MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
		MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
		MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
		MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
		MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
		MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
		MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
		MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
		MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
		MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
		MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
		MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
		MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
		MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
		MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
		MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
		MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
		MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
		MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
		MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
		MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
		MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
		MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
		MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
		MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
		MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	},
};

static u32 mx53_fv40_lcd_gpios[] = {
	MX53_FV40_DI0_DISP_CLK,
	MX53_FV40_DI0_PIN15,
	MX53_FV40_DI0_PIN2,
	MX53_FV40_DI0_PIN3,
	MX53_FV40_DISP0_DAT0,
	MX53_FV40_DISP0_DAT1,
	MX53_FV40_DISP0_DAT2,
	MX53_FV40_DISP0_DAT3,
	MX53_FV40_DISP0_DAT4,
	MX53_FV40_DISP0_DAT5,
	MX53_FV40_DISP0_DAT6,
	MX53_FV40_DISP0_DAT7,
	MX53_FV40_DISP0_DAT8,
	MX53_FV40_DISP0_DAT9,
	MX53_FV40_DISP0_DAT10,
	MX53_FV40_DISP0_DAT11,
	MX53_FV40_DISP0_DAT12,
	MX53_FV40_DISP0_DAT13,
	MX53_FV40_DISP0_DAT14,
	MX53_FV40_DISP0_DAT15,
	MX53_FV40_DISP0_DAT16,
	MX53_FV40_DISP0_DAT17,
	MX53_FV40_DISP0_DAT18,
	MX53_FV40_DISP0_DAT19,
	MX53_FV40_DISP0_DAT20,
	MX53_FV40_DISP0_DAT21,
	MX53_FV40_DISP0_DAT22,
	MX53_FV40_DISP0_DAT23,
};

static void tm050rdh05_lcd_power_set(struct plat_lcd_data *pd,
				     unsigned int power)
{
	int i;

	gpio_request(lcd_pwer_pin, "lcd-pwr-en");
	gpio_request(MX53_FV40_LCD_STANDBY, "lcd-standby");
	/* 开机power_supply要比lcd晚注册，所以开机过程中
	 * power_supply_is_system_supplied一直会返回0 */
	if (power) {
		gpio_direction_output(lcd_pwer_pin, 1);
		msleep(10);
		gpio_direction_output(MX53_FV40_LCD_STANDBY, 1);
		printk("open lcd standby\n");
		mxc_iomux_v3_setup_multiple_pads(mx53_fv40_lcd_pads[1],
						 ARRAY_SIZE(mx53_fv40_lcd_pads[1]));
	} else{
		if (!power_supply_is_system_supplied()) { //充电时不关LCD,因为要点亮LED
			/* cut off lcd power */
			gpio_direction_output(lcd_pwer_pin, 0);
		}
		gpio_direction_output(MX53_FV40_LCD_STANDBY, 0);
		/* config lcd pads as GPIO with low-level output */
		mxc_iomux_v3_setup_multiple_pads(mx53_fv40_lcd_pads[0],
				ARRAY_SIZE(mx53_fv40_lcd_pads[0]));
		for (i = 0; i < ARRAY_SIZE(mx53_fv40_lcd_gpios); i++) {
			gpio_request(mx53_fv40_lcd_gpios[i], NULL);
			gpio_direction_output(mx53_fv40_lcd_gpios[i], 0);
			gpio_free(mx53_fv40_lcd_gpios[i]);
		}
	} 

	gpio_free(MX53_FV40_LCD_STANDBY);
	gpio_free(lcd_pwer_pin);
}

static struct plat_lcd_data tm050rdh05_lcd_powerdev_data = {
	.set_power = tm050rdh05_lcd_power_set,
};

static struct platform_device tm050rdh05_lcd_powerdev = {
	.name = "platform-lcd",
	.dev.platform_data = &tm050rdh05_lcd_powerdev_data,
};

static void __init fv40_add_lcd_powerdev(void)
{
	platform_device_register(&tm050rdh05_lcd_powerdev);
}

static void mxc_iim_enable_fuse(void)
{
	u32 reg;
	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;
	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}


static struct mxc_iim_platform_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

static struct android_pmem_platform_data android_pmem_data = {
	.name = "pmem_adsp",
	.size = SZ_64M,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_gpu_data = {
	.name = "pmem_gpu",
	.size = SZ_64M,
	.cached = 0,
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	.type = "tp050rcz_ts",
	.addr = 0x5C,
	.irq = gpio_to_irq(MX53_FV40_CAP_TCH_INT0),
	},
};

/* HW Initialization, if return 0, initialization is successful. */
static int mx53_fv40_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	/* Enable SATA PWR  */
	ret = gpio_request(MX53_FV40_SATA_PWR_EN, "ahci-sata-pwr");
	if (ret) {
		printk(KERN_ERR "failed to get SATA_PWR_EN: %d\n", ret);
		return ret;
	}
	gpio_direction_output(MX53_FV40_SATA_PWR_EN, 1);

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	sata_ref_clk = clk_get(NULL, "usb_phy1_clk");
	if (IS_ERR(sata_ref_clk)) {
		dev_err(dev, "no sata ref clock.\n");
		ret = PTR_ERR(sata_ref_clk);
		goto release_sata_clk;
	}
	ret = clk_enable(sata_ref_clk);
	if (ret) {
		dev_err(dev, "can't enable sata ref clock.\n");
		goto put_sata_ref_clk;
	}

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_ref_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_ref_clk:
	clk_disable(sata_ref_clk);
put_sata_ref_clk:
	clk_put(sata_ref_clk);
release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx53_fv40_sata_exit(struct device *dev)
{
	clk_disable(sata_ref_clk);
	clk_put(sata_ref_clk);

	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx53_fv40_sata_data = {
	.init = mx53_fv40_sata_init,
	.exit = mx53_fv40_sata_exit,
};

static void mx53_fv40_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX53_FV40_OTG_VBUS, 1);
	else
		gpio_set_value(MX53_FV40_OTG_VBUS, 0);
}

static void __init mx53_fv40_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX53_IO_ADDRESS(MX53_OTG_BASE_ADDR);
	ret = gpio_request(MX53_FV40_OTG_VBUS, "usb-pwr");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO FV40_OTG_VBUS: %d\n", ret);
		return;
	}
	gpio_direction_output(MX53_FV40_OTG_VBUS, 0);

	mx5_set_otghost_vbus_func(mx53_fv40_usbotg_vbus);
	mx5_usb_dr_init();
	mx5_usbh1_init();
}

static struct mxc_audio_platform_data fv40_audio_data;

static int fv40_wm8960_osc_enable(int enable)
{
	gpio_request(MX53_FV40_OSC_CKIH1_EN, "osc-ckih-en");
	gpio_direction_output(MX53_FV40_OSC_CKIH1_EN, enable ? 1 : 0);
	gpio_free(MX53_FV40_OSC_CKIH1_EN);
	return 0;
}

static int fv40_wm8960_init(void)
{
	fv40_audio_data.sysclk = 22579200;
	return fv40_wm8960_osc_enable(1);
}

static struct mxc_audio_platform_data fv40_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.osc_enable = fv40_wm8960_osc_enable,
	.init = fv40_wm8960_init,
	.hp_gpio = MX53_FV40_HEADPHONE_DEC,
	.hp_active_low = 1,
};

static struct platform_device fv40_audio_device = {
	.name = "imx-wm8960",
};

static struct imx_ssi_platform_data fv40_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data fv40_fb_data[] = {
#if 0
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1024x768M-16@60",
	.default_bpp = 16,
	.int_clk = false,
	},
#endif
	{
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "800x480-32@60",
	.default_bpp = 32,
	.int_clk = true,
	},
};

static struct imx_ipuv3_platform_data ipu_data = {
	.rev = 3,
	.csi_clk[0] = "ssi_ext1_clk",
};

static int backlight_notify(struct device *dev, int brightness)
{
	/* close lcd standby when close backlight , it's happend for charging*/
	if (0 == brightness) {
		if(power_supply_is_system_supplied()) {
			printk("close lcd standby\n");
			gpio_request(MX53_FV40_LCD_STANDBY, "lcd-standby");
			gpio_direction_output(MX53_FV40_LCD_STANDBY, 0);
			gpio_free(MX53_FV40_LCD_STANDBY);
		}
	}

	return brightness;
}

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 45454,
	.notify = backlight_notify,
};

static struct mxc_gpu_platform_data mx53_fv40_gpu_pdata __initdata = {
	.enable_mmu = 0,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SIN1,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* Souce from CKIH1 for 44.1K */
	/* Source from CCM spdif_clk (24M) for 48k and 32k
	 * It's not accurate
	 */
	.spdif_clk_48000 = 1,
	.spdif_div_44100 = 8,
	.spdif_div_48000 = 8,
	.spdif_div_32000 = 12,
	.spdif_rx_clk = 0,	/* rx clk from spdif stream */
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_dvfs_platform_data fv40_dvfs_core_data = {
	.reg_id = "cpu_vddgp",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_regulator_platform_data fv40_regulator_data = {
	.cpu_reg_id = "cpu_vddgp",
};

extern struct imx_mxc_gpu_data imx53_gpu_data;

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "pmem=");
			if (str != NULL) {
				str += 5;
				android_pmem_gpu_data.size =
						memparse(str, &str);
				if (*str == ',') {
					str++;
					android_pmem_data.size =
						memparse(str, &str);
				}
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				fv40_fb_data[i++].res_size[0] =
						memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(fv40_fb_data)) {
					str++;
					fv40_fb_data[i++].res_size[0] =
						memparse(str, &str);
				}
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				imx53_gpu_data.gmem_reserved_size =
						memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "fs_sdcard=");
			if (str != NULL) {
				str += 10;
				fs_in_sdcard = memparse(str, &str);
			}
			break;
		}
	}
}

/*
 * | Status   | SYS_ON_OFF_CTL |
 * |----------+----------------|
 * | Running  |       1        |
 * | Suspend  |       1        |
 * | PowerOff |       0        |
 */

enum fv40_state_enum {
	FV40_STATE_RUNNING,
	FV40_STATE_SUSPEND,
	FV40_STATE_POWEROFF,
};

static void fv40_pic_state_sync(enum fv40_state_enum state)
{
	gpio_request(MX53_FV40_SYS_ON_OFF_CTL, "sys-on-off-ctl");
	gpio_direction_output(MX53_FV40_SYS_ON_OFF_CTL, 1);

	switch (state) {
	case FV40_STATE_RUNNING:
		gpio_set_value(MX53_FV40_SYS_ON_OFF_CTL, 1);
		break;
	case FV40_STATE_SUSPEND:
		gpio_set_value(MX53_FV40_SYS_ON_OFF_CTL, 1);
		break;
	case FV40_STATE_POWEROFF:
		gpio_set_value(MX53_FV40_SYS_ON_OFF_CTL, 0);
		mdelay(20); // FIXME : only delay on QA1
		break;
	}
	gpio_free(MX53_FV40_SYS_ON_OFF_CTL);
}

static void mx53_fv40_power_off_prepare(void)
{
	fv40_pic_state_sync(FV40_STATE_POWEROFF);
}

static void mx53_fv40_power_off(void)
{
	/* power off by sending shutdown command to da9053*/
	da9053_power_off();
}

static int __init mx53_fv40_power_init(void)
{
	/* cpu get regulator needs to be in lateinit so that
	   regulator list gets updated for i2c da9052 regulators */
	mx5_cpu_regulator_init();

	if (machine_is_mx53_fv40()) {
		pm_power_off_prepare = mx53_fv40_power_off_prepare;
		pm_power_off = mx53_fv40_power_off;
	}

	return 0;
}
late_initcall(mx53_fv40_power_init);

extern int pcb_version;
static void __init mx53_fv40_board_init(void)
{
	int i;

	printk(">>>>>>>>>>>>>>>pcb version:%d\n", pcb_version);
	/* try handle different hardware, for unknow pcb version, use qa0
	 * config */
	/* init as QA0 Version */
	if(pcb_version ) { /* QA1 */
		wlan_reset_pin = MX53_FV40_WLAN_RESET_QA1;
		lcd_pwer_pin = MX53_FV40_LCD_PWR_EN_QA1;
	}

	mxc_iomux_v3_setup_multiple_pads(mx53_fv40_pads,
					ARRAY_SIZE(mx53_fv40_pads));
	/* Enable MX53_FV40_DCDC1V8_EN */
	gpio_request(MX53_FV40_DCDC1V8_EN, "dcdc1v8-en");
	gpio_direction_output(MX53_FV40_DCDC1V8_EN, 1);
	gpio_set_value(MX53_FV40_DCDC1V8_EN, 1);
	/* default close sdio */
	wifi_status = 1;
	sdio_set_power(1, 0);
	wifi_status = 0;

	/* Wifi initialization */
	gpio_request(wlan_reset_pin, "wifi-reset");
	gpio_direction_output(wlan_reset_pin, 0);
	/* Enable WiFi/BT Power*/
	gpio_request(wlan_pwr_pin, "bt-wifi-pwren");
	gpio_direction_output(wlan_pwr_pin, 0);
#if 0
	/* 驱动自己去打开 */
	gpio_request(MX53_FV40_LCD_STANDBY, "lcd-standby");
	gpio_direction_output(MX53_FV40_LCD_STANDBY, 1);
	gpio_free(MX53_FV40_LCD_STANDBY);
	/* lcd power on */
	gpio_request(lcd_pwer_pin, "lcd-pwr-en");
	gpio_direction_output(lcd_pwer_pin, 1);
	gpio_free(lcd_pwer_pin);

	/* Touch screen disable */
	gpio_request(MX53_FV40_CAP_TCH_FUN0, "cap-tch-reset");
	gpio_direction_output(MX53_FV40_CAP_TCH_FUN0,  0);
	gpio_free(MX53_FV40_CAP_TCH_FUN0);
#endif
#if 0
	/* first close lcd power */
	gpio_request(lcd_pwer_pin, "lcd-pwr-en");
	gpio_direction_output(lcd_pwer_pin, 0);
	gpio_free(lcd_pwer_pin);
#endif	
	gp_reg_id = fv40_regulator_data.cpu_reg_id;
	lp_reg_id = fv40_regulator_data.vcc_reg_id;

	mx53_fv40_init_uart();
	mx53_fv40_fec_reset();
	mxc_register_device(&mxc_pm_device, &fv40_pm_data);
	imx53_add_fec(&mx53_fv40_fec_data);
	imx53_add_imx2_wdt(0, NULL);
	imx53_add_srtc();
	imx53_add_imx_i2c(0, &mx53_fv40_i2c_data);
	imx53_add_imx_i2c(1, &mx53_fv40_i2c_data);
	imx53_add_imx_i2c(2, &mx53_fv40_i2c_data);

	/* Add lcd power device */
	fv40_add_lcd_powerdev();
	imx53_add_ipuv3(0, &ipu_data);
	for (i = 0; i < ARRAY_SIZE(fv40_fb_data); i++)
		imx53_add_ipuv3fb(i, &fv40_fb_data[i]);
	imx53_add_lcdif(&lcdif_data);
	if (!mxc_fuse_get_vpu_status())
		imx53_add_vpu();
	imx53_add_ldb(&ldb_data);
	imx53_add_v4l2_output(0);
	imx53_add_v4l2_capture(0);
#if 1
	char *string = saved_command_line;
	char *p =strstr(string, "androidboot.mode=charger" );
	if ( p != NULL) {
		mxc_pwm_backlight_data.dft_brightness = 0;
	}
	imx53_add_mxc_pwm(1);
	imx53_add_mxc_pwm_backlight(0, &mxc_pwm_backlight_data);
#else
	imx53_add_mxc_pwm(1);
	imx53_add_mxc_pwm_backlight(0, &mxc_pwm_backlight_data);
#endif
	gpio_request(MX53_FV40_SD1_PW, NULL);
	gpio_direction_output(MX53_FV40_SD1_PW, 0);
	mdelay(200);	
	gpio_direction_output(MX53_FV40_SD1_PW, 1);
	gpio_free(MX53_FV40_SD1_PW);
	if (fs_in_sdcard == 1) {
		imx53_add_sdhci_esdhc_imx(0, &mx53_fv40_sd1_data);
		imx53_add_sdhci_esdhc_imx(1, &mx53_fv40_sd2_data);
		imx53_add_sdhci_esdhc_imx(2, &mx53_fv40_sd3_data);
	} else {
		imx53_add_sdhci_esdhc_imx(2, &mx53_fv40_sd3_data);
		imx53_add_sdhci_esdhc_imx(1, &mx53_fv40_sd2_data);
		imx53_add_sdhci_esdhc_imx(0, &mx53_fv40_sd1_data);
	}

	imx53_add_ahci(0, &mx53_fv40_sata_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);
	mx53_fv40_init_usb();
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx53_add_asrc(&imx_asrc_data);

	imx53_add_iim(&iim_data);
	fv40_add_device_buttons();
	fv40_add_device_leds();

	mx53_fv40_init_da9052();

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	imx53_add_imx_ssi(1, &fv40_ssi_pdata);

	mxc_register_device(&fv40_audio_device, &fv40_audio_data);

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx53_add_spdif(&mxc_spdif_data);
	imx53_add_spdif_dai();
	imx53_add_spdif_audio_device();

	mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device,
				&android_pmem_gpu_data);

	/*GPU*/
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0)
		mx53_fv40_gpu_pdata.z160_revision = 1;
	else
		mx53_fv40_gpu_pdata.z160_revision = 0;

	if (!mxc_fuse_get_gpu_status())
		imx53_add_mxc_gpu(&mx53_fv40_gpu_pdata);

	/* this call required to release SCC RAM partition held by ROM
	  * during boot, even if SCC2 driver is not part of the image
	  */
	imx53_add_mxc_scc2();
	pm_i2c_init(MX53_I2C1_BASE_ADDR);

	fv40_pic_state_sync(FV40_STATE_RUNNING);

	imx53_add_dvfs_core(&fv40_dvfs_core_data);
	imx53_add_busfreq();
}

static void __init mx53_fv40_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 22579200, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(MX53_UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx53_fv40_timer = {
	.init	= mx53_fv40_timer_init,
};

#define SZ_TRIPLE_1080P	ALIGN((1920*ALIGN(1080, 128)*2*3), SZ_4K)
static void __init mx53_fv40_reserve(void)
{
	phys_addr_t phys;
	int i;

	if (imx53_gpu_data.gmem_reserved_size) {
		phys = memblock_alloc(imx53_gpu_data.gmem_reserved_size,
					   SZ_4K);
		memblock_free(phys, imx53_gpu_data.gmem_reserved_size);
		memblock_remove(phys, imx53_gpu_data.gmem_reserved_size);
		imx53_gpu_data.gmem_reserved_base = phys;
	}
#ifdef CONFIG_ANDROID_PMEM
	if (android_pmem_data.size) {
		phys = memblock_alloc(android_pmem_data.size, SZ_4K);
		memblock_free(phys, android_pmem_data.size);
		memblock_remove(phys, android_pmem_data.size);
		android_pmem_data.start = phys;
	}

	if (android_pmem_gpu_data.size) {
		phys = memblock_alloc(android_pmem_gpu_data.size, SZ_4K);
		memblock_free(phys, android_pmem_gpu_data.size);
		memblock_remove(phys, android_pmem_gpu_data.size);
		android_pmem_gpu_data.start = phys;
	}
#endif

	for (i = 0; i < ARRAY_SIZE(fv40_fb_data); i++)
		if (fv40_fb_data[i].res_size[0]) {
			/* reserve for background buffer */
			phys = memblock_alloc(fv40_fb_data[i].res_size[0],
						SZ_4K);
			memblock_free(phys, fv40_fb_data[i].res_size[0]);
			memblock_remove(phys, fv40_fb_data[i].res_size[0]);
			fv40_fb_data[i].res_base[0] = phys;

			/* reserve for overlay buffer */
			phys = memblock_alloc(SZ_TRIPLE_1080P, SZ_4K);
			memblock_free(phys, SZ_TRIPLE_1080P);
			memblock_remove(phys, SZ_TRIPLE_1080P);
			fv40_fb_data[i].res_base[1] = phys;
			fv40_fb_data[i].res_size[1] = SZ_TRIPLE_1080P;
		}
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_FV40 data structure.
 */
MACHINE_START(MX53_FV40, "Freescale iMX53 FV40 Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx53_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.timer = &mx53_fv40_timer,
	.init_machine = mx53_fv40_board_init,
	.reserve = mx53_fv40_reserve,
MACHINE_END
