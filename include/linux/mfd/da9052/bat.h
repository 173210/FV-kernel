/*
 * da9052 BAT module declarations.
  *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_MFD_DA9052_BAT_H
#define __LINUX_MFD_DA9052_BAT_H

#include <linux/power_supply.h>

enum charger_type_enum {
	DA9052_NOCHARGER = 1,
	DA9052_USB_HUB,
	DA9052_USB_CHARGER,
	DA9052_WALL_CHARGER
};

struct da9052_bat_event_registration {
	unsigned long evbit[BITS_TO_LONGS(EVE_CNT)];
};

struct da9052_bat_hysteresis {
	int bat_volt_arr[3];
	int array_hys_batvoltage[2];
	int upper_limit;
	int lower_limit;
	int index;
	int hys_flag;
};

struct da9052_charger_device {
	struct da9052		*da9052;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work 	monitor_work;
	struct power_supply	bat;
	struct power_supply	ac;
	struct power_supply	usb;
	int			status;
	int			health;
	int			charger_type;
	int			capacity;

	int			technology;
	int			bat_capacity_limit_low;
	int			bat_capacity_full;
	int			bat_capacity_limit_high;
	int			bat_volt_cutoff;
	int			bat_with_no_resistor;
	int			charger_voltage_drop;
	int			bat_target_voltage;
	int			chg_end_current;
	int			hysteresis_window_size;
	int			chg_hysteresis_const;
	int			hysteresis_reading_interval;
	int			hysteresis_no_of_reading;
	int			vbat_first_valid_detect_iteration;
};
	

static inline  u8 bat_temp_reg_to_C(u16 value) { return (55 - value); }
static inline  u8 bat_mV_to_reg(u16 value) { return (((value-4100)/100)<<4); }
static inline  u8 bat_drop_mV_to_reg(u16 value)
		{ return (((value-100)/100)<<6); }
static inline  u16 bat_reg_to_mV(u8 value) { return ((value*100) + 4100); }
static inline  u16 bat_drop_reg_to_mV(u8 value) { return ((value*100)+100); }
static inline  u8 vch_thr_mV_to_reg(u16 value) { return ((value-3700)/100); }
static inline  u8 precharge_mA_to_reg(u8 value) { return ((value/20)<<6); }
static inline  u8 vddout_mon_mV_to_reg(u16 value)
		{ return (((value-2500)*128)/1000); }
static inline  u16 vddout_reg_to_mV(u8 value)
		{ return ((value*1000)/128)+2500; }

static inline int volt_reg_to_mV(int value)
{
	return ((value * 1000) / 512) + 2500;
}

static inline int ichg_mA_to_reg(u16 value) { return (value/4); }

static inline int ichg_reg_to_mA(u8 value)
{
	return ((value * 3900) / 1000);
}

static inline int iset_mA_to_reg(u16 iset_value)
{
	if ((70 <= iset_value) && (iset_value <= 120))
		return (iset_value-70)/10;
	else if ((400 <= iset_value) && (iset_value <= 700))
		return ((iset_value-400)/50)+6;
	else if ((900 <= iset_value) && (iset_value <= 1300))
		return ((iset_value-900)/200)+13; else return 0;
}

#define DA9052_BAT_PROFILE		0
#define SUCCESS				0
#define FAILURE				1

#define TRUE				1
#define FALSE				0

#define set_bits(value, mask)		(value | mask)
#define clear_bits(value, mask)		(value & ~(mask))


/* SSC Read or Write Error */
#define DA9052_SSC_FAIL			150

/* To enable debug output for your module, set this to 1 */
#define 	DA9052_BAT_DEBUG 	0
#define		DA9052_SSC_DEBUG	0

#undef DA9052_DEBUG

#if (DA9052_BAT_DEBUG || DA9052_SSC_DEBUG)
#define DA9052_DEBUG(fmt, args...) printk(KERN_CRIT "" fmt, ##args)
#else
#define DA9052_DEBUG(fmt, args...)
#endif

#endif /* __LINUX_MFD_DA9052_BAT_H */
