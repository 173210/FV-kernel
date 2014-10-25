/*
 * da9052-battery.c  --  Battery Driver for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/bat.h>
#include <linux/mfd/da9052/adc.h>

#define DISABLE	0
#define ENABLE	1

#if 0
#define pr_da9052(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define pr_da9052(fmt, ...)
#endif

#define DA9052_BAT_DEVICE_NAME	"da9052-bat"

static const char __initdata banner[] = KERN_INFO "DA9052 BAT, (c) \
					2009 Dialog semiconductor Ltd.\n";

static struct da9052_bat_event_registration event_status;
static struct wake_lock bat_wake_lock;
static int vbus_detect_plug = 0;

enum batt_voltage_mode {
	BATT_VOLTAGE_ACTUAL,
	BATT_VOLTAGE_TRY_ADJUST, /* 充电状态时会尝试调整电压值 */
};

static const int tbat_lookup[][2] = {
	{  2, 121}, {  3, 109}, {  4, 100}, {  5,  93},
	{  6,  88}, {  7,  83}, {  8,  79}, {  9,  75},
	{ 10,  72}, { 11,  69}, { 12,  66}, { 13,  64},
	{ 14,  61}, { 15,  59}, { 16,  57}, { 17,  56},
	{ 18,  54}, { 19,  52}, { 20,  51}, { 21,  49},
	{ 22,  48}, { 23,  47}, { 24,  46}, { 25,  44},
	{ 26,  43}, { 27,  42}, { 28,  41}, { 29,  40},
	{ 30,  39}, { 31,  38}, { 33,  37}, { 34,  36},
	{ 35,  35}, { 36,  34}, { 38,  33}, { 39,  32},
	{ 40,  31}, { 42,  30}, { 43,  29}, { 45,  28},
	{ 47,  27}, { 49,  26}, { 51,  25}, { 52,  24},
	{ 54,  23}, { 57,  22}, { 59,  21}, { 61,  20},
	{ 64,  19}, { 66,  18}, { 69,  17}, { 71,  16},
	{ 74,  15}, { 77,  14}, { 81,  13}, { 84,  12},
	{ 87,  11}, { 91,  10}, { 95,   9}, { 99,   8},
	{103,   7}, {107,   6}, {112,   5}, {117,   4},
	{122,   3}, {127,   2}, {132,   1}, {138,   0},
	{144,  -1}, {151,  -2}, {158,  -3}, {165,  -4},
	{172,  -5}, {180,  -6}, {189,  -7}, {197,  -8},
	{207,  -9}, {216, -10}, {226, -11}, {237, -12},
	{249, -13},
};

static int da9052_bat_read_volt(struct da9052_charger_device *chg,
				int *volt_mV, enum batt_voltage_mode mode);

/* Assuming that the battery being stable after five consecutive scans */
#define BATTERY_STABLE_TIMES	7
static int interval_count = 0;

static const unsigned long interval_array[] = {
	HZ * 1, HZ * 1, HZ * 1, HZ * 2, HZ * 2, HZ * 3, HZ * 5, /* 15s for debounce */
	HZ * 1, HZ * 1, HZ * 1, HZ * 1, HZ * 1,  /* 5s to check if battery is full */
	HZ * 30,  /* scan every 30s */
};

static unsigned long jf = 0;

static void reset_battery_update_interval(void)
{
	jf = jiffies;
	interval_count = 0;
}

/* 等待一段时间，电压/电流稳定下来 */
static int wait_for_debounce(void)
{
	pr_da9052("%s, %d(ms)\n", __func__, jiffies_to_msecs(jiffies - jf));
	return interval_count > BATTERY_STABLE_TIMES;
}

static unsigned long get_battery_update_interval(void)
{
	unsigned long itl;

	if (interval_count >= 0 &&
	    interval_count < ARRAY_SIZE(interval_array)) {
		itl = interval_array[interval_count];
		interval_count++;
	} else
		itl = interval_array[ARRAY_SIZE(interval_array) - 1];

	return itl;
}

static int da9052_bat_read_chg_current(struct da9052_charger_device *chg,
				       int *current_mA)
{
	int ret;
	u8 reg;

	if (chg->status == POWER_SUPPLY_STATUS_DISCHARGING) {
		*current_mA = 0;
		return 0;	/* -ENODEV: no charging supply */
	}

	ret = da9052_read(chg->da9052, DA9052_ICHGAV_REG, &reg);
	if (ret)
		return -EIO;

	*current_mA = ichg_reg_to_mA(reg & DA9052_ICHGAV_ICHGAV);

	return 0;
}

static int da9052_set_usb_ilim(struct da9052_charger_device *chg, int en)
{
	struct da9052_ssc_msg msg;

	msg.addr = DA9052_CHGBUCK_REG;
	msg.data = 0;

	da9052_lock(chg->da9052);

	if (chg->da9052->read(chg->da9052, &msg)) {
		da9052_unlock(chg->da9052);
		return DA9052_SSC_FAIL;
	}

	if (en == DISABLE)
		msg.data = clear_bits(msg.data, DA9052_CHGBUCK_CHGUSBILIM);
	else
		msg.data = set_bits(msg.data, DA9052_CHGBUCK_CHGUSBILIM);

	if (chg->da9052->write(chg->da9052, &msg)) {
		da9052_unlock(chg->da9052);
		return DA9052_SSC_FAIL;
	}

	da9052_unlock(chg->da9052);
	return 0;
}

extern int usb_detected_by_line_status(void);

static int da9052_bat_check_status(struct da9052_charger_device *chg, int *status, int keep_status)
{
	uint8_t status_a, status_b;
	bool dcinsel, dcindet;
	bool vbussel, vbusdet;
	bool dc, vbus;
	int bat_status = chg->status;

	if (da9052_read(chg->da9052, DA9052_STATUSA_REG, &status_a) ||
	    da9052_read(chg->da9052, DA9052_STATUSB_REG, &status_b))
		return -EIO;

	dcinsel = status_a & DA9052_STATUSA_DCINSEL;
	dcindet = status_a & DA9052_STATUSA_DCINDET;
	vbussel = status_a & DA9052_STATUSA_VBUSSEL;
	vbusdet = status_a & DA9052_STATUSA_VBUSDET;
	dc = dcinsel && dcindet;
	vbus = vbussel && vbusdet;

	if (dc || vbus) {
		/* 使用USB寄存器来判断充电类型是USB还是AC */
		if (usb_detected_by_line_status())
			chg->charger_type = DA9052_USB_CHARGER;
		else
			chg->charger_type = DA9052_WALL_CHARGER;

		/* 电池到达FULL状态，一直保持 */
		if (bat_status != POWER_SUPPLY_STATUS_FULL)
			bat_status = POWER_SUPPLY_STATUS_CHARGING;
	} else if (dcindet || vbusdet) {
		if (usb_detected_by_line_status())
			chg->charger_type = DA9052_USB_CHARGER;
		else
			chg->charger_type = DA9052_WALL_CHARGER;

		bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		chg->charger_type = DA9052_NOCHARGER;
		bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	/*
	 * 外接电源插入，待稳定之后：
	 *   如果i.MX53检测为AC而DA9053检测为USB。
	 *   此时DA9053认为是USB且以小电流充电。此时强制DA9053重新检测。
	 */
	if (vbus_detect_plug && wait_for_debounce()) {
		pr_da9052("%d: vbus_detect_plug %d\n", __LINE__, vbus_detect_plug);
		if ((chg->charger_type == DA9052_WALL_CHARGER) &&
		    !(status_a & DA9052_STATUSA_VDATDET)) {
			pr_da9052("%d: vbus_detect_plug %d\n", __LINE__, vbus_detect_plug);
			da9052_set_usb_ilim(chg, 0);
			msleep(1);
			da9052_set_usb_ilim(chg, 1);
			vbus_detect_plug = 0;
		} else
			vbus_detect_plug = 0;
	}

	if(!keep_status)
		chg->status = bat_status;

	if (status)
		*status = bat_status;

	pr_da9052("%s status %d, %d\n", __func__, bat_status, chg->status);
	return 0;
}

static int da9052_bat_temperature(int tbat)
{
	int index, temp = 0;

	if (tbat == 0)
		return 0;

	if (tbat <= tbat_lookup[0][0])
		return tbat_lookup[0][1];
	else if (tbat >= tbat_lookup[ARRAY_SIZE(tbat_lookup) - 1][0])
		return tbat_lookup[ARRAY_SIZE(tbat_lookup) - 1][1];

	for (index = 0; index < ARRAY_SIZE(tbat_lookup) - 1; index++) {
		if (tbat == tbat_lookup[index][0])
			temp = tbat_lookup[index][1];
		else if (tbat == tbat_lookup[index+1][0])
			temp = tbat_lookup[index+1][1];
		else if (tbat > tbat_lookup[index][0] && tbat < tbat_lookup[index+1][0])
			temp = tbat_lookup[index][1];
	}

	return temp;
}

static int da9052_adc_read_temp(struct da9052_charger_device *chg, int *temp)
{
	int ret, i;
	uint32_t t;
	u8 reg_data;

	for (t = 0, i = 0; i < 4; i++) {
		ret = da9052_read(chg->da9052, DA9052_TBATRES_REG, &reg_data);
		if (ret)
			return ret;
		t += reg_data;
	}
	*temp = t >> 2;

	return 0;
}

static int da9052_bat_read_temperature(struct da9052_charger_device *chg,
				       int *temperature)
{
	int t;
	int ret;

	ret = da9052_adc_read_temp(chg, &t);
	if (!ret)
		*temperature = da9052_bat_temperature(t);

	return ret;
}

static int da9052_bat_check_presence(struct da9052_charger_device *chg,
				     int *illegal)
{
	int bat_temp;
	int ret;

	ret = da9052_adc_read_temp(chg, &bat_temp);
	if (ret)
		return ret;

	if (bat_temp > chg->bat_with_no_resistor)
		*illegal = 1;
	else
		*illegal = 0;

	return 0;
}

#if 0
static int da9052_bat_suspend_charging(struct da9052_charger_device *chg)
{
	u8 reg_data;
	int ret;

	ret = da9052_read(chg->da9052, DA9052_INPUTCONT_REG, &reg_data);
	if (ret)
		return ret;

	/* set both Wall charger and USB charger suspend bit */
	reg_data = set_bits(reg_data, DA9052_INPUTCONT_DCINSUSP);
	reg_data = set_bits(reg_data, DA9052_INPUTCONT_VBUSSUSP);

	return da9052_write(chg->da9052, DA9052_INPUTCONT_REG, reg_data);
}

/* suspend charging of battery if illegal battey is detected */
static void detect_illegal_battery(struct da9052_charger_device *chg)
{
	int ret;
	int bat_illegal = 0;

	ret = da9052_bat_check_presence(chg, &bat_illegal);
	if (ret || bat_illegal)
		da9052_bat_suspend_charging(chg);
	return bat_illegal;
}
#endif

static const int vs_tbl_charging[][2] = {
	{ 3710, 3498 }, { 3740, 3542 }, { 3875, 3623 }, { 3892, 3634 },
	{ 3919, 3679 }, { 3949, 3707 }, { 3955, 3720 }, { 3966, 3726 },
	{ 3974, 3730 }, { 3970, 3751 }, { 3994, 3767 }, { 4003, 3783 },
	{ 4023, 3806 }, { 4054, 3833 }, { 4060, 3859 }, { 4089, 3873 },
	{ 4100, 3906 }, { 4110, 3935 }, { 4121, 3960 }, { 4130, 4000 },
	{ 4140, 4050 }, { 4163, 4074 }, { 4179, 4140 },
};

static inline int da9052_bat_voltage_adjust(int mv)
{
	int i;
	int found = 0;
	int x1, x2, y1, y2;

	if (mv <= vs_tbl_charging[0][0])
		return vs_tbl_charging[0][1];

	if (mv >= vs_tbl_charging[ARRAY_SIZE(vs_tbl_charging)-1][0])
		return vs_tbl_charging[ARRAY_SIZE(vs_tbl_charging)-1][1];

	for (i = 0; i < ARRAY_SIZE(vs_tbl_charging) - 1; i++) {
		if (mv >= vs_tbl_charging[i][0] && mv < vs_tbl_charging[i+1][0]) {
			found = 1;
			x1 = vs_tbl_charging[i][0];
			x2 = vs_tbl_charging[i+1][0];
			y1 = vs_tbl_charging[i][1];
			y2 = vs_tbl_charging[i+1][1];
			break;
		}
	}

	if (!found)	/* found一定为1 */
		return 0;

	/* 按比例在两点之间取一个值
	 *               (mv - x1)
	 * result = y1 + --------- * (y2 - y1)
	 *               (x2 - x1)
	 */
	return y1 + (((mv - x1)*(y2 - y1)) / (x2 - x1));
}

static int check_vbus_presence(struct da9052_charger_device *chg)
{
	uint8_t status_a;

	if (!da9052_read(chg->da9052, DA9052_STATUSA_REG, &status_a))
		return !!(status_a & DA9052_STATUSA_VBUSSEL);

	return 0;
}

static int da9052_bat_read_volt(struct da9052_charger_device *chg,
				int *volt_mV, enum batt_voltage_mode mode)
{
	int ret, i;
	uint32_t volt = 0;

	for (i = 0; i < (1 << 2); i++) {
		ret = da9052_manual_read(chg->da9052, DA9052_ADC_VBAT);
		if (ret < 0)
			return ret;
		volt += volt_reg_to_mV(ret);
	}
	volt >>= 2;

	if (mode == BATT_VOLTAGE_TRY_ADJUST) {
		if (check_vbus_presence(chg))
			volt = da9052_bat_voltage_adjust(volt);
	}

	*volt_mV = volt;
	return 0;
}

static void adjust_bat_capacity(struct da9052_charger_device *chg, int val)
{
	int old , new;

	old = chg->capacity;
	new = val;

	pr_da9052("%s: status %d, old %d, new %d\n", __func__, chg->status, old, new);

	/* 第一次计算电量，直接赋值并退出 */
	if (old < 0)
		goto out;

	/* 电池未稳定时允许波动，直接赋新值 */
	if (!wait_for_debounce()) {
		pr_da9052("wait_for_debounce() failed, %d\n", __LINE__);
		goto out;
	}

	/* 充电时，不允许更低的电压等级*/
	if (chg->status == POWER_SUPPLY_STATUS_CHARGING && new < old)
		new = old;
	/* 放电时，不允许更高的电压等级 */
	if (chg->status == POWER_SUPPLY_STATUS_DISCHARGING && new > old)
		new = old;

out:
	if (chg->status == POWER_SUPPLY_STATUS_CHARGING) { 
		// 如果插入USB之前电量是满的，则插入后立即亮绿灯
		if (old == 100)
			chg->status = POWER_SUPPLY_STATUS_FULL;
		else if (new == 100)
			new--;
	}
	chg->capacity = new;
}

#define DA9052_MEAN(x, y)	((x + y) / 2)
static const int vc_tbl_ref[] = { 10, 25, 40 };

static int da9052_dtermine_vc_tbl_index(int adc_temp)
{
	int i;

	if (adc_temp <= vc_tbl_ref[0])
		return 0;

	if (adc_temp > vc_tbl_ref[ARRAY_SIZE(vc_tbl_ref) - 1])
		return 0;

	for (i = 0; i < ARRAY_SIZE(vc_tbl_ref); i++) {
		if ((adc_temp > vc_tbl_ref[i]) &&
		    (adc_temp <= DA9052_MEAN(vc_tbl_ref[i], vc_tbl_ref[i + 1])))
			return i;
		if ((adc_temp > DA9052_MEAN(vc_tbl_ref[i], vc_tbl_ref[i + 1])) &&
		    (adc_temp <= vc_tbl_ref[i+1]))
			return i + 1;
	}

	return 0;
}

static int interpolated(int vbat_lower, int vbat_upper, int level_lower,
			int level_upper, int bat_voltage)
{
	int tmp;

	/*apply formula y= yk + (x - xk) * (yk+1 -yk)/(xk+1 -xk) */

	tmp = ((level_upper - level_lower) * 1000) / (vbat_upper - vbat_lower);
	tmp = level_lower + (((bat_voltage - vbat_lower) * tmp) / 1000);

	return tmp;
}

#define DA9052_VC_TBL_SZ	26

static const int vs_tbl[][DA9052_VC_TBL_SZ][2] = {
	/* For temperature 10 degree celisus */
	{
		{ 4050, 100 }, { 4023,  96 }, { 3993,  92 }, { 3955,  88 },
		{ 3929,  84 }, { 3895,  80 }, { 3862,  76 }, { 3831,  72 },
		{ 3802,  68 }, { 3771,  64 }, { 3750,  60 }, { 3731,  56 },
		{ 3714,  52 }, { 3699,  48 }, { 3685,  44 }, { 3674,  40 },
		{ 3664,  36 }, { 3654,  32 }, { 3646,  28 }, { 3638,  24 },
		{ 3629,  20 }, { 3614,  16 }, { 3591,  12 }, { 3573,   8 },
		{ 3556,   4 }, { 3500,   0 },
	},
	{
		{ 4050, 100 }, { 4023,  96 }, { 3993,  92 }, { 3955,  88 },
		{ 3929,  84 }, { 3895,  80 }, { 3862,  76 }, { 3831,  72 },
		{ 3802,  68 }, { 3771,  64 }, { 3750,  60 }, { 3731,  56 },
		{ 3714,  52 }, { 3699,  48 }, { 3685,  44 }, { 3674,  40 },
		{ 3664,  36 }, { 3654,  32 }, { 3646,  28 }, { 3638,  24 },
		{ 3629,  20 }, { 3614,  16 }, { 3591,  12 }, { 3573,   8 },
		{ 3556,   4 }, { 3500,   0 },
	},
	{
		{ 4050, 100 }, { 4023,  96 }, { 3993,  92 }, { 3955,  88 },
		{ 3929,  84 }, { 3895,  80 }, { 3862,  76 }, { 3831,  72 },
		{ 3802,  68 }, { 3771,  64 }, { 3750,  60 }, { 3731,  56 },
		{ 3714,  52 }, { 3699,  48 }, { 3685,  44 }, { 3674,  40 },
		{ 3664,  36 }, { 3654,  32 }, { 3646,  28 }, { 3638,  24 },
		{ 3629,  20 }, { 3614,  16 }, { 3591,  12 }, { 3573,   8 },
		{ 3556,   4 }, { 3500,   0 },
	},
};

static int calc_level_by_voltage(int i, int bat_voltage)
{
	int level = 0;
	int vbat_upper, vbat_lower;
	int level_upper, level_lower;
	int j;

	if (bat_voltage >= vs_tbl[i][0][0])
		return 100;
	if (bat_voltage <= vs_tbl[i][DA9052_VC_TBL_SZ - 1][0])
		return 0;

	for (j = 0; j < (DA9052_VC_TBL_SZ - 1); j++) {
		if ((bat_voltage <= vs_tbl[i][j][0]) &&
		    (bat_voltage >= vs_tbl[i][j+1][0])) {
			vbat_upper = vs_tbl[i][j][0];
			vbat_lower = vs_tbl[i][j+1][0];
			level_upper = vs_tbl[i][j][1];
			level_lower = vs_tbl[i][j+1][1];
			break;
		}
	}

	level = interpolated(vbat_lower, vbat_upper, level_lower,
			     level_upper, bat_voltage);
	return level;
}

struct filter_struct {
	int initref;
	int upper_limit;
	int lower_limit;
	int pool[1];
	int failref;
	int factor;        /* 0 ~ 100 */
};

struct filter_struct filter;
const static int filter_inited_factor = 30;

static void inline filter_factor(int factor)
{
	WARN(factor > 100, "Filter factor can't more than 100\n");
	filter.factor = factor;
}

static void filter_init(int factor)
{
	filter.initref = 0;
	filter_factor(factor);
}

static int filter_calc(int *value)
{
	int x, y;
	int offset;

	x = y = *value;

	pr_debug("\nBefore filter: filter.initref %d, value %d upper_limit %d, lower_limit %d\n",
	       filter.initref, x, filter.upper_limit, filter.lower_limit);

	/* init 5 times */
	if (filter.initref < BATTERY_STABLE_TIMES) {
		if (filter.initref++ == 0)
			filter.pool[0] = x;
		if (filter.factor < 90)
			filter.factor += 20;
		if (filter.factor > 90)
			filter.factor = 90;
	}
	/* limit */
	else if (x < filter.lower_limit || x > filter.upper_limit) {
		if (++filter.failref < 10) {
			pr_da9052("Filter: failed\n");
			return -EIO;
		}
	}

	offset = x / 100;
	filter.upper_limit = x + offset;
	filter.lower_limit = x - offset;
	filter.failref = 0;

	pr_debug("Before filter: %d: filter.pool[0] %d\n", __LINE__, filter.pool[0]);
	/* Digital C Filter, formula Yn = k*Xn-1 + (1-k)*Xn */
	y = ((filter.factor*filter.pool[0]) + ((100-filter.factor)*x)) / 100;

	filter.pool[0] = y;
	*value = y;
	pr_debug(" After filter: %d\n\n", y);
	return 0;
}

static int da9052_bat_read_capacity(struct da9052_charger_device *chg,
				    int *capacity)
{
	int adc_temp;
	int bat_voltage;
	int ret;
	int t;
	int level;

	level = chg->capacity;

	ret = da9052_bat_read_volt(chg, &bat_voltage, BATT_VOLTAGE_TRY_ADJUST);
	if (ret)
		goto out;

	ret = filter_calc(&bat_voltage);
	if (ret)
		goto out;

	ret = da9052_bat_read_temperature(chg, &adc_temp);
	if (ret)
		goto out;

	t = da9052_dtermine_vc_tbl_index(adc_temp);

	level = calc_level_by_voltage(t, bat_voltage);

out:
	adjust_bat_capacity(chg, level);
	if (capacity)
		*capacity = chg->capacity;
	return 0;
}

static int da9052_bat_check_health(struct da9052_charger_device *chg,
				   int *health)
{
	int ret;
	int bat_illegal;
	int capacity;

	ret = da9052_bat_check_presence(chg, &bat_illegal);
	if (ret)
		return ret;

	if (bat_illegal) {
		chg->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		return 0;
	}

	if (chg->health != POWER_SUPPLY_HEALTH_OVERHEAT) {
		ret = da9052_bat_read_capacity(chg, &capacity);
		if (ret)
			return ret;
		if (capacity < chg->bat_capacity_limit_low)
			chg->health = POWER_SUPPLY_HEALTH_DEAD;
		else
			chg->health = POWER_SUPPLY_HEALTH_GOOD;
	}

	*health = chg->health;
	return 0;
}

static enum power_supply_property da9052_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int da9052_bat_get_property(struct power_supply *bat,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int ret;
	int illegal;
	struct da9052_charger_device *chg =
		container_of(bat, struct da9052_charger_device, bat);

	ret = da9052_bat_check_presence(chg, &illegal);
	if (ret) {
		return ret;
	}

	if (illegal && psp != POWER_SUPPLY_PROP_PRESENT) {
		printk("battery illegal %d\n", illegal);
		return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = da9052_bat_check_status(chg, &val->intval, 1);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (chg->charger_type == DA9052_NOCHARGER) ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = da9052_bat_check_presence(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = da9052_bat_check_health(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chg->bat_target_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chg->bat_volt_cutoff;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = da9052_bat_read_volt(chg, &val->intval, BATT_VOLTAGE_TRY_ADJUST);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		/* 使用voltage_avg接口可以读到电压实际值 */
		ret = da9052_bat_read_volt(chg, &val->intval, BATT_VOLTAGE_ACTUAL);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = da9052_bat_read_chg_current(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = da9052_bat_read_capacity(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = da9052_bat_read_temperature(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = chg->technology;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static enum power_supply_property da9052_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int da9052_ac_get_property(struct power_supply *ac,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct da9052_charger_device *chg =
		container_of(ac, struct da9052_charger_device, ac);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (chg->charger_type == DA9052_WALL_CHARGER)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property da9052_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int da9052_usb_get_property(struct power_supply *usb,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct da9052_charger_device *chg =
		container_of(usb, struct da9052_charger_device, usb);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (chg->charger_type == DA9052_USB_CHARGER)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void battery_reinit(struct da9052_charger_device *chg)
{
	filter_init(filter_inited_factor);
	reset_battery_update_interval();

	// 有可能电池不在充电，此时进入休眠则会一直绿灯。
	// 要确定完电池状态是否FULL，才能进入休眠。
	if (check_vbus_presence(chg) &&
	    chg->status != POWER_SUPPLY_STATUS_FULL) {
		pr_da9052("%s, request wake_lock_timeout 20s\n", __func__);
		wake_lock_timeout(&bat_wake_lock, HZ * 20);
	}
}

static void da9052_bat_event_handler(struct da9052_eh_nb *eh_data, unsigned int event)
{
	struct da9052_charger_device *chg = eh_data->priv;

	pr_da9052("=============== %s, %d\n", __func__, eh_data->eve_type);
	switch (eh_data->eve_type) {
	case VBUS_DET_EVE:
		vbus_detect_plug = 1;
		battery_reinit(chg);
		break;
	case VBUS_REM_EVE:
		/* 如果电池满了，为了延迟这一满状态，不能做重取电池数据动作 */
		if (chg->status != POWER_SUPPLY_STATUS_FULL)
			battery_reinit(chg);
		break;
	case VDD_LOW_EVE:
		break;
	case CHG_END_EVE:
		break;
	case TBAT_EVE:
		chg->health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	default:
		return;
	}

	cancel_delayed_work(&chg->monitor_work);
	queue_delayed_work(chg->monitor_wqueue, &chg->monitor_work, HZ/10);
}

static struct da9052_eh_nb da9052_events[] = {
	{ 	/* Battery over/ under temp caused event */
		.eve_type = TBAT_EVE,
		.call_back = da9052_bat_event_handler,
	},
	{ 	/* VBUS 4.4V detection caused event */
		.eve_type = VBUS_DET_EVE,
		.call_back = da9052_bat_event_handler,
	},
	{ 	/* VBUS removal caused event */
		.eve_type = VBUS_REM_EVE,
		.call_back = da9052_bat_event_handler,
	},
	{
		.eve_type = VDD_LOW_EVE,
		.call_back = da9052_bat_event_handler,
	},
	{
		.eve_type = CHG_END_EVE,
		.call_back = da9052_bat_event_handler,
	}
};

static int da9052_bat_register_event(struct da9052_charger_device *chg)
{
	int ret, i;
	struct da9052 *da9052 = chg->da9052;
	struct da9052_eh_nb *nb;

	for (i = 0; i < ARRAY_SIZE(da9052_events); ++i) {
		nb = &da9052_events[i];

		if (test_bit(nb->eve_type, event_status.evbit))
			continue;

		ret = da9052->register_event_notifier(da9052, nb);
		if (ret)
			return -EIO;

		nb->priv = (void *)chg;
		__set_bit(nb->eve_type, event_status.evbit);
	}

	return 0;
}

static int da9052_bat_unregister_event(struct da9052_charger_device *chg)
{
	int ret, i;
	struct da9052 *da9052 = chg->da9052;
	struct da9052_eh_nb *nb;

	for (i = 0; i < ARRAY_SIZE(da9052_events); ++i) {
		nb = &da9052_events[i];
		DA9052_DEBUG("events = %d\n",nb->eve_type);

		if (test_bit(nb->eve_type, event_status.evbit)) {
			ret = da9052->unregister_event_notifier(da9052, nb);
			if (ret)
				return -EIO;

			__clear_bit(nb->eve_type, event_status.evbit);
		}
	}

	return 0;
}

static void check_bat_full(struct da9052_charger_device *chg)
{
	int current_mA;
	static int ref = 0;

	if (chg->status != POWER_SUPPLY_STATUS_CHARGING)
		return;

	if (!wait_for_debounce()) {
		pr_da9052("wait_for_debounce() failed, %d\n", __LINE__);
		return;
	}

	if (da9052_bat_read_chg_current(chg, &current_mA))
		return;

	pr_da9052("%s current_mA %d\n", __func__, current_mA);

	/* 判断电池为FULL的条件(需要同时满足)：
	 * - 充电电流<=300mA
	 * - 当前电量等级>=99 (当前电压在4050mV以上)
	 * - 连续3次满足上述条件
	 */
	if ((current_mA <= chg->chg_end_current && chg->capacity >= 99) ||
	    (current_mA <= 23 && chg->capacity >= 90)) {
		++ref;
	} else 
		ref = 0;

	if (ref > 3) {
		chg->status = POWER_SUPPLY_STATUS_FULL;
		chg->capacity = 100;
	}
}

static int da9052_update_bat_properties(struct da9052_charger_device *chg)
{
	int ret;
	int bat_status, bat_capacity;
	
	bat_status = chg->status;
	bat_capacity = chg->capacity;
	//printk("\n\n=== %d ================= bat_status %d, bat_capacity %d\n", 
	//       __LINE__, bat_status, bat_capacity);

	ret = da9052_bat_check_status(chg, NULL, 0);
	if (ret)
		return ret;

	ret = da9052_bat_read_capacity(chg, NULL);
	if (ret)
		return ret;

	check_bat_full(chg);

	//printk("=== %d ================= bat_status %d, bat_capacity %d\n\n\n", 
	//       __LINE__, chg->status, chg->capacity);
	if (chg->status != bat_status ||
	    chg->capacity != bat_capacity)
		power_supply_changed(&chg->bat);

	return 0;
}

static void da9052_bat_external_power_changed(struct power_supply *bat)
{
	struct da9052_charger_device *chg =
		container_of(bat, struct da9052_charger_device, bat);

	cancel_delayed_work(&chg->monitor_work);
	queue_delayed_work(chg->monitor_wqueue, &chg->monitor_work, HZ/10);
}

static void da9052_bat_work(struct work_struct *work)
{
	struct da9052_charger_device *chg = container_of(work,
		struct da9052_charger_device,monitor_work.work);
	unsigned long interval = get_battery_update_interval();

	pr_da9052("%s: interval %lu(s)\n", __func__, interval/HZ);
	da9052_update_bat_properties(chg);

	queue_delayed_work(chg->monitor_wqueue, &chg->monitor_work, interval);
}

static int default_battery_capacity = -1;

static int __init battery_setup(char *str)
{
	if (!strcmp(str, "100"))
		default_battery_capacity = 100;
	pr_da9052("Default battery capacity %d\n", default_battery_capacity);
	return 0;
}
__setup("capacity=", battery_setup);


static s32 __devinit da9052_bat_probe(struct platform_device *pdev)
{
	struct da9052_charger_device *chg;
	u8 reg_data;
	int ret;

	chg = kzalloc(sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->da9052 = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, chg);

	/* Ac */
	chg->ac.name			= "da9052-ac";
	chg->ac.type			= POWER_SUPPLY_TYPE_MAINS;
	chg->ac.properties 		= da9052_ac_props;
	chg->ac.num_properties 		= ARRAY_SIZE(da9052_ac_props);
	chg->ac.get_property 		= da9052_ac_get_property;

	/* Usb */
	chg->usb.name			= "da9052-usb";
	chg->usb.type			= POWER_SUPPLY_TYPE_USB;
	chg->usb.properties 		= da9052_usb_props;
	chg->usb.num_properties 		= ARRAY_SIZE(da9052_usb_props);
	chg->usb.get_property 		= da9052_usb_get_property;

	/* Battery */
	chg->bat.name			= "da9052-bat";
	chg->bat.type			= POWER_SUPPLY_TYPE_BATTERY;
	chg->bat.properties 		= da9052_bat_props;
	chg->bat.num_properties 	= ARRAY_SIZE(da9052_bat_props);
	chg->bat.get_property 		= da9052_bat_get_property;
	chg->bat.external_power_changed = da9052_bat_external_power_changed;
	chg->bat.use_for_apm 		= 1;

	chg->charger_type 		= DA9052_NOCHARGER;
	chg->status 			= POWER_SUPPLY_STATUS_UNKNOWN;
	chg->health 			= POWER_SUPPLY_HEALTH_UNKNOWN;
	chg->technology 		= POWER_SUPPLY_TECHNOLOGY_LION;
	chg->bat_with_no_resistor 	= 145; // 0
	chg->bat_capacity_limit_low 	= 4;
	chg->bat_capacity_limit_high 	= 70;
	chg->bat_capacity_full 		= 100;
	chg->bat_volt_cutoff 		= 3500;
	chg->vbat_first_valid_detect_iteration = 3;
	chg->hysteresis_window_size	= 1;
	chg->chg_hysteresis_const	= 89;
	chg->hysteresis_reading_interval= 1000;
	chg->hysteresis_no_of_reading	= 10;
	chg->capacity			= default_battery_capacity;

	wake_lock_init(&bat_wake_lock, WAKE_LOCK_SUSPEND, "da9052-battery");

	ret = da9052_read(chg->da9052, DA9052_INPUTCONT_REG, &reg_data);
	if (ret)
		goto err_charger_init;
	chg->charger_voltage_drop = bat_drop_reg_to_mV(reg_data && DA9052_CHGCONT_TCTR);

	ret = da9052_read(chg->da9052, DA9052_CHGCONT_REG, &reg_data);
	if (ret)
		goto err_charger_init;
	chg->bat_target_voltage = bat_reg_to_mV(reg_data && DA9052_CHGCONT_VCHGBAT);

	/* Set charging current detection threshold 500mA */
	ret = da9052_write(chg->da9052, DA9052_ICHGTHD_REG, 0x80);
	if (ret)
		goto err_charger_init;

	/* Set charging end point current detection threshold 100mA */
	ret = da9052_write(chg->da9052, DA9052_ICHGEND_REG, 0x19);
	if (ret)
		goto err_charger_init;

	/* Set VDDOUT_MON threshold 3.5V */
	ret = da9052_write(chg->da9052, DA9052_VDDMON_REG, 0x99);
	if (ret)
		goto err_charger_init;

	chg->chg_end_current = 300;	/* mA */

	da9052_bat_register_event(chg);
	if (ret)
		goto err_charger_init;

	// 初始电池监测数据
	battery_reinit(chg);

	ret = power_supply_register(&pdev->dev, &chg->ac);
	if (ret)
		goto err_charger_init;
	ret = power_supply_register(&pdev->dev, &chg->usb);
	if (ret)
		goto err_usb_init;
	ret = power_supply_register(&pdev->dev, &chg->bat);
	if (ret)
		goto err_battery_init;

	INIT_DELAYED_WORK(&chg->monitor_work, da9052_bat_work);
	chg->monitor_wqueue = create_singlethread_workqueue(pdev->dev.init_name);/* bus_id */
	if (!chg->monitor_wqueue) {
		ret = -ENOMEM;
		goto err_final;
	}
	queue_delayed_work(chg->monitor_wqueue, &chg->monitor_work, HZ * 1);

	return 0;

err_final:
	power_supply_unregister(&chg->bat);
err_battery_init:
	power_supply_unregister(&chg->usb);
err_usb_init:
	power_supply_unregister(&chg->ac);
err_charger_init:
	platform_set_drvdata(pdev, NULL);
	wake_lock_destroy(&bat_wake_lock);
	kfree(chg);
	return ret;
}

static int __devexit da9052_bat_remove(struct platform_device *dev)
{
	struct da9052_charger_device *chg = platform_get_drvdata(dev);

	/* unregister the events.*/
	da9052_bat_unregister_event(chg);

	cancel_delayed_work_sync(&chg->monitor_work);
	destroy_workqueue(chg->monitor_wqueue);
	wake_lock_destroy(&bat_wake_lock);

	power_supply_unregister(&chg->bat);
	power_supply_unregister(&chg->usb);
	power_supply_unregister(&chg->ac);

	return 0;
}

static int da9052_bat_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int da9052_bat_resume(struct platform_device *dev)
{
	struct da9052_charger_device *chg = platform_get_drvdata(dev);

	/* 当小机放置较长时间时，电池自然放电较多，唤醒后电量应该相差较大
	 * 唤醒后重启取限幅滤波各项参数 */
	battery_reinit(chg);
	cancel_delayed_work(&chg->monitor_work);
	queue_delayed_work(chg->monitor_wqueue, &chg->monitor_work, HZ/10);
	return 0;
}

static struct platform_driver da9052_bat_driver = {
	.probe		= da9052_bat_probe,
	.suspend	= da9052_bat_suspend,
	.resume         = da9052_bat_resume,
	.remove		= __devexit_p(da9052_bat_remove),
	.driver.name	= DA9052_BAT_DEVICE_NAME,
	.driver.owner	= THIS_MODULE,
};

static int __init da9052_bat_init(void)
{
	printk(banner);
	return platform_driver_register(&da9052_bat_driver);
}

static void __exit da9052_bat_exit(void)
{
	// To remove printk("DA9052: Unregistering BAT device.\n");
	platform_driver_unregister(&da9052_bat_driver);
}

module_init(da9052_bat_init);
module_exit(da9052_bat_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd");
MODULE_DESCRIPTION("DA9052 BAT Device Driver");
MODULE_LICENSE("GPL");
