/*
 * linux/arch/mips/tx-boards/generic/pm.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/init.h>
#include <linux/pm.h>

#if !defined(CONFIG_TOSHIBA_TC90411) && !defined(CONFIG_TOSHIBA_TC90412)
static int tx_pm_enter(suspend_state_t state)
{
	/* empty pm_enter for suspend/resume debugging */
	return 0;
}

static struct pm_ops tx_pm_ops = {
	.enter		= tx_pm_enter,
};

static int __init tx_pm_init(void)
{
	if (!pm_ops)
		pm_set_ops(&tx_pm_ops);
	return 0;
}

late_initcall(tx_pm_init);
#endif
