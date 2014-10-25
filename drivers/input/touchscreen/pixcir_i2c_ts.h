//if your TP has button please define this
#define BUTTON

static int attb_read_val(void);
static void pixcir_init(void);
static void pixcir_reset(void);

#define X_MAX 799
#define Y_MAX 479




#define MX53_FV40_CAP_TCH_FUN0		(5*32 + 6)	/* GPIO_6_6 */
#define MX53_FV40_CAP_TCH_INT0		(2*32 + 31)	/* GPIO_3_31 */
#include <mach/gpio.h>
#define ATTB		MX53_FV40_CAP_TCH_INT0
#define get_attb_value	gpio_get_value
#define RESETPIN_CFG	gpio_request(MX53_FV40_CAP_TCH_FUN0,"tp-reset")
#define RESETPIN_SET0	gpio_direction_output(MX53_FV40_CAP_TCH_FUN0, 0)
#define RESETPIN_SET1	gpio_direction_output(MX53_FV40_CAP_TCH_FUN0, 1)

extern int pcb_version;
static int attb_read_val(void)
{
	return get_attb_value(ATTB);
}

#define QA2	2
static void pixcir_io_suspend(void)
{
	RESETPIN_CFG;
	if(pcb_version > QA2){ /* QA2.1为新touchpanel芯片 */
		RESETPIN_SET0;
	}
	else {
		RESETPIN_SET0;
	}
}
static void pixcir_reset(void)
{
	RESETPIN_CFG;
	if(pcb_version > QA2){ /* QA2.1为新touchpanel芯片 */
		RESETPIN_SET1;
		mdelay(10);
		RESETPIN_SET0;
	} else {
		RESETPIN_SET0;
		mdelay(100);
		RESETPIN_SET1;
		mdelay(100);
	}
}

static void pixcir_init(void)
{
	pixcir_reset();
}


