/*
 * ltt-heartbeat.c
 *
 * (C) Copyright	2006 -
 * 		Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * notes : heartbeat timer cannot be used for early tracing in the boot process,
 * as it depends on timer interrupts.
 *
 * The timer needs to be only on one CPU to support hotplug.
 * We have the choice between schedule_delayed_work_on and an IPI to get each
 * CPU to write the heartbeat. IPI have been chosen because it is considered
 * faster than passing through the timer to get the work scheduled on all the
 * CPUs.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/ltt-core.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/timex.h>
#include <linux/bitops.h>
#include <linux/marker.h>
#include <linux/ltt-facilities.h>
#include <linux/ltt-tracer.h>

#define BITS_OF_COMPACT_DATA		11

/* Expected minimum probe duration, in cycles */
#define MIN_PROBE_DURATION		400
/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define EXPECTED_INTERRUPT_LATENCY	30

static struct timer_list heartbeat_timer;
static unsigned int precalc_heartbeat_expire = 0;

int ltt_compact_data_shift = 0;
EXPORT_SYMBOL_GPL(ltt_compact_data_shift);

int ltt_tsc_lsb_truncate = 0;	/* Number of LSB removed from compact tsc */
EXPORT_SYMBOL_GPL(ltt_tsc_lsb_truncate);
int ltt_tscbits = 0;		/* 32 - tsc_lsb_truncate - tsc_msb_cutoff */
EXPORT_SYMBOL_GPL(ltt_tscbits);

#ifdef CONFIG_LTT_SYNTHETIC_TSC
/* For architectures with 32 bits TSC */
static struct synthetic_tsc_struct {
	u32 tsc[2][2];	/* a pair of 2 32 bits. [0] is the MSB, [1] is LSB */
	unsigned int index;	/* Index of the current synth. tsc. */
} ____cacheline_aligned synthetic_tsc[NR_CPUS];

static void ltt_heartbeat_init_synthetic_cpu_tsc(int cpu)
{
	synthetic_tsc[cpu].tsc[0][0] = 0;	
	synthetic_tsc[cpu].tsc[0][1] = 0;	
	synthetic_tsc[cpu].tsc[1][0] = 0;
	synthetic_tsc[cpu].tsc[1][1] = 0;
	synthetic_tsc[cpu].index = 0;
}

/* Called from one CPU, before any tracing starts, to init each structure */
static void ltt_heartbeat_init_synthetic_tsc(void)
{
	int cpu;
	for_each_possible_cpu(cpu)
		ltt_heartbeat_init_synthetic_cpu_tsc(cpu);
	smp_wmb();
}

/* Called from heartbeat IPI : either in interrupt or process context */
static void ltt_heartbeat_update_synthetic_tsc(void)
{
	struct synthetic_tsc_struct *cpu_synth;
	u32 tsc;

	preempt_disable();
	cpu_synth = &synthetic_tsc[smp_processor_id()];
	tsc = (u32)get_cycles();	/* We deal with a 32 LSB TSC */

	if (tsc < cpu_synth->tsc[cpu_synth->index][1]) {
		unsigned int new_index = cpu_synth->index ? 0 : 1; /* 0 <-> 1 */
		/* Overflow */
		/* Non atomic update of the non current synthetic TSC, followed
		 * by an atomic index change. There is no write concurrency,
		 * so the index read/write does not need to be atomic. */
		cpu_synth->tsc[new_index][1] = tsc; /* LSB update */
		cpu_synth->tsc[new_index][0] =
			cpu_synth->tsc[cpu_synth->index][0]+1; /* MSB update */
		cpu_synth->index = new_index;	/* atomic change of index */
	} else {
		/* No overflow : we can simply update the 32 LSB of the current
		 * synthetic TSC as it's an atomic write. */
		cpu_synth->tsc[cpu_synth->index][1] = tsc;
	}
	preempt_enable();
}

/* Called from buffer switch : in _any_ context (even NMI) */
u64 ltt_heartbeat_read_synthetic_tsc(void)
{
	struct synthetic_tsc_struct *cpu_synth;
	u64 ret;
	unsigned int index;
	u32 tsc;

	preempt_disable();
	cpu_synth = &synthetic_tsc[smp_processor_id()];
	index = cpu_synth->index; /* atomic read */
	tsc = (u32)get_cycles();	/* We deal with a 32 LSB TSC */

	if (tsc < cpu_synth->tsc[index][1]) {
		/* Overflow */
		ret = ((u64)(cpu_synth->tsc[index][0]+1) << 32) | ((u64)tsc);
	} else {
		/* no overflow */
		ret = ((u64)cpu_synth->tsc[index][0] << 32) | ((u64)tsc);
	}
	preempt_enable();
	return ret;
}
EXPORT_SYMBOL_GPL(ltt_heartbeat_read_synthetic_tsc);
#endif //CONFIG_LTT_SYNTHETIC_TSC


static void heartbeat_ipi(void *info)
{
#ifdef CONFIG_LTT_SYNTHETIC_TSC
	ltt_heartbeat_update_synthetic_tsc();
#endif //CONFIG_LTT_SYNTHETIC_TSC

#ifdef CONFIG_LTT_HEARTBEAT_EVENT
	/* Log a heartbeat event for each trace, each tracefile */
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL),
		core_time_heartbeat, 
		MARK_NOARGS,
		GET_CHANNEL_INDEX(facilities));
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL),
		core_time_heartbeat,
		MARK_NOARGS,
		GET_CHANNEL_INDEX(interrupts));
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL),
		core_time_heartbeat,
		MARK_NOARGS,
		GET_CHANNEL_INDEX(processes));
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL),
		core_time_heartbeat,
		MARK_NOARGS,
		GET_CHANNEL_INDEX(modules));
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL),
		core_time_heartbeat,
		MARK_NOARGS,
		GET_CHANNEL_INDEX(cpu));
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL),
		core_time_heartbeat,
		MARK_NOARGS,
		GET_CHANNEL_INDEX(network));
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_COMPACT),
		compact_time_heartbeat,
		MARK_NOARGS);
#endif //CONFIG_LTT_HEARTBEAT_EVENT
}

static void heartbeat_ipi_full(void *info)
{
#ifdef CONFIG_LTT_SYNTHETIC_TSC
	ltt_heartbeat_update_synthetic_tsc();
#endif //CONFIG_LTT_SYNTHETIC_TSC

#ifdef CONFIG_LTT_HEARTBEAT_EVENT
	/* Log a heartbeat event for each trace, each tracefile */
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL)
		| ltt_flag_mask(LTT_FLAG_FORCE),
		core_time_heartbeat_full,
		"%8b",
		GET_CHANNEL_INDEX(facilities),
		ltt_get_timestamp64());
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL)
		| ltt_flag_mask(LTT_FLAG_FORCE),
		core_time_heartbeat_full,
		"%8b",
		GET_CHANNEL_INDEX(interrupts),
		ltt_get_timestamp64());
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL)
		| ltt_flag_mask(LTT_FLAG_FORCE),
		core_time_heartbeat_full,
		"%8b",
		GET_CHANNEL_INDEX(processes),
		ltt_get_timestamp64());
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL)
		| ltt_flag_mask(LTT_FLAG_FORCE),
		core_time_heartbeat_full,
		"%8b",
		GET_CHANNEL_INDEX(modules),
		ltt_get_timestamp64());
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL)
		| ltt_flag_mask(LTT_FLAG_FORCE),
		core_time_heartbeat_full,
		"%8b",
		GET_CHANNEL_INDEX(cpu),
		ltt_get_timestamp64());
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_CHANNEL)
		| ltt_flag_mask(LTT_FLAG_FORCE),
		core_time_heartbeat_full,
		"%8b",
		GET_CHANNEL_INDEX(network),
		ltt_get_timestamp64());
	_MARK(_MF_DEFAULT | ltt_flag_mask(LTT_FLAG_COMPACT)
		| ltt_flag_mask(LTT_FLAG_FORCE),
		compact_time_heartbeat_full,
		"%8b",
		ltt_get_timestamp64());
#endif //CONFIG_LTT_HEARTBEAT_EVENT
}

/* Write the full TSC in the traces. To be called when we cannot assume that the heartbeat events
 * will keep a trace buffer synchronized. (missing timer events, tracing starts, tracing restarts)
 */
void ltt_write_full_tsc(void)
{
	on_each_cpu(heartbeat_ipi_full, NULL, 1, 0);
}
EXPORT_SYMBOL_GPL(ltt_write_full_tsc);

/* We need to be in process context to do an IPI */
static void heartbeat_work(struct work_struct *work)
{
	on_each_cpu(heartbeat_ipi, NULL, 1, 0);
}
static DECLARE_WORK(hb_work, heartbeat_work);

/**
 * heartbeat_timer : - Timer function generating hearbeat.
 * @data: unused
 *
 * Guarantees at least 1 execution of heartbeat before low word of TSC wraps.
 */
static void heartbeat_timer_fct(unsigned long data)
{
	PREPARE_WORK(&hb_work, heartbeat_work);
	schedule_work(&hb_work);
	
	mod_timer(&heartbeat_timer, jiffies + precalc_heartbeat_expire);
}


/**
 * init_heartbeat_timer: - Start timer generating hearbeat events.
 */
static void init_heartbeat_timer(void)
{
	int tsc_msb_cutoff;
	int data_bits = BITS_OF_COMPACT_DATA;
	int max_tsc_msb_cutoff;
	unsigned long mask;

	if (loops_per_jiffy > 0) {
		printk(KERN_DEBUG "LTT : ltt-heartbeat init\n");
		printk("Requested number of bits %d\n", data_bits);

		ltt_tsc_lsb_truncate = max(0,
			(int)get_count_order(MIN_PROBE_DURATION)-1);
		max_tsc_msb_cutoff =
			32 - 1 - get_count_order(((1UL << 1)
			+ (EXPECTED_INTERRUPT_LATENCY*HZ/1000)
			+ LTT_PERCPU_TIMER_INTERVAL + 1)
			* (loops_per_jiffy << 1));
		printk("Available number of bits %d\n",
			ltt_tsc_lsb_truncate + max_tsc_msb_cutoff);
		if (ltt_tsc_lsb_truncate + max_tsc_msb_cutoff < 
			data_bits) {
			printk("Number of bits truncated to %d\n",
				ltt_tsc_lsb_truncate + max_tsc_msb_cutoff);
			data_bits = ltt_tsc_lsb_truncate + max_tsc_msb_cutoff;
		}
			
		tsc_msb_cutoff = data_bits - ltt_tsc_lsb_truncate;

		if (tsc_msb_cutoff > 0)
			mask = (1UL<<(32-tsc_msb_cutoff))-1;
		else
			mask = 0xFFFFFFFFUL;
		precalc_heartbeat_expire =
			(mask/(loops_per_jiffy << 1)
				- 1 - LTT_PERCPU_TIMER_INTERVAL
				- (EXPECTED_INTERRUPT_LATENCY*HZ/1000)) >> 1;
		WARN_ON(precalc_heartbeat_expire == 0);
		printk("Heartbeat timer will fire each %u jiffies.\n",
			precalc_heartbeat_expire);

		//precalc_heartbeat_expire = ( 0xFFFFFFFFUL/(loops_per_jiffy << 1)
		//	- 1 - LTT_PERCPU_TIMER_INTERVAL) >> 1;

		//tsc_msb_cutoff = 32 - 1 -
		//	get_count_order(( (EXPECTED_INTERRUPT_LATENCY
		//		+ (precalc_heartbeat_expire * 1000 / HZ))
		//		* cpu_khz ));
		ltt_tscbits = 32 - ltt_tsc_lsb_truncate - tsc_msb_cutoff;
		printk("Compact TSC init : truncate %d lsb, cutoff %d msb.\n",
			ltt_tsc_lsb_truncate, tsc_msb_cutoff);
	} else
		printk(KERN_WARNING
			"LTT: no tsc for heartbeat timer "
			"- continuing without one \n");
}

/* ltt_init_compact_facility reserves the number of bits to identify the event
 * numbers in the compact headers. It must be called every time the compact
 * facility is changed. */
void ltt_init_compact_facility(void)
{
	if (loops_per_jiffy > 0) {
		ltt_compact_data_shift =
			get_count_order(ltt_compact_facility_num_events)
						+ ltt_tscbits;
		printk("Data shifted from %d bits\n", ltt_compact_data_shift);

		printk("%d bits used for event IDs, %d available for data.\n",
			get_count_order(ltt_compact_facility_num_events),
			32 - ltt_compact_data_shift);
	} else
		printk(KERN_WARNING
			"LTT: no tsc for heartbeat timer "
			"- continuing without one \n");
}
EXPORT_SYMBOL_GPL(ltt_init_compact_facility);


static void start_heartbeat_timer(void)
{
	if (precalc_heartbeat_expire > 0) {
		printk(KERN_DEBUG "LTT : ltt-heartbeat start\n");

		init_timer(&heartbeat_timer);
		heartbeat_timer.function = heartbeat_timer_fct;
		heartbeat_timer.expires = jiffies + precalc_heartbeat_expire;
		add_timer(&heartbeat_timer);
	} else
		printk(KERN_WARNING
			"LTT: no tsc for heartbeat timer "
			"- continuing without one \n");
}

/**
 * stop_heartbeat_timer: - Stop timer generating hearbeat events.
 */
static void stop_heartbeat_timer(void)
{
 	if (loops_per_jiffy > 0) {
		printk(KERN_DEBUG "LTT : ltt-heartbeat stop\n");
		del_timer(&heartbeat_timer);
	}
}

#ifdef CONFIG_LTT_SYNTHETIC_TSC
/**
 * 	heartbeat_hotcpu_callback - CPU hotplug callback
 * 	@nb: notifier block
 * 	@action: hotplug action to take
 * 	@hcpu: CPU number
 *	
 *	Sets the new CPU's current synthetic TSC to the same value as the
 *	currently running CPU.
 *
 * 	Returns the success/failure of the operation. (NOTIFY_OK, NOTIFY_BAD)
 */
static int __cpuinit heartbeat_hotcpu_callback(struct notifier_block *nb,
				unsigned long action,
				void *hcpu)
{
	unsigned int hotcpu = (unsigned long)hcpu;
	struct synthetic_tsc_struct *cpu_synth;
	u64 local_count;

	switch(action) {
	case CPU_UP_PREPARE:
		ltt_heartbeat_init_synthetic_cpu_tsc(hotcpu);
		cpu_synth = &synthetic_tsc[hotcpu];
		local_count = ltt_heartbeat_read_synthetic_tsc();
		cpu_synth->tsc[0][1] = (u32)local_count; /* LSB */
		cpu_synth->tsc[0][0] = (u32)(local_count >> 32); /* MSB */
		smp_wmb();
		break;
	case CPU_ONLINE:
		/* FIXME : heartbeat events are currently broken with CPU
		 * hotplug : events can be recorded before heartbeat, heartbeat
		 * too far from trace start and are broken with trace
		 * stop/start as well.
		 */
		/* As we are preemptible, make sure it runs on the right cpu */
		smp_call_function_single(hotcpu, heartbeat_ipi, NULL, 1, 0);
		break;
	}
	return NOTIFY_OK;
}
#endif //CONFIG_LTT_SYNTHETIC_TSC

int ltt_heartbeat_trigger(enum ltt_heartbeat_functor_msg msg)
{
	printk(KERN_DEBUG "LTT : ltt-heartbeat trigger\n");
	switch (msg) {
		case LTT_HEARTBEAT_START:
			start_heartbeat_timer();
			break;
		case LTT_HEARTBEAT_STOP:
			stop_heartbeat_timer();
			break;
	}
	return 0;
}

EXPORT_SYMBOL_GPL(ltt_heartbeat_trigger);

static int __init ltt_heartbeat_init(void)
{
	printk(KERN_INFO "LTT : ltt-heartbeat init\n");
#ifdef CONFIG_LTT_SYNTHETIC_TSC
	/* higher priority than relay */
	hotcpu_notifier(heartbeat_hotcpu_callback, 1);
	ltt_heartbeat_init_synthetic_tsc();
#endif //CONFIG_LTT_SYNTHETIC_TSC
	init_heartbeat_timer();
	return 0;
}

__initcall(ltt_heartbeat_init);
