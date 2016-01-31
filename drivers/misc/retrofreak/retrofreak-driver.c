#include "retrofreak-driver.h"

DEFINE_MUTEX(init_lock);
static int dev_major;
static dev_t device_handle;
static struct class *device_class;

void retrofreak_hw_powerctrl(int enable);
unsigned int rk30_get_databus(void);
void rk30_set_databus(unsigned long data);

static int file_read(char *file_path, char *buf);
static int file_write(char *file_path, char *buf);
static void sleep_early_resume(struct early_suspend *h);
static void sleep_early_suspend(struct early_suspend *h);

static int get_pcba_revision(void)
{
	int raw_ver =	(gpio_get_value(PCBA_VER_GPIO2) << 2) |
					(gpio_get_value(PCBA_VER_GPIO1) << 1) |
					(gpio_get_value(PCBA_VER_GPIO0) << 0);
	return raw_ver;
}

long retrofreak_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	u8 cmd_buf[1024];
	unsigned int *cmd32 = (unsigned int *)cmd_buf;
	int rv = -EINVAL, port;
	
//	D_PRINTF("ioctl: %X\n", cmd);
	
	// if init lock is still held, then the hwman init thread has not completed - we must wait for it
	if(mutex_is_locked(&init_lock))
	{
		mutex_lock(&init_lock);
		mutex_unlock(&init_lock);
	}

	switch(cmd)
	{
		case IOCTL_DRIVER_VERSION:
			rv = RETRO_VERSION_TAG | DRIVER_VER;
			break;
		case IOCTL_PCBA_VERSION:
			cmd32[0] = get_pcba_revision();
			cmd32[1] = 0;
			cmd32[2] = 0;
			if(copy_to_user((void *)arg, cmd_buf, 3*sizeof(int)))
				BAIL(-1);
			rv = 0;
			break;
		default:
			D_PRINTF("unknown ioctl: %X\n", cmd);
			BAIL(-1);
	}
	
end:	
	return rv;
}
EXPORT_SYMBOL(retrofreak_ioctl);

void retrofreak_hw_powerctrl(int enable)
{
	D_PRINTF("retrofreak_hw_powerctrl: %d\n", enable);
	gpio_direction_output(POWER_CTRL_GPIO, 1);
	if(enable)
	{
		gpio_set_value(POWER_CTRL_GPIO, POWER_CTRL_ON);
		gpio_set_value(HOST_DRV_GPIO, POWER_CTRL_ON);
	}
	else
	{
		gpio_set_value(POWER_CTRL_GPIO, POWER_CTRL_OFF);
		gpio_set_value(HOST_DRV_GPIO, POWER_CTRL_OFF);
	}
}

static struct early_suspend early_suspend_handler = {
	.suspend = sleep_early_suspend,
	.resume = sleep_early_resume,
};

static struct file_operations fops = {
	.unlocked_ioctl = retrofreak_ioctl,
};

static int __init retrofreak_module_init(void)
{	
	int i, rv;

	// hold init lock to prevent user mode performing any operations until our
	// hardware init thread has completed
	mutex_lock(&init_lock);

	gpio_request(POWER_CTRL_GPIO, NULL);
	gpio_request(HOST_DRV_GPIO, NULL);
	gpio_request(OTG_DRV_GPIO, NULL);
	gpio_request(OTG_DISABLE_GPIO, NULL);
	gpio_request(PCBA_VER_GPIO0, NULL);
	gpio_request(PCBA_VER_GPIO1, NULL);
	gpio_request(PCBA_VER_GPIO2, NULL);
	
	gpio_direction_output(OTG_DRV_GPIO, 1);
	gpio_direction_output(OTG_DISABLE_GPIO, 1);

	retrofreak_hw_powerctrl(1);

	register_early_suspend(&early_suspend_handler);

	// setup PCB-A version checking
	gpio_direction_input(PCBA_VER_GPIO0);
	gpio_direction_input(PCBA_VER_GPIO1);
	gpio_direction_input(PCBA_VER_GPIO2);
	gpio_pull_updown(PCBA_VER_GPIO0, 1);
	gpio_pull_updown(PCBA_VER_GPIO1, 1);
	gpio_pull_updown(PCBA_VER_GPIO2, 1);

	int pcbaVer = get_pcba_revision();
	printk(TAG "PCB-A revision: %X\n", pcbaVer);
	switch(pcbaVer)
	{
	case PCBA_VER_2:
		mutex_unlock(&init_lock);
		break;
	default:
		printk(TAG "unsupported PCB-A revision!\n");
		return -1;
	}

	dev_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (dev_major < 0)
	{
		D_PRINTF("Registering the character device failed with %d\n", dev_major);
		return -1;
	}

	device_handle = MKDEV(dev_major, 0);
    device_class = class_create(THIS_MODULE, DEVICE_FILE);
    if(device_class == NULL)
    {
		unregister_chrdev(dev_major, DEVICE_NAME);
		D_PRINTF("Class creation failed\n");
		return -1;
    }

    if(device_create(device_class, NULL, device_handle, NULL, DEVICE_FILE) == NULL)
    {
		class_destroy(device_class);
		unregister_chrdev(dev_major, DEVICE_NAME);
		D_PRINTF("Device creation failed\n");
		return -1;
    }

    printk(TAG "driver initialised, major = %d\n", dev_major);
 	return 0;
}

static void __exit retrofreak_module_exit(void)
{
	D_PRINTF("driver exit\n");
	device_destroy(device_class, device_handle);
	class_destroy(device_class);	
	unregister_chrdev(dev_major, DEVICE_NAME);
	unregister_early_suspend(&early_suspend_handler);
}  

// Power management stuff

#define FILE_CPU_MAX_FREQ 	"/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"
#define NORMAL_CPU_FREQ		(1608 * 1000)
#define SLEEP_CPU_FREQ		(252 * 1000)

static int file_read(char *file_path, char *buf)
{
	struct file *file = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

	D_PRINTF("file_read %s\n", file_path);
	file = filp_open(file_path, O_RDONLY, 0);

	if (IS_ERR(file)) {
		D_PRINTF("%s error open file  %s\n", __func__, file_path);
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	file->f_op->read(file, (char *)buf, 32, &offset);
	sscanf(buf, "%s", buf);

	set_fs(old_fs);
	filp_close(file, NULL);  
	file = NULL;
	return 0;

}

static int file_write(char *file_path, char *buf)
{
	struct file *file = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

	D_PRINTF("file_write %s %s size = %d\n", file_path, buf, strlen(buf));
	file = filp_open(file_path, O_RDWR, 0);

	if (IS_ERR(file)) {
		D_PRINTF("%s error open file  %s\n", __func__, file_path);
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	file->f_op->write(file, (char *)buf, strlen(buf), &offset);

	set_fs(old_fs);
	filp_close(file, NULL);  
	file = NULL;
	return 0;

}

static void set_cpu_freq(int freq)
{
	char buf[32];
	snprintf(buf, 32, "%d", freq);
	file_write(FILE_CPU_MAX_FREQ, buf);
}

static void sleep_early_suspend(struct early_suspend *h)
{
	set_cpu_freq(SLEEP_CPU_FREQ);
	retrofreak_hw_powerctrl(0);
}

static void sleep_early_resume(struct early_suspend *h)
{
	retrofreak_hw_powerctrl(1);
	set_cpu_freq(NORMAL_CPU_FREQ);
}

module_init(retrofreak_module_init);
module_exit(retrofreak_module_exit);
MODULE_LICENSE("GPL");
