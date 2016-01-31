#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/display-sys.h>
#include <linux/interrupt.h>
#include "rk30_hdmi.h"
#include "rk30_hdmi_hw.h"

static int hdmi_get_enable(struct rk_display_device *device)
{
	struct hdmi *hdmi = device->priv_data;
	int enable;
	
	mutex_lock(&hdmi->enable_mutex);
	enable = hdmi->enable;
	mutex_unlock(&hdmi->enable_mutex);
	
	return enable;
}

static int hdmi_set_enable(struct rk_display_device *device, int enable)
{
//	struct hdmi *hdmi = device->priv_data;
	
	mutex_lock(&hdmi->enable_mutex);
	if(hdmi->enable == enable) {
		mutex_unlock(&hdmi->enable_mutex);
		return 0;
	}
	hdmi->enable = enable;
	
	if(hdmi->suspend ) {
		mutex_unlock(&hdmi->enable_mutex);
		return 0;
	}
	
	if(enable == 0) {
		disable_irq(hdmi->irq);
		mutex_unlock(&hdmi->enable_mutex);
		hdmi->command = HDMI_CONFIG_ENABLE;
		queue_delayed_work(hdmi->workqueue, &hdmi->delay_work, 0);
	}
	else {
		enable_irq(hdmi->irq);
		mutex_unlock(&hdmi->enable_mutex);
	}
	return 0;
}

static int hdmi_get_status(struct rk_display_device *device)
{
	struct hdmi *hdmi = device->priv_data;
	if(hdmi->hotplug == HDMI_HPD_ACTIVED)
		return 1;
	else
		return 0;
}

static int hdmi_get_modelist(struct rk_display_device *device, struct list_head **modelist)
{
	struct hdmi *hdmi = device->priv_data;
//	if(!hdmi->hotplug)
//		return -1;
	*modelist = &hdmi->edid.modelist;
	return 0;
}

static int hdmi_set_mode(struct rk_display_device *device, struct fb_videomode *mode)
{
	struct hdmi *hdmi = device->priv_data;
	int vic = hdmi_videomode_to_vic(mode);
	
	if(!hdmi->hotplug)
		return -1;
	hdmi->autoconfig = HDMI_DISABLE;
	if(vic && hdmi->vic != vic)
	{
		hdmi->vic = vic;
		hdmi->command = HDMI_CONFIG_VIDEO;
		init_completion(&hdmi->complete);
		hdmi->wait = 1;
		queue_delayed_work(hdmi->workqueue, &hdmi->delay_work, 0);
		wait_for_completion_interruptible_timeout(&hdmi->complete,
								msecs_to_jiffies(10000));
	}
	return 0;
}

static int hdmi_get_mode(struct rk_display_device *device, struct fb_videomode *mode)
{
	struct hdmi *hdmi = device->priv_data;
	struct fb_videomode *vmode;
	
//	if(!hdmi->hotplug)
//		return -1;
		
	vmode = (struct fb_videomode*) hdmi_vic_to_videomode(hdmi->vic);
	if(unlikely(vmode == NULL))
		return -1;
	*mode = *vmode;
	return 0;
}

static int hdmi_set_scale(struct rk_display_device *device, int direction, int value)
{
	struct hdmi *hdmi = device->priv_data;
	
	if(!hdmi || value < 0 || value > 100)
		return -1;
			
	if(direction == DISPLAY_SCALE_X)
		hdmi->xscale = value;
	else if(direction == DISPLAY_SCALE_Y)
		hdmi->yscale = value;
	else
		return -1;
//	rk_fb_disp_scale(hdmi->xscale, hdmi->yscale, HDMI_SOURCE_DEFAULT);
	return 0;
}

static int hdmi_get_scale(struct rk_display_device *device, int direction)
{
	struct hdmi *hdmi = device->priv_data;
	
	if(!hdmi)
		return -1;
		
	if(direction == DISPLAY_SCALE_X)
		return hdmi->xscale;
	else if(direction == DISPLAY_SCALE_Y)
		return hdmi->yscale;
	else
		return -1;
}

static struct rk_display_ops hdmi_display_ops = {
	.setenable = hdmi_set_enable,
	.getenable = hdmi_get_enable,
	.getstatus = hdmi_get_status,
	.getmodelist = hdmi_get_modelist,
	.setmode = hdmi_set_mode,
	.getmode = hdmi_get_mode,
	.setscale = hdmi_set_scale,
	.getscale = hdmi_get_scale,
};

static int hdmi_display_probe(struct rk_display_device *device, void *devdata)
{
	device->owner = THIS_MODULE;
	strcpy(device->type, "HDMI");
	device->priority = DISPLAY_PRIORITY_HDMI;
	device->name = "hdmi-soc";
	device->property = DISPLAY_MAIN;
	device->priv_data = devdata;
	device->ops = &hdmi_display_ops;
	return 1;
}

static struct rk_display_driver display_hdmi = {
	.probe = hdmi_display_probe,
};

static ssize_t colour_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = snprintf(buf, PAGE_SIZE, "%d\n", hdmi->colour_mode);
	buf[strlen(buf) + 1] = '\0';
	return ret;
}

static ssize_t colour_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	long int mode = 0;
	int ret = kstrtol(buf, 10, &mode);
	if(ret == 0)
	{
		if(mode != HDMI_COLOUR_MODE_LIMITED && mode != HDMI_COLOUR_MODE_FULL)
			mode = HDMI_COLOUR_MODE_LIMITED;
			
		if(!hdmi->hotplug)
			return count;
		
		if(hdmi->colour_mode == mode)
		{
			hdmi_dbg(hdmi->dev, "new colour mode matches old, nothing to do\n");
			return count;
		}
			
		// set the new colour mode
		hdmi->colour_mode = mode;

		// issue the command to the task handler to apply the new setting
		hdmi->command = HDMI_CONFIG_COLOR;
		init_completion(&hdmi->complete);
		hdmi->wait = 1;
		queue_delayed_work(hdmi->workqueue, &hdmi->delay_work, 0);
		wait_for_completion_interruptible_timeout(&hdmi->complete, msecs_to_jiffies(10000));
	}
	return count;
}

static ssize_t edid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t edid_blk[HDMI_EDID_BLOCK_SIZE];
	mutex_lock(&hdmi->edid_mutex);
	ret = rk30_hdmi_read_edid(hdmi->edid_block, edid_blk);
	mutex_unlock(&hdmi->edid_mutex);
	if(ret != 0)
	{
		hdmi_dbg(hdmi->dev, "failed to read EDID: %d\n", ret);
		return 0;
	}
	memcpy(buf, edid_blk, HDMI_EDID_BLOCK_SIZE);
	return HDMI_EDID_BLOCK_SIZE;
}

static ssize_t edid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	long int block = 0;
	int ret = kstrtol(buf, 10, &block);
	if(ret == 0)
	{
		mutex_lock(&hdmi->edid_mutex);
		hdmi_dbg(hdmi->dev, "set current EDID block: %d\n", block);
		hdmi->edid_block = block;
		mutex_unlock(&hdmi->edid_mutex);
		return count;
	}
	return 0;
}

static ssize_t vsync_show_event(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	ssize_t ret = 0;

	spin_lock_irqsave(&hdmi->vsync_lock, flags);
	if (hdmi->vsync_wait_cnt == 0)
		INIT_COMPLETION(hdmi->vsync_comp);
	hdmi->vsync_wait_cnt++;
	spin_unlock_irqrestore(&hdmi->vsync_lock, flags);

	ret = wait_for_completion_interruptible(&hdmi->vsync_comp);
	if(ret < 0)
	{
		hdmi_dbg(hdmi->dev, "wait_for_completion_interruptible failed: %d\n", ret);
		return ret;
	}
/*	
	{
		// tested - this is usually small, around 50us
		ktime_t nowtime = ktime_get();
		hdmi_dbg(hdmi->dev, "jitter = %dus\n", (int)(ktime_to_ns(nowtime) - ktime_to_ns(hdmi->vsync_time)) / 1000);
	}
*/	
	ret = snprintf(buf, PAGE_SIZE, "VSYNC=%llu", ktime_to_ns(hdmi->vsync_time));
	buf[strlen(buf) + 1] = '\0';
	return ret;
}

static DEVICE_ATTR(vsync_event, S_IRUGO, vsync_show_event, NULL);
static DEVICE_ATTR(edid, S_IRUGO | S_IWUGO, edid_show, edid_store);
static DEVICE_ATTR(colour_mode, S_IRUGO | S_IWUGO, colour_mode_show, colour_mode_store);

struct rk_display_device* hdmi_register_display_sysfs(struct hdmi *hdmi, struct device *parent)
{
	struct rk_display_device *dev = rk_display_device_register(&display_hdmi, parent, hdmi);
	
	// Add vsync event, edid probe and colour mode config to sysfs - retrofreak
	if(sysfs_create_file(&dev->dev->kobj, (const struct attribute *)&dev_attr_vsync_event))
		hdmi_dbg(hdmi->dev, "failed to add vsync_event");
	if(sysfs_create_file(&dev->dev->kobj, (const struct attribute *)&dev_attr_edid))
		hdmi_dbg(hdmi->dev, "failed to add EDID access");
	if(sysfs_create_file(&dev->dev->kobj, (const struct attribute *)&dev_attr_colour_mode))
		hdmi_dbg(hdmi->dev, "failed to add EDID access");
	
	return dev;
}

void hdmi_unregister_display_sysfs(struct hdmi *hdmi)
{
	if(hdmi->ddev)
		rk_display_device_unregister(hdmi->ddev);
}
