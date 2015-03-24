/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2008 Toshiba Corporation
 *
 * Bus Glue for TC90416
 *
 * Based on "ohci-sa1111.c" and "ohci-au1xxx.c"
 *
 * This file is licenced under the GPL.
 */

#include <asm-mips/tx-boards/tc90416-regs.h>
#include <linux/platform_device.h>

extern int usb_disabled(void);

/*-------------------------------------------------------------------------*/

static void tc90416_start_hc(struct platform_device *dev)
{
	struct tc90416_usbhcb_reg *usbhcb = (struct tc90416_usbhcb_reg *)TC90416_USBHC_REG;
	printk(KERN_DEBUG __FILE__
	       ": starting TC90416 OHCI USB Controller\n");

	while ((__raw_readl(&usbhcb->control) & TC90416_USBHCB_CTRL_RSTSTS) == 0)
		;

	/* setup dummy read */
	__raw_writel(__raw_readl(&usbhcb->control) & ~TC90416_USBHCB_CTRL_DREN, &usbhcb->control);
	__raw_writel(0x18470000, &usbhcb->draddress);
	__raw_writel(0x00000000, &usbhcb->drtrigger0);
	__raw_writel(0x0000ffff, &usbhcb->drtriggermask0);

	__raw_writel(__raw_readl(&usbhcb->control) | TC90416_USBHCB_CTRL_DREN, &usbhcb->control);
}

static void tc90416_stop_hc(struct platform_device *dev)
{
	printk(KERN_DEBUG __FILE__
	       ": stopping TC90416 OHCI USB Controller\n");

}


/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */


/**
 * usb_hcd_tc90416_probe - initialize TC90416 based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
static int usb_hcd_tc90416_probe (const struct hc_driver *driver,
			  struct platform_device *dev)
{
	struct usb_hcd *hcd;
	int retval;

	hcd = usb_create_hcd (driver, &dev->dev, "tc90416");
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dbg("request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ioremap failed\n");
		retval = -ENOMEM;
		goto err2;
	}

	tc90416_start_hc(dev);
	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, dev->resource[1].start, IRQF_DISABLED | IRQF_SHARED);
	if (retval == 0)
		return retval;

	tc90416_stop_hc(dev);
	iounmap(hcd->regs);
 err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
 err1:
	usb_put_hcd(hcd);
	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_tc90416_remove - shutdown processing for TC90416 HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_tc90416_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_tc90416_remove (struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	tc90416_stop_hc(dev);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}

/*-------------------------------------------------------------------------*/

static int __devinit
ohci_tc90416_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		err ("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}
	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_tc90416_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"TC90416 OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_tc90416_start,
	.stop =			ohci_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_tc90416_drv_probe(struct platform_device *pdev)
{
	int ret;

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_tc90416_probe(&ohci_tc90416_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_tc90416_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_tc90416_remove(hcd, pdev);
	return 0;
}

#ifdef CONFIG_PM
static int ohci_hcd_tc90416_drv_suspend(struct platform_device *dev, pm_message_t message)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	unsigned long	flags;
	int		rc = 0;

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave (&ohci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ohci_writel(ohci, OHCI_INTR_MIE, &ohci->regs->intrdisable);
	(void)ohci_readl(ohci, &ohci->regs->intrdisable);

	/* make sure snapshot being resumed re-enumerates everything */
	if (message.event == PM_EVENT_PRETHAW)
		ohci_usb_reset(ohci);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
 bail:
	spin_unlock_irqrestore (&ohci->lock, flags);

	tc90416_stop_hc(dev);
	return rc;
}
static int ohci_hcd_tc90416_drv_resume(struct platform_device *dev)
{
	struct usb_hcd *hcd =   platform_get_drvdata(dev);

	tc90416_start_hc(dev);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
 	ohci_finish_controller_resume(hcd);

	return 0;
}
#endif

static struct platform_driver ohci_hcd_tc90416_driver = {
	.probe		= ohci_hcd_tc90416_drv_probe,
	.remove		= ohci_hcd_tc90416_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend	= ohci_hcd_tc90416_drv_suspend,
	//.suspend_late	= ohci_hcd_tc90416_drv_suspend,
	//.resume_early	= ohci_hcd_tc90416_drv_resume,
	.resume		= ohci_hcd_tc90416_drv_resume,
#endif
	.driver = {
		.name	= "tc90416-ohci",
		.owner	= THIS_MODULE,
	},
};

static int __init ohci_hcd_tc90416_init (void)
{
	dbg (DRIVER_INFO " (TC90416)");
	dbg ("block sizes: ed %d td %d",
		sizeof (struct ed), sizeof (struct td));

	return platform_driver_register(&ohci_hcd_tc90416_driver);
}

static void __exit ohci_hcd_tc90416_cleanup (void)
{
	platform_driver_unregister(&ohci_hcd_tc90416_driver);
}

module_init (ohci_hcd_tc90416_init);
module_exit (ohci_hcd_tc90416_cleanup);
