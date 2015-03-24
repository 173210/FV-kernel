/*
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 2008 Toshiba Corporation
 * (C) Copyright 2000-2004 David Brownell <dbrownell@users.sourceforge.net>
 *
 * Bus Glue for TC90416
 *
 * Based on "ehci-au1xxx.c"
 *
 * This file is licenced under the GPL.
 */

#include <asm-mips/tx-boards/tc90416-regs.h>
#include <linux/platform_device.h>

extern int usb_disabled(void);

/*-------------------------------------------------------------------------*/

static void tc90416_start_ehc(struct platform_device *dev)
{
	struct tc90416_usbhcb_reg *usbhcb = (struct tc90416_usbhcb_reg *)TC90416_USBHC_REG;

	while ((__raw_readl(&usbhcb->control) & TC90416_USBHCB_CTRL_RSTSTS) == 0)
		;

	/* setup dummy read */
	__raw_writel(__raw_readl(&usbhcb->control) & ~TC90416_USBHCB_CTRL_DREN, &usbhcb->control);
	__raw_writel(0x18470000, &usbhcb->draddress);
	__raw_writel(0x00000000, &usbhcb->drtrigger0);
	__raw_writel(0x0000ffff, &usbhcb->drtriggermask0);

	__raw_writel(__raw_readl(&usbhcb->control) | TC90416_USBHCB_CTRL_DREN, &usbhcb->control);
}

static void tc90416_stop_ehc(struct platform_device *dev)
{
}

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_ehci_tc90416_probe - initialize TC90416-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
static int usb_ehci_tc90416_probe(const struct hc_driver *driver,
			  struct usb_hcd **hcd_out, struct platform_device *dev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;

	tc90416_start_ehc(dev);

	if (dev->resource[1].flags != IORESOURCE_IRQ) {
		pr_debug("resource[1] is not IORESOURCE_IRQ");
		retval = -ENOMEM;
	}
	hcd = usb_create_hcd(driver, &dev->dev, "tc90416");
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err2;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	/* ehci_hcd_init(hcd_to_ehci(hcd)); */

	ehci->sbrn = 0x20;

	retval =
	    usb_add_hcd(hcd, dev->resource[1].start, IRQF_DISABLED | IRQF_SHARED);
	if (retval == 0)
		return retval;

	tc90416_stop_ehc(dev);
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
 * usb_ehci_hcd_tc90416_remove - shutdown processing for TC90416-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_tc90416_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_ehci_tc90416_remove(struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	ehci_shutdown(hcd); /* change to halt state for next insmod */
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	tc90416_stop_ehc(dev);
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_tc90416_hc_driver = {
	.description = hcd_name,
	.product_desc = "TC90416 EHCI",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_init,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif
};

/*-------------------------------------------------------------------------*/

static int ehci_hcd_tc90416_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd = NULL;
	int ret;

	pr_debug("In ehci_hcd_tc90416_drv_probe\n");

	if (usb_disabled())
		return -ENODEV;

	ret = usb_ehci_tc90416_probe(&ehci_tc90416_hc_driver, &hcd, pdev);
	return ret;
}

static int ehci_hcd_tc90416_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_ehci_tc90416_remove(hcd, pdev);
	return 0;
}

#ifdef CONFIG_PM
static int ehci_hcd_tc90416_drv_suspend(struct platform_device *dev, pm_message_t message)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);	
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	unsigned long flags;
	int rc = 0;
	
	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave (&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	writel (0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	/* make sure snapshot being resumed re-enumerates everything */
	if (message.event == PM_EVENT_PRETHAW) {
		ehci_halt(ehci);
		ehci_reset(ehci);
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
 bail:
	spin_unlock_irqrestore (&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	tc90416_stop_ehc(dev);
	return rc;
}

static int ehci_hcd_tc90416_drv_resume(struct platform_device *dev)
{
	struct usb_hcd *hcd =   platform_get_drvdata(dev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);

	tc90416_start_ehc(dev);
	
	// maybe restore FLADJ

	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	/* If CF is still set, we maintained PCI Vaux power.
	 * Just undo the effect of ehci_pci_suspend().
	 */
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		if (!device_may_wakeup(&hcd->self.root_hub->dev))
			mask &= ~STS_PCD;
		writel(mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);
		return 0;
	}

	ehci_dbg(ehci, "lost power, restarting\n");
	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		ehci->reclaim_ready = 1;
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	writel(ehci->command, &ehci->regs->command);
	writel(FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	hcd->state = HC_STATE_SUSPENDED;
	return 0;
}
#endif

MODULE_ALIAS("tc90416-ehci");
static struct platform_driver ehci_hcd_tc90416_driver = {
	.probe = ehci_hcd_tc90416_drv_probe,
	.remove = ehci_hcd_tc90416_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend      = ehci_hcd_tc90416_drv_suspend,
	//.suspend_late	= ehci_hcd_tc90416_drv_suspend,
	//.resume_early	= ehci_hcd_tc90416_drv_resume,
	.resume       = ehci_hcd_tc90416_drv_resume,
#endif
	.driver = {
		.name = "tc90416-ehci",
		.bus = &platform_bus_type
	}
};
