/*  Copyright (C) MOXA Inc. All rights reserved.
 *
 *      This software is distributed under the terms of the
 *          MOXA License.  See the file COPYING-MOXA for details.
 *
 *     File name: moxa_irigb.c
 *
 *	  Device file:
 *	  	mknod /dev/IRIG-B c 10 $minor
 *
 *	v1.0	08/26/2014 Jared Wu	New release for DA-820-LX IRIG-B
 *	v1.1	11/04/2014 Jared Wu	Suport Redhat Enterprise 5.6, linux-2.6.18
 *	v1.2	05/05/2015 Jared Wu	Fix the got too many interrupts while IRIG-B transist to off-line status.
 *	v1.3	18/10/2019 Elvis Yao	Fix coding style
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include "moxa_irigb.h"

#define DRIVER_VERSION		"1.3"

struct irigb_struct {
	unsigned long phyaddr;
	unsigned long phylen;
	void *vaddr;
	unsigned int irq;
	unsigned char di_state_keep;		/* current di state */
	unsigned char do_state_keep;		/* current do state */
	struct pci_dev *pdev;			/* pci device */
	struct miscdevice irigb_misc;		/* struct miscdevice */
	u32 TimeStatus;
	spinlock_t counter_lock;		/* lock for counter state */
} ;

/* Define the function prototype for IRIG-B */
int irigb_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
void irigb_remove(struct pci_dev *pdev);

int irigb_open (struct inode *inode, struct file *fp);
long irigb_ioctl (struct file *fp, unsigned int cmd, unsigned long arg);
ssize_t irigb_read(struct file *fp, char *buffer, size_t length, loff_t *offset);
int irigb_release (struct inode *inode, struct file *fp);

/* Defined the golbal variables */
/* This variable is for IRIG_B */
struct irigb_struct IRIG_B;

static struct file_operations irigb_fops = {
	.owner = THIS_MODULE,
	.open = irigb_open,
	.unlocked_ioctl = irigb_ioctl,
	.read = irigb_read,
	.release = irigb_release,
};

static struct pci_device_id irigb_pcibrds[] = {
	{PCI_VENDOR_ID_MOXA, MX_IRIGB_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (kernel_ulong_t *)&IRIG_B},
	{0}
};

static struct pci_driver irigb_driver = {
	.name = MX_IRIGB_NAME,
	.id_table = irigb_pcibrds,
	.probe = irigb_probe,
	.remove = irigb_remove
};

int irigb_open(struct inode *inode, struct file *fp)
{
	return 0;
}

ssize_t irigb_read(struct file *fp, char *buffer, size_t length, loff_t *offset)
{
	return 0;
}

long irigb_ioctl (struct file *fp, unsigned int cmd, unsigned long arg) {
	long retval = 0;
	struct reg_val_pair_struct reg_val_pair;
	struct reg_bit_pair_struct reg_bit_pair;
	u32 *reg = IRIG_B.vaddr;
	int i;
	u32 val;

	switch (cmd) {
	case IOCTL_GET_REGISTER:
		retval = copy_from_user(&reg_val_pair, (struct reg_val_pair_struct *)arg, sizeof(struct reg_val_pair_struct));
		if (retval < 0) {
			printk(KERN_INFO"irigb_ioctl(): IOCTL_GET_REGISTER copy_from_user() fail\n");
			return retval;
		}
		if (reg_val_pair.count == 0) {
			printk(KERN_INFO"irigb_ioctl(): IOCTL_GET_REGISTER reg_val_pair.count is 0\n");
			retval = -EINVAL;
			return retval;
		}

		for (i = 0; i < reg_val_pair.count; i++) {
			reg_val_pair.val[i] = (u32) readl(reg + reg_val_pair.addr[i]);
		}

		retval = copy_to_user((struct reg_val_pair_struct *)arg, &reg_val_pair, sizeof(struct reg_val_pair_struct));
		if (retval < 0) {
			printk(KERN_INFO"irigb_ioctl(): IOCTL_GET_REGISTER copy_to_user() fail\n");
			return retval;
		}
		break;
	case IOCTL_SET_REGISTER:
		retval = copy_from_user(&reg_val_pair, (struct reg_val_pair_struct *)arg, sizeof(struct reg_val_pair_struct));
		if (retval < 0) {
			printk(KERN_INFO"irigb_ioctl(): IOCTL_SET_REGISTER copy_from_user() fail\n");
			return retval;
		}

		if (reg_val_pair.count == 0) {
			printk(KERN_INFO"irigb_ioctl(): IOCTL_SET_REGISTER reg_val_pair.count is 0\n");
			retval = -EINVAL;
			return retval;
		}
		for (i = 0; i < reg_val_pair.count; i++) {
			writel(reg_val_pair.val[i], reg + reg_val_pair.addr[i]);
		}
		break;
	case IOCTL_SETCLR_REGISTER_BIT:
		retval = copy_from_user(&reg_bit_pair, (struct reg_bit_pair_struct *)arg, sizeof(struct reg_bit_pair_struct));
		if (retval < 0) {
			printk(KERN_INFO"irigb_ioctl(): IOCTL_SETCLR_REGISTER_BIT copy_from_user() fail\n");
			return retval;
		}
		val = (u32) readl(reg + reg_bit_pair.addr);
		val &= (~reg_bit_pair.clear_bit);
		val |= (reg_bit_pair.set_bit);
		writel(val, reg + reg_bit_pair.addr);
		break;
	case IOCTL_GET_TIMESRC_STATUS:
		retval = copy_to_user((u32 *)arg, &IRIG_B.TimeStatus, sizeof(u32));
		if (retval < 0) {
			printk(KERN_INFO"irigb_ioctl(): IOCTL_GET_TIMESRC_STATUS copy_to_user() fail\n");
			return retval;
		}
		break;
	default:
		printk("ioctl:error\n");
		retval = -EINVAL;
		return retval ;
	}

	return retval;
}

int irigb_release(struct inode *inode, struct file *fp)
{
	return 0;
}

static irqreturn_t irigb_interrupt(int irq, void *dev_id)
{
	u32 intMsk;
	u32 intSts;
	u32 maskReg;
	struct irigb_struct *pIRIG_B = (struct irigb_struct *) dev_id;
	u32 *reg = pIRIG_B->vaddr;

	intMsk = ~(readl(reg+INTMSK));

	if (intMsk == 0) {
		return IRQ_NONE;
	}

	intSts = readl(reg+INTSTS);
	if ((intMsk & intSts) == 0) {
		return IRQ_NONE;
	}

	maskReg = INTSTS_BIT_IRIG0DE_OFF | INTSTS_BIT_IRIG0DE_FRMERR | INTSTS_BIT_IRIG0DE_PARERR | INTSTS_BIT_IRIG0DE_DONE;
	if (0 != (intSts & maskReg)) {
		pIRIG_B->TimeStatus &= (~maskReg);
		pIRIG_B->TimeStatus |= (intSts & maskReg);

		/* mask INTSTS_BIT_IRIG0DE_OFF to avoid interrupt every 20ms */
		if (intSts & maskReg & INTSTS_BIT_IRIG0DE_OFF)
			intMsk &= ~INTSTS_BIT_IRIG0DE_OFF;
		else
			intMsk |= INTSTS_BIT_IRIG0DE_OFF;
	}

	maskReg = INTSTS_BIT_IRIG1DE_OFF|INTSTS_BIT_IRIG1DE_FRMERR|INTSTS_BIT_IRIG1DE_PARERR|INTSTS_BIT_IRIG1DE_DONE;

	if (0 != (intSts & maskReg)) {
		pIRIG_B->TimeStatus &= (~maskReg);
		pIRIG_B->TimeStatus |= (intSts & maskReg);

		/* mask INTSTS_BIT_IRIG1DE_OFF to avoid interrupt every 20ms */
		if (intSts & maskReg & INTSTS_BIT_IRIG1DE_OFF)
			intMsk &= (~INTSTS_BIT_IRIG1DE_OFF);
		else
			intMsk |= INTSTS_BIT_IRIG1DE_OFF;
	}

	maskReg = INTSTS_BIT_PPSDE_TIMEOUT | INTSTS_BIT_PPSDE_DONE;

	if (0 != (intSts & maskReg)) {
		pIRIG_B->TimeStatus &= ~maskReg;
		pIRIG_B->TimeStatus |= (intSts&maskReg);

		/* mask INTSTS_BIT_PPSDE_TIMEOUT to avoid interrupt every 20ms */
		if (intSts & maskReg & INTSTS_BIT_PPSDE_TIMEOUT)
			intMsk &= (~INTSTS_BIT_PPSDE_TIMEOUT);
		else
			intMsk |= INTSTS_BIT_PPSDE_TIMEOUT;
	}
#if 0	/* Jared: remove to prevent got too many interrupts while IRIG-B got off-line */
	intMsk = readl(reg+INTMSK);
#endif
	writel(~intMsk, reg+INTMSK);

	return IRQ_HANDLED;
}

int irigb_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int retval = 0;
	u32 intMsk = 0xFFFFFFFF;
	struct resource *r;
	struct irigb_struct *pIRIG_B = &IRIG_B;
	u32 *reg;

	printk(KERN_INFO"Found device %x:%x\n", pdev->vendor, pdev->device);

	pci_set_drvdata(pdev, &IRIG_B);

	pIRIG_B->pdev = pdev;

	pIRIG_B->phyaddr = pci_resource_start(pdev, 0);
	pIRIG_B->phylen = pci_resource_len(pdev, 0);

	r = request_mem_region(pIRIG_B->phyaddr, pIRIG_B->phylen, MX_IRIGB_NAME);
	if (r == NULL) {
		printk("Moxa %s request_mem_region() fail !\n", MX_IRIGB_NAME);
		retval = -ENOMEM;
		goto cleanup_probe0;
	}

	pIRIG_B->vaddr = ioremap(pIRIG_B->phyaddr, pIRIG_B->phylen);
	reg = pIRIG_B->vaddr;

	if (pci_enable_device(pdev) < 0) {
		printk("pci_enable_device() fail !\n");
		goto cleanup_probe1;
	}

	writel(SYSCON_BIT_RESET, reg+SYSCON); /* Reset this chip */

	/* The IRQ of IRIG-B device */
	pIRIG_B->irq = pdev->irq;
	printk(KERN_INFO"request_irq():%d\n", pIRIG_B->irq);

	retval = request_irq(pIRIG_B->irq, irigb_interrupt, IRQF_SHARED, MX_IRIGB_NAME, (void *)&IRIG_B);
	if (retval < 0) {
		printk("request_irq(), pIRIG_B->irq:%d, fail !\n", pIRIG_B->irq);
		goto cleanup_probe2;
	}


#ifdef DEBUG	/* Just used for debugging */
	unsigned int *reg = pIRIG_B->vaddr;

	/* writel(SYSCON_BIT_RESET, reg+SYSCON ); */ /* Reset this chip */
	printk("reg:%x\n", reg+DEVICEID);
	printk("device id pIRIG_B->vaddr+DEVICEID:%x=%x\n", reg+DEVICEID, readl(reg+DEVICEID));
	printk("date code reg+DATECODE:%x=%x\n", reg+DATECODE, readl(reg+DATECODE));
	printk("date code reg+PORTDAT:%x=%x\n", reg+PORTDAT, readl(reg+PORTDAT));
	printk("INTMASK:%x\n", readl(reg+INTMSK));
	printk("RTCDATA0:%x\n", readl(reg+RTCDAT0));
	printk("RTCDATA1:%x!\n", readl(reg+RTCDAT1));
	printk("RTCDATA2:%x!\n", readl(reg+RTCDAT2));
	printk("RTCDATA3:%x!\n", readl(reg+RTCDAT3));
#endif

	/* Enable Interrupt */
	intMsk &= ~(INTSTS_BIT_IRIG0DE_OFF | \
			INTSTS_BIT_IRIG0DE_FRMERR | \
			INTSTS_BIT_IRIG0DE_PARERR | \
			INTSTS_BIT_IRIG0DE_DONE | \
			INTSTS_BIT_IRIG1DE_OFF | \
			INTSTS_BIT_IRIG1DE_FRMERR | \
			INTSTS_BIT_IRIG1DE_PARERR | \
			INTSTS_BIT_IRIG1DE_DONE | \
			INTSTS_BIT_PPSDE_TIMEOUT | \
			INTSTS_BIT_PPSDE_DONE);

	writel(intMsk, reg + INTMSK);

	pIRIG_B->irigb_misc.minor = MX_IRIGB_MINOR;
	pIRIG_B->irigb_misc.fops = &irigb_fops;
	pIRIG_B->irigb_misc.name = MX_IRIGB_NAME;

	if (misc_register(&pIRIG_B->irigb_misc) < 0) {
		printk("misc_register() fail\n");
		goto cleanup_probe3;
	}

	return 0;

cleanup_probe3:
	/* Mask all the interrupts */
	writel(~0x0, reg+INTMSK);
	/* Free IRQ */
	free_irq(pIRIG_B->irq, &IRIG_B);
cleanup_probe2:
	pci_disable_device(pdev);
cleanup_probe1:
	release_mem_region(pIRIG_B->phyaddr, pIRIG_B->phylen);
	iounmap(pIRIG_B->vaddr);
cleanup_probe0:
	return retval;
}

void irigb_remove(struct pci_dev *pdev)
{
	struct irigb_struct *pIRIG_B;

	printk(KERN_INFO"Remove device %x:%x\n", pdev->vendor, pdev->device);

	pIRIG_B = pci_get_drvdata(pdev);

	/* Mask all the interrupts */
	writel(~0x0, pIRIG_B->vaddr+INTMSK);
	mdelay(10);
	/* Free IRQ */
	free_irq(pIRIG_B->irq, &IRIG_B);

	pci_disable_device(pdev);

	release_mem_region(pIRIG_B->phyaddr, pIRIG_B->phylen);
	iounmap(pIRIG_B->vaddr);

	misc_deregister(&pIRIG_B->irigb_misc);
}

static int __init irigb_init(void)
{
	int retval = 0;

	printk(KERN_INFO"%s Driver version %s\n", MX_IRIGB_NAME, DRIVER_VERSION);

	retval = pci_register_driver(&irigb_driver);
	if (retval < 0) {
		printk(KERN_ERR "Can't register pci driver\n");
		return -ENODEV;
	}

	return 0;
}

void __exit irigb_exit(void)
{
	printk(KERN_INFO"%s removed\n", MX_IRIGB_NAME);
	pci_unregister_driver(&irigb_driver);
}

module_init(irigb_init);
module_exit(irigb_exit);

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("jared.wu@moxa.com");
MODULE_DESCRIPTION("IRIG-B module device driver");

