/*
 * HIFIFO: Harmon Instruments PCI Express to FIFO
 * Copyright (C) 2014 Harmon Instruments, LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/mm.h>

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysctl.h>
#include <linux/types.h>

#define hififo_min(x,y) ((x) > (y) ? (y) : (x))

#define DEVICE_NAME "hififo"
#define DRIVER_NAME "hififo_axi"

#define HIFIFO_IOC_MAGIC 'f'
#define IOC_INFO 0x10
#define IOC_RUN 0x11
#define IOC_TIMEOUT 0x13
#define IOC_BUILD 0x15

#define REG_INTERRUPT 0
#define REG_ID 1
#define REG_BUILD 2
#define REG_RESET 3
#define REG_RESET_SET 3
#define REG_RESET_CLEAR 4

#define writelle(data, addr) (writel(cpu_to_le32(data), addr))
#define readlle(addr) (le32_to_cpu(readl(addr)))
#define writereg(s, data, addr) (writelle(data, &s->pio_reg_base[(addr)]))
#define readreg(s, addr) (readlle(&s->pio_reg_base[(addr)]))

struct hififo_dev {
	struct class *class;
	struct cdev cdev;
	dev_t devt;
	u32 *pio_reg_base;
	int idreg;
	u32 build;
	int irq;
	wait_queue_head_t queue;
	struct mutex sem;
	dma_addr_t ring_dma_addr;
	u32 *local_base;
	spinlock_t lock_open;
	int timeout;
};

static int hififo_release(struct inode *inode, struct file *filp)
{
	struct hififo_dev *drvdata = filp->private_data;
	//hififo_set_abort(fifo, 1);
	udelay(100); /* allow any pending DMA to complete */
	printk(KERN_INFO DEVICE_NAME " close\n");
	//spin_unlock(&drvdata->lock_open);
	//drvdata->is_open = 0;
	return 0;
}

static int hififo_open(struct inode *inode, struct file *filp)
{
	struct hififo_dev *drvdata;
	drvdata = container_of(inode->i_cdev, struct hififo_dev, cdev);
	//if (!spin_trylock(&drvdata->lock_open))
	//	return -EBUSY;
	filp->private_data = drvdata;
	printk(KERN_INFO DEVICE_NAME " open\n");
	//hififo_set_abort(fifo, 1);
	udelay(100);
	drvdata->timeout = (250 * HZ) / 1000; /* default of 250 ms */
	wmb();
	/* clear the abort bit on this FIFO in hardware */
	//hififo_set_abort(fifo, 0);
	udelay(100);
	return 0;
}

static long hififo_ioctl (struct file *file,
			  unsigned int command,
			  unsigned long arg)
{
	struct hififo_dev *drvdata = file->private_data;
	long status = mutex_lock_interruptible(&drvdata->sem);
	int fifo;
	int rc;
        if (status)
                return status;
	status = -ENOTTY;

	if(command == _IO(HIFIFO_IOC_MAGIC, IOC_RUN)){
		fifo = arg >> 28;
		// start
		//printk(KERN_INFO DEVICE_NAME ": preread %d\n", readreg(drvdata, fifo+8));
		writereg(drvdata, arg, fifo+8);
		// wait
		//printk(KERN_INFO DEVICE_NAME ": running %d on fifo %d\n", readreg(drvdata, fifo+8), fifo);
		if(fifo != 0) {
			rc = wait_event_interruptible_timeout(drvdata->queue,
							      readreg(drvdata, fifo+8) == 0,
							      drvdata->timeout);
			if(rc < 1) {
				printk(KERN_INFO DEVICE_NAME ": R8 %d\n", readreg(drvdata, 8));
				printk(KERN_INFO DEVICE_NAME ": R9 %d\n", readreg(drvdata, 9));
				printk(KERN_INFO DEVICE_NAME ": timeout on FIFO %d\n", fifo);
				status = -ETIME;
			}
		}
		status = 0;
	}

	if(command == _IO(HIFIFO_IOC_MAGIC, IOC_TIMEOUT)){
		/* 1 extra jiffy so we round up rather than down */
		drvdata->timeout = 1 + (arg * HZ) / 1000;
		status = 0;
	}
	if(command == _IO(HIFIFO_IOC_MAGIC, IOC_BUILD))
		status = (long) drvdata->build;
	mutex_unlock(&drvdata->sem);
	return status;
}

static struct file_operations fops_fpc = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hififo_ioctl,
	.open = hififo_open,
	.release = hififo_release
};

static irqreturn_t hififo_interrupt(int irq, void *data)
{
	struct hififo_dev *drvdata = data;
	readreg(drvdata, REG_INTERRUPT);
	//printk(KERN_INFO DEVICE_NAME " interrupt: sr = %x\n", sr);
	wake_up_all(&drvdata->queue);
	/*for(i=0; i<8; i++){
		if(!(sr & (1<<i)))
			continue;
		if(drvdata->fifo[i] == NULL)
			continue;

			}*/
	return IRQ_HANDLED;
	//spin_lock(&drvdata->lock);
	//spin_unlock(&drvdata->lock);
}

static int hififo_probe(struct platform_device *pdev)
{
	int i;
	int rc;
	dev_t devt = 0;
	struct hififo_dev *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	printk(KERN_INFO DEVICE_NAME ": Found Harmon Instruments AXI interface\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->pio_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->pio_reg_base))
		return PTR_ERR(drvdata->pio_reg_base);

	init_waitqueue_head(&drvdata->queue);
	drvdata->irq = platform_get_irq(pdev, 0);

	printk("irq = %d\n", drvdata->irq);
	rc = devm_request_irq(&pdev->dev,
			      drvdata->irq,
			      &hififo_interrupt,
			      0,
			      dev_name(&pdev->dev),
			      drvdata);
	if (rc) {
		dev_err(&pdev->dev, "No IRQ available");
		return rc;
	}

	platform_set_drvdata(pdev, drvdata);

	dev_info(&pdev->dev, "ioremap %pa to %p\n",
		 &res->start, drvdata->pio_reg_base);

	for(i=0; i<16; i++)
		printk("bar0[%d] = %.8x\n", i, (u32) readreg(drvdata, i));

	drvdata->idreg = readreg(drvdata, REG_ID);
	drvdata->build = readreg(drvdata, REG_BUILD);
	printk(KERN_INFO DEVICE_NAME " FPGA build = 0x%.8X\n", drvdata->build);
	/* reset it */
	writereg(drvdata, 0xFFFF, REG_RESET_SET);
	udelay(10); /* wait for completion of anything that was running */
	writereg(drvdata, 0xFFFF, REG_RESET_CLEAR);

	rc = alloc_chrdev_region(&devt, 0, 1, DRIVER_NAME);
	if (rc < 0) {
		printk(KERN_ERR DEVICE_NAME ": alloc_chrdev_region() failed\n");
		return rc;
	}
	drvdata->devt = devt;
	drvdata->class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(drvdata->class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		goto fail6;
	}
	printk(KERN_INFO DEVICE_NAME " idreg = 0x%.8X\n", drvdata->idreg);
	cdev_init(&drvdata->cdev, &fops_fpc); /* returns void */
	drvdata->cdev.ops = &fops_fpc;
	drvdata->cdev.owner = THIS_MODULE;
	rc = cdev_add (&drvdata->cdev, devt, 1);
	if (rc){
		printk(KERN_NOTICE DEVICE_NAME ": Error %d adding cdev\n", rc);
		return rc; // should
	}
	device_create(drvdata->class, NULL, devt, NULL, "hififo_0");
	//if(rc)
	//	goto fail7;
	spin_lock_init(&drvdata->lock_open);
	mutex_init(&drvdata->sem);
	/* enable interrupts */
	writereg(drvdata, 0xFFFF, REG_INTERRUPT);
	return 0;
//fail8:
//	device_destroy(drvdata->class, drvdata->devt);
//fail7:
//	class_destroy(drvdata->class);
fail6:
	unregister_chrdev_region(devt, 1);
//fail5:
	return rc;
}


static int hififo_remove(struct platform_device *pdev){
	struct hififo_dev *drvdata = platform_get_drvdata(pdev);
	if (!drvdata)
		return -ENODEV;
	writereg(drvdata, 0xFF, REG_RESET_SET);
	unregister_chrdev_region(drvdata->devt, 1);
	device_destroy(drvdata->class, drvdata->devt);
	class_destroy(drvdata->class);
	cdev_del(&drvdata->cdev);
	return 0;
}

static struct of_device_id hififo_of_match[] = {
	{ .compatible = "harmoninstruments,hififo_axi-1.0", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, hififo_of_match);

/* Driver Structure */
static struct platform_driver hififo_platform_driver = {
	.probe = hififo_probe,
	.remove = hififo_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = hififo_of_match,
	},
};

module_platform_driver(hififo_platform_driver);

MODULE_AUTHOR("Harmon Instruments, LLC");
MODULE_DESCRIPTION("Harmon Instruments FIFO (AXI)");
MODULE_LICENSE("GPL");
