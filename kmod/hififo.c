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
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/mm.h>

#define hififo_min(x,y) ((x) > (y) ? (y) : (x))

#define VENDOR_ID 0x10EE
#define DEVICE_ID 0x7024
#define DEVICE_NAME "hififo"

#define HIFIFO_IOC_MAGIC 'f'
#define IOC_INFO 0x10
#define IOC_TIMEOUT 0x13
#define IOC_BUILD 0x15

#define MAX_FIFOS 8

static struct pci_device_id hififo_pci_table[] = {
  {VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0 ,0},
  {0,}
};

#define BUFFER_SIZE (4 << 20)
#define BUFFER_MASK (BUFFER_SIZE - 1)

#define REG_INTERRUPT 0
#define REG_ID 1
#define REG_BUILD 2
#define REG_RESET 3
#define REG_RESET_SET 3
#define REG_RESET_CLEAR 4

#define writeqle(data, addr) (writeq(cpu_to_le64(data), addr))
#define readlle(addr) (le32_to_cpu(readl(addr)))
#define writereg(s, data, addr) (writeqle(data, &s->pio_reg_base[(addr)]))
#define readreg(s, addr) (readlle(&s->pio_reg_base[(addr)]))

#define IS_TO_PC(fifo) (((fifo)->n > (MAX_FIFOS/2)) ? 1 : 0)
#define DMA_DIRECTION(fifo) (IS_TO_PC(fifo) ? \
			     PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE)

static struct class *hififo_class;
static int hififo_count; /* track the number of cards found */

struct hififo_fifo {
	dma_addr_t ring_dma_addr;
	u64 *ring;
	u64 *local_base;
	struct cdev cdev;
	struct mutex sem;
	u32 p_hw, p_sw;
	u32 bytes_available;
	wait_queue_head_t queue;
	spinlock_t lock_open;
	int n; /* fifo number */
	int timeout;
	u32 build;
};

struct hififo_dev {
	struct hififo_fifo * fifo[MAX_FIFOS];
	u64 *pio_reg_base;
	int major;
	int nfifos;
	int idreg;
	u32 build;
};

static inline void hififo_set_match(struct hififo_fifo *fifo, u32 v) {
	writeqle(1 | v, fifo->local_base); /* 1 is the command (3 lsbs) */
}

static inline void hififo_set_stop(struct hififo_fifo *fifo, u32 v) {
	writeqle(2 | v, fifo->local_base); /* 2 is the command (3 lsbs) */
}

/* Set the hardware base address for this FIFO */
static inline void hififo_set_addr(struct hififo_fifo *fifo, dma_addr_t addr) {
	writeqle(3 | addr, fifo->local_base); /* 3 is the command (3 lsbs) */
}

static inline void hififo_set_abort(struct hififo_fifo *fifo, int enabled) {
	/* 4 is the command (3 lsbs), bit 8 is the abort enable bit */
	writeqle(4 | (enabled ? (1<<8) : 0), fifo->local_base);
}

static int hififo_release(struct inode *inode, struct file *filp)
{
	struct hififo_fifo *fifo = filp->private_data;
	hififo_set_abort(fifo, 1);
	udelay(100); /* allow any pending DMA to complete */
	module_put(THIS_MODULE); /* decrement the usage count */
	printk(KERN_INFO DEVICE_NAME " %d: close\n", fifo->n);
	spin_unlock(&fifo->lock_open);
	return 0;
}

static int hififo_open(struct inode *inode, struct file *filp)
{
	struct hififo_fifo *fifo = container_of(inode->i_cdev,
						struct hififo_fifo,
						cdev);
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
	if (!spin_trylock(&fifo->lock_open))
		return -EBUSY;
	filp->private_data = fifo;
	printk(KERN_INFO DEVICE_NAME " %d: open\n", fifo->n);
	printk(KERN_INFO DEVICE_NAME " alloc %llx, %llx\n", (u64) fifo->ring, fifo->ring_dma_addr);
	if(fifo->ring == NULL)
		goto fail;
	hififo_set_abort(fifo, 1);
	udelay(100);
	fifo->timeout = (250 * HZ) / 1000; /* default of 250 ms */
	fifo->p_hw = 0;
	fifo->p_sw = 0;
	fifo->bytes_available = 0;
	hififo_set_addr(fifo, fifo->ring_dma_addr);
	wmb();
	/* clear the abort bit on this FIFO in hardware */
	hififo_set_abort(fifo, 0);
	udelay(100);
	if(IS_TO_PC(fifo))
		hififo_set_stop(fifo, fifo->p_sw + BUFFER_SIZE - 512);
	return 0;
fail:
	printk(KERN_ERR DEVICE_NAME " %d failed to allocate buffer", fifo->n);
	hififo_release(inode, filp);
	return -ENOMEM;
}

/* Returns 1 if the FIFO contains at least count bytes, 0 otherwise */
static bool hififo_ready_read(struct hififo_fifo *fifo, int count)
{
	if(fifo->bytes_available >= count)
		return 1;
	fifo->p_hw = readlle(fifo->local_base);
	fifo->bytes_available = BUFFER_MASK & (fifo->p_hw - fifo->p_sw);
	return (fifo->bytes_available >= count);
}

#define hififo_wait(fifo, condition) \
	wait_event_interruptible_timeout(fifo->queue, condition, fifo->timeout)

static ssize_t hififo_read(struct file *filp,
			   char *buf,
			   size_t length,
			   loff_t * offset)
{
	struct hififo_fifo *fifo = filp->private_data;
	size_t bytes_copied = 0;
	size_t csize;
	ssize_t status;
	printk(KERN_INFO DEVICE_NAME " %d: read, %zu\n", fifo->n, length);
	if((buf == NULL) || ((length & 0x7F) != 0))
		return -EINVAL;
	status = mutex_lock_interruptible(&fifo->sem);
        if (status)
                return status;
	while(bytes_copied < length){
		csize = hififo_min(BUFFER_SIZE/2, length - bytes_copied);
		csize = hififo_min(csize, BUFFER_SIZE - fifo->p_sw);
		// wait for DMA data
		hififo_set_stop(fifo, fifo->p_sw + BUFFER_SIZE - 512);
		hififo_set_match(fifo, fifo->p_sw + csize);
		wmb();
		if( hififo_wait(fifo, hififo_ready_read(fifo, csize)) < 1 ){
			printk(KERN_INFO DEVICE_NAME " %d: rtimeout\n", fifo->n);
			break;
		}
		if(copy_to_user(&buf[bytes_copied],
				fifo->ring + fifo->p_sw/8, csize) != 0){
			printk(KERN_INFO DEVICE_NAME " %d: rcfail\n", fifo->n);
			break;
		}
		fifo->p_sw += csize;
		fifo->p_sw &= BUFFER_MASK;
		fifo->bytes_available -= csize;
		bytes_copied += csize;
	}
	mutex_unlock(&fifo->sem);
	return bytes_copied;
}

/* Returns 1 if the FIFO contains at least count bytes, 0 otherwise */
static bool hififo_ready_write(struct hififo_fifo *fifo, int count)
 {
	u32 bytes_in_ring;
	if(fifo->bytes_available >= count)
		return 1;
	fifo->p_hw = readlle(fifo->local_base);
	bytes_in_ring = BUFFER_MASK & (fifo->p_sw - fifo->p_hw);
	fifo->bytes_available = BUFFER_SIZE - (bytes_in_ring + 512);
	return (fifo->bytes_available >= count);
}

static ssize_t hififo_write(struct file *filp, const char *buf, size_t length,
			    loff_t * off)
{
	struct hififo_fifo *fifo = filp->private_data;
	size_t bytes_copied = 0, csize;
	ssize_t status;
	printk(KERN_INFO DEVICE_NAME " %d: write, %zu\n", fifo->n, length);
	if((buf == NULL) || ((length & 0x1FF) != 0))
		return -EINVAL;
	status = mutex_lock_interruptible(&fifo->sem);
        if (status)
                return status;
	while(bytes_copied < length){
		csize = hififo_min(BUFFER_SIZE/2, length - bytes_copied);
		csize = hififo_min(csize, BUFFER_SIZE - fifo->p_sw);
		hififo_set_match(fifo, fifo->p_sw + csize + 512 - BUFFER_SIZE);
		wmb();
		if( hififo_wait(fifo, hififo_ready_write(fifo, csize)) < 1 ){
			printk(KERN_INFO DEVICE_NAME " %d: wtimeout\n", fifo->n);
			break;
		}
		if(copy_from_user
		   (fifo->ring + fifo->p_sw/8, &buf[bytes_copied], csize) != 0){
			printk(KERN_INFO DEVICE_NAME " %d: wcfail\n", fifo->n);
			break;
		}
		fifo->p_sw += csize;
		fifo->p_sw &= BUFFER_MASK;
		hififo_set_stop(fifo, fifo->p_sw);
		fifo->bytes_available -= csize;
		wmb();
		bytes_copied += csize;
	}
	mutex_unlock(&fifo->sem);
	return bytes_copied;
}

static long hififo_ioctl (struct file *file,
			  unsigned int command,
			  unsigned long arg)
{
	struct hififo_fifo *fifo = file->private_data;
	long status = mutex_lock_interruptible(&fifo->sem);
        if (status)
                return status;
	status = -ENOTTY;
	if(command == _IO(HIFIFO_IOC_MAGIC, IOC_TIMEOUT)){
		/* 1 extra jiffy so we round up rather than down */
		fifo->timeout = 1 + (arg * HZ) / 1000;
		status = 0;
	}
	if(command == _IO(HIFIFO_IOC_MAGIC, IOC_BUILD))
		status = (long) fifo->build;
	mutex_unlock(&fifo->sem);
	return status;
}

static struct file_operations fops_tpc = {
	.read = hififo_read,
	.unlocked_ioctl = hififo_ioctl,
	.open = hififo_open,
	.release = hififo_release
};

static struct file_operations fops_fpc = {
	.write = hififo_write,
	.unlocked_ioctl = hififo_ioctl,
	.open = hififo_open,
	.release = hififo_release
};

static irqreturn_t hififo_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct hififo_dev *drvdata = dev_id;
	u32 sr = readreg(drvdata, REG_INTERRUPT);
	int i;
	//printk(KERN_INFO DEVICE_NAME " interrupt: sr = %x\n", sr);
	for(i=0; i<MAX_FIFOS; i++){
		if(!(sr & (1<<i)))
			continue;
		if(drvdata->fifo[i] == NULL)
			continue;
		wake_up_all(&drvdata->fifo[i]->queue);
	}
	return IRQ_HANDLED;
}

static int hififo_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int i;
	int rc;
	dev_t dev = 0;
	char tmpstr[16];
	struct hififo_dev *drvdata;
	struct hififo_fifo * fifo;

	drvdata = devm_kzalloc(&pdev->dev,
			       sizeof(struct hififo_dev),
			       GFP_KERNEL);
	if (!drvdata){
		printk(KERN_ERR DEVICE_NAME "failed to alloc drvdata\n");
		return -ENOMEM;
	}

	rc = pcim_enable_device(pdev);
	if(rc){
		printk(KERN_ERR DEVICE_NAME ": pcim_enable_device() failed\n");
		return rc;
	}

	rc = pci_request_regions(pdev, DEVICE_NAME);
	if(rc < 0){
		printk(KERN_ERR DEVICE_NAME ": pci_request_regions() failed\n");
		return rc;
	}

	printk(KERN_INFO DEVICE_NAME\
	       ": Found Harmon Instruments PCI Express interface board\n");
	pci_set_drvdata(pdev, drvdata);

	pci_set_master(pdev); /* returns void */

	pci_set_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

	pci_set_consistent_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

	rc = pci_enable_msi(pdev);
	if(rc < 0){
		printk(KERN_ERR DEVICE_NAME ": pci_enable_msi() failed\n");
		return rc;
	}

	rc = devm_request_irq(&pdev->dev,
			      pdev->irq,
			      (irq_handler_t) hififo_interrupt,
			      0, /* flags */
			      DEVICE_NAME,
			      drvdata);
	if(rc){
		printk(KERN_ERR DEVICE_NAME ": request_irq() failed\n");
		return rc;
	}

	drvdata->pio_reg_base = (u64 *) pcim_iomap(pdev, 0, 0);
	printk(KERN_INFO DEVICE_NAME					\
	       ": pci_resource_start(dev, 0) = 0x%.8llx, virt = 0x%.16llx\n",
	       (u64) pci_resource_start(pdev, 0),
	       (u64) drvdata->pio_reg_base);

	for(i=0; i<8; i++)
		printk("bar0[%d] = %.8x\n", i, (u32) readreg(drvdata, i));
	drvdata->idreg = readreg(drvdata, REG_ID);
	drvdata->build = readreg(drvdata, REG_BUILD);
	printk(KERN_INFO DEVICE_NAME " FPGA build = 0x%.8X\n", drvdata->build);
	drvdata->nfifos = 0;
	for(i=0; i<MAX_FIFOS; i++){
		if(drvdata->idreg & (1 << i))
			drvdata->nfifos ++;
	}

	if(drvdata->nfifos == 0){
		printk(KERN_INFO DEVICE_NAME "no fifos reported on card\n");
		return -1;
	}

	/* reset it */
	writereg(drvdata, 0xFFFF, REG_RESET_SET);
	udelay(10); /* wait for completion of anything that was running */
	writereg(drvdata, 0xFFFF, REG_RESET_CLEAR);

	rc = alloc_chrdev_region(&dev, 0, drvdata->nfifos, DEVICE_NAME);
	if (rc) {
		printk(KERN_ERR DEVICE_NAME ": alloc_chrdev_region() failed\n");
		return rc;
	}

	drvdata->major = MAJOR(dev);

	for(i=0; i<MAX_FIFOS; i++){
		if((drvdata->idreg & (1 << i)) == 0)
			continue; /* fifo not present */
		fifo = devm_kzalloc(&pdev->dev,
				    sizeof(struct hififo_fifo),
				    GFP_KERNEL);
		if (!fifo){
			printk(KERN_ERR DEVICE_NAME\
			       "failed to alloc hififo_fifo\n");
			return -ENOMEM;
		}
		drvdata->fifo[i] = fifo;
		if(i<MAX_FIFOS/2){
			cdev_init(&fifo->cdev, &fops_fpc); /* returns void */
			fifo->cdev.ops = &fops_fpc;
		}
		else{
			cdev_init(&fifo->cdev, &fops_tpc); /* returns void */
			fifo->cdev.ops = &fops_tpc;
		}

		fifo->cdev.owner = THIS_MODULE;

		rc = cdev_add (&fifo->cdev, MKDEV(MAJOR(dev), i), 1);
		if (rc){
			printk(KERN_NOTICE DEVICE_NAME\
			       ": Error %d adding cdev\n", rc);
			return rc;
		}
		sprintf(tmpstr, "hififo_%d_%d", hififo_count, i);
		device_create(hififo_class,
			      NULL,
			      MKDEV(MAJOR(dev), i),
			      NULL,
			      tmpstr);
		fifo->n = i;
		spin_lock_init(&fifo->lock_open);
		fifo->local_base = drvdata->pio_reg_base+8+i;
		init_waitqueue_head(&fifo->queue);
		fifo->build = drvdata->build;
		mutex_init(&fifo->sem);
		fifo->ring = pci_alloc_consistent(pdev,
						  BUFFER_SIZE,
						  &fifo->ring_dma_addr);
	}
	hififo_count++;
	/* enable interrupts */
	writereg(drvdata, 0xFFFF, REG_INTERRUPT);
	return 0;
}

static void hififo_remove(struct pci_dev *pdev){
	struct hififo_dev *drvdata = pci_get_drvdata(pdev);
	int i;
	writereg(drvdata, 0xFF, REG_RESET_SET);
	for(i=0; i<MAX_FIFOS; i++){
		if(drvdata->fifo[i] == NULL)
			continue;
		if(drvdata->fifo[i]->ring != NULL)
			pci_free_consistent(pdev,
					    BUFFER_SIZE,
					    drvdata->fifo[i]->ring,
					    drvdata->fifo[i]->ring_dma_addr);
		drvdata->fifo[i]->ring = NULL;
		device_destroy(hififo_class, MKDEV(drvdata->major, i));
	}
	unregister_chrdev_region (MKDEV(drvdata->major, 0), drvdata->nfifos);
}

static struct pci_driver hififo_driver = {
	.name = "hififo",
	.id_table = hififo_pci_table,
	.probe = hififo_probe,
	.remove = hififo_remove,
	// see Documentation/PCI/pci.txt - pci_register_driver call
};

static int __init hififo_init(void){
	hififo_count = 0;
	hififo_class = class_create(THIS_MODULE, "hififo");
	if (IS_ERR(hififo_class)) {
		printk(KERN_ERR DEVICE_NAME "Error creating class.\n");
		//goto error;
	}
	return pci_register_driver(&hififo_driver);
}

static void __exit hififo_exit(void){
	pci_unregister_driver(&hififo_driver);
	class_destroy(hififo_class);
	return;
}

module_init(hififo_init);
module_exit(hififo_exit);

MODULE_LICENSE("GPL");
