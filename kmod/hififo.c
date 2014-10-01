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
#define IOC_GET_TO_PC 0x11
#define IOC_PUT_TO_PC 0x12
#define IOC_GET_FROM_PC 0x13
#define IOC_PUT_FROM_PC 0x14
#define IOC_SET_TIMEOUT 0x15

#define HIFIFO_MATCH(x) (cpu_to_le64( 1 | (x)))
#define HIFIFO_ADDR(x) ( 2 | (x))
#define HIFIFO_COUNT(x) ( 3 | (x))
#define HIFIFO_DESCRIPTOR(x) ( 4 | (x))
#define HIFIFO_ABORT(x) ( 5 | ((x) ? 0x100 : 0x000))

#define MAX_FIFOS 8

static struct pci_device_id hififo_pci_table[] = {
  {VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0 ,0},
  {0,}
};

#define MAX_PAGES 4096 /* Maximum number of pages for one DMA run */
#define MAX_DMA_REQUEST 65536 /* Maximum number of bytes in one DMA descriptor slot */
#define REQ_SIZE (MAX_PAGES * 17)

#define REG_INTERRUPT 0
#define REG_ID 1
#define REG_RESET 3
#define REG_RESET_SET 3
#define REG_RESET_CLEAR 4

#define writereg(s, data, addr) (writeq(cpu_to_le64(data), &s->pio_reg_base[(addr)]))
#define readreg(s, addr) le32_to_cpu(readl(&s->pio_reg_base[(addr)]))

#define DMA_DIRECTION(fifo) (((fifo)->n > (MAX_FIFOS/2)) ? PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE)

static struct class *hififo_class;
static int hififo_count;

struct hififo_ioctl{
  void *buf;
  ssize_t length;
  s32 wait;
  s32 abort;
  s32 timeout;
  u32 info;
};

struct hififo_dma {
  struct scatterlist sglist[MAX_PAGES];
  struct page * page_list[MAX_PAGES];
  dma_addr_t req_dma_addr;
  u64 *req;
  u32 bytes_requested;
  int n_pages;
  int mapped;
};

struct hififo_fifo {
  //  u64 *pio_reg_base;
  u64 *local_base;
  struct pci_dev *pdev;
  struct hififo_dma * dma[2];
  struct cdev cdev;
  u32 n_requested, n_completed;
  wait_queue_head_t queue_dma;
  wait_queue_head_t queue_count;
  s32 bytes_requested;
  spinlock_t lock_open;
  int n; // fifo number
  int timeout;
};

struct hififo_dev {
  struct hififo_fifo * fifo[MAX_FIFOS];
  u64 *pio_reg_base;
  int major;
  int nfifos;
  int idreg;
};

static int hififo_wait(struct hififo_fifo *fifo, u32 tag);
static int hififo_release(struct inode *inode, struct file *filp);
static void hififo_abort(struct hififo_fifo *fifo);

static int hififo_open(struct inode *inode, struct file *filp){
  struct hififo_fifo *fifo = container_of(inode->i_cdev, struct hififo_fifo, cdev);
  int i;
  if (!try_module_get(THIS_MODULE))
    return -ENODEV;
  if (! spin_trylock(&fifo->lock_open))
    return -EBUSY;
  filp->private_data = fifo;
  printk(KERN_INFO "hififo %d: open\n", fifo->n);
  fifo->dma[0] = kcalloc(2, sizeof(struct hififo_dma), GFP_KERNEL);
  if(fifo->dma[0] == NULL)
    goto fail;
  fifo->dma[1] = ((void *) fifo->dma[0]) + sizeof(struct hififo_dma);
  for(i=0; i<2; i++){
    fifo->dma[i]->req = pci_alloc_consistent(fifo->pdev, REQ_SIZE, &fifo->dma[i]->req_dma_addr);
    if(fifo->dma[i]->req == NULL)
      goto fail;
  }
  fifo->timeout = HZ/4;
  fifo->n_requested = 0;
  fifo->n_completed = 0;
  return 0;
 fail:
  hififo_release(inode, filp);
  return -ENOMEM;
}

static int hififo_release(struct inode *inode, struct file *filp){
  struct hififo_fifo *fifo = filp->private_data;
  int i;
  hififo_wait(fifo, fifo->n_requested);
  for(i=0; i<2; i++){
    if(fifo->dma[i] == NULL)
      break;
    if(fifo->dma[i]->req != NULL)
      pci_free_consistent(fifo->pdev, REQ_SIZE, fifo->dma[i]->req, fifo->dma[i]->req_dma_addr);
  }
  if(fifo->dma[0] != NULL)
    kfree(fifo->dma[0]);
  fifo->dma[0] = NULL;
  fifo->dma[1] = NULL;
  module_put(THIS_MODULE); // decrement the usage count
  printk("hififo %d: close\n", fifo->n);
  spin_unlock(&fifo->lock_open);
  return 0;
}

static int hififo_generate_descriptor(struct hififo_dma * dma, int sg_count)
{
  int i, j=0;
  size_t len;
  dma_addr_t hw_addr;
  for (i = 0; i < sg_count; i++) {
    hw_addr = sg_dma_address(&dma->sglist[i]);
    len = sg_dma_len(&dma->sglist[i]);
    while(i+1 < sg_count){
      if(hw_addr+len != sg_dma_address(&dma->sglist[i+1]))
	break;
      len += sg_dma_len(&dma->sglist[i++]);
      if(len >= MAX_DMA_REQUEST)
	break;
    }
    if(j % 64 == 62){
      dma->req[j] = 0;
      dma->req[j+1] = HIFIFO_DESCRIPTOR(dma->req_dma_addr + 8*(j + 2));
      j+= 2;
    }
    dma->req[j++] = HIFIFO_ADDR(hw_addr);
    dma->req[j++] = HIFIFO_COUNT(len);
  }
  while((j&(64-1)) != 0)
    dma->req[j++] = 0;
  return 0;
}

/* unmap and unpin pages */
static void hififo_unmap_sg(struct hififo_fifo *fifo, struct hififo_dma *dma){
  int i;
  if(dma == NULL)
    return;
  if(dma->mapped)
    pci_unmap_sg(fifo->pdev, dma->sglist, dma->n_pages, DMA_DIRECTION(fifo));
  dma->mapped = 0;
  for (i = 0; i < dma->n_pages; i++) {
    if (dma->page_list[i] != NULL) {
      if(fifo->n > 4)
	set_page_dirty_lock(dma->page_list[i]);
      put_page(dma->page_list[i]);
    }
  }
  dma->n_pages = 0;
}

/* map and pin user pages */
static int hififo_map_sg(struct hififo_fifo *fifo, struct hififo_dma *dma, void *buf, size_t length){
  int i, rc;
  dma->n_pages = DIV_ROUND_UP(length, PAGE_SIZE);
  printk ("hififo %d: start gup\n", fifo->n);
  rc = get_user_pages_fast((loff_t)buf,
			   dma->n_pages,
			   fifo->n > 4 ? 1 : 0,
			   dma->page_list);
  if (rc < dma->n_pages) {
    dma->n_pages = rc;
    printk("hififo %d: get user pages failed with %d\n", fifo->n, rc);
    if(rc > 0)
      hififo_unmap_sg(fifo, dma);
    dma->n_pages = 0;
    return -EFAULT;
  }
  /* Map a scatter-gather list to point at the userspace pages */
  /*first, middle*/
  for(i=0; i < dma->n_pages-1; i++)
    sg_set_page(&dma->sglist[i], dma->page_list[i], PAGE_SIZE, 0);
  /*last*/
  if (dma->n_pages > 1) 
    sg_set_page(&dma->sglist[dma->n_pages-1], dma->page_list[dma->n_pages-1], length - ((dma->n_pages-1)*PAGE_SIZE), 0);
  
  rc = pci_map_sg(fifo->pdev, dma->sglist, dma->n_pages, DMA_DIRECTION(fifo));
  //printk ("hififo %d: pci_map_sg returned %d\n", fifo->n, rc);
  dma->mapped = 1;
  hififo_generate_descriptor(dma, rc);
  rc = wait_event_interruptible_timeout(fifo->queue_dma,
					(le32_to_cpu(readl(fifo->local_base)) & 0x1),
					fifo->timeout);
  // check rc
  writeq(cpu_to_le64(HIFIFO_DESCRIPTOR(dma->req_dma_addr)), fifo->local_base);
  fifo->bytes_requested += length;
  dma->bytes_requested = fifo->bytes_requested;
  printk("hififo %d: end gup, doing request, addr = 0x%.16llx, %d jiffies remain\n", fifo->n, dma->req_dma_addr, rc);
  return 0;
}

static s32 hififo_bytes_remaining(struct hififo_fifo *fifo, struct hififo_dma *dma){
  s32 rv = (s32) (dma->bytes_requested - (le32_to_cpu(readl(fifo->local_base)) & 0xFFFFFFF8));
  printk("hififo %d, %d bytes remain\n", fifo->n, rv);
  return rv;
}

/* wait for completion, returns number of bytes not transferred */
static int hififo_wait(struct hififo_fifo *fifo, u32 tag){
  int rc;
  s32 requests_outstanding = (s32) (tag - fifo->n_completed);
  struct hififo_dma * dma = fifo->dma[(tag-1) & 1];
  if(requests_outstanding < 1)
    return 0;
  writeq(HIFIFO_MATCH(dma->bytes_requested), fifo->local_base);
  rc = wait_event_interruptible_timeout(fifo->queue_count,
					hififo_bytes_remaining(fifo, dma) <= 0, 
					fifo->timeout); // rc: 0 if timed out, else number of jiffies remaining
  printk("hififo %d: wait, %d jiffies remain, %.8x bytes tf, %.8x bytes requested\n", fifo->n, rc, le32_to_cpu(readl(fifo->local_base)), fifo->bytes_requested);
  if(rc <= 0){
    hififo_abort(fifo);
    return -EAGAIN;
  }
  while(requests_outstanding--)
    hififo_unmap_sg(fifo, fifo->dma[fifo->n_completed++ & 0x01]);
  return 0;
}

/* abort any pending transfers */
static void hififo_abort(struct hififo_fifo *fifo){
  writeq(cpu_to_le64(HIFIFO_ABORT(1)), fifo->local_base);
  udelay(100); // allow pending DMA to complete
  writeq(cpu_to_le64(HIFIFO_ABORT(0)), fifo->local_base);
  fifo->n_completed = fifo->n_requested;
  hififo_unmap_sg(fifo, fifo->dma[0]);
  hififo_unmap_sg(fifo, fifo->dma[1]);
  fifo->bytes_requested = (le32_to_cpu(readl(fifo->local_base)) & 0xFFFFFFF8);
}

static int hififo_queue_request(struct hififo_fifo *fifo, char * buf, size_t length){
  size_t copy_length;
  size_t count = 0;
  if((((size_t) buf) & 0xFFF) != 0)
    return -EINVAL;
  if((length&0x1FF) != 0){ /* reads must be a multiple of 8 bytes */
    printk("hififo_read: invalid length");
    return -EINVAL;
  }
  while(count != length){
    copy_length = hififo_min(MAX_PAGES*PAGE_SIZE, length-count);
    if(hififo_wait(fifo, fifo->n_requested - 1) != 0)
      return -EAGAIN;
    if(hififo_map_sg(fifo, fifo->dma[fifo->n_requested & 0x01], buf+count, copy_length) != 0){
      hififo_abort(fifo);
      return -EAGAIN;
    }
    count += copy_length;
    fifo->n_requested++;
  }
  return 0;
}

static ssize_t hififo_read(struct file *filp,
			    char *buf,
			    size_t length,
			    loff_t * offset){
  struct hififo_fifo *fifo = filp->private_data;
  s32 bytes_requested_initial = fifo->bytes_requested;
  printk("hififo %d: read, %x bytes, addr = 0x%p\n", fifo->n, (int) length, buf);
  if((buf == NULL) || (length == 0))
    return -EINVAL;
  hififo_queue_request(fifo, buf, length);
  hififo_wait(fifo, fifo->n_requested);
  return fifo->bytes_requested - bytes_requested_initial;
}

static ssize_t hififo_write(struct file *filp,
			     const char *buf,
			     size_t length,
			     loff_t * off){
  struct hififo_fifo *fifo = filp->private_data;
  s32 bytes_requested_initial = fifo->bytes_requested;
  printk("hififo %d: write, %x bytes, addr = 0x%p\n", fifo->n, (int) length, buf);
  if((buf == NULL) || (length == 0))
    return -EINVAL;
  hififo_queue_request(fifo, (void *) buf, length);
  hififo_wait(fifo, fifo->n_requested);
  return fifo->bytes_requested - bytes_requested_initial;
}

static long hififo_ioctl (struct file *file, unsigned int command, unsigned long arg){
  struct hififo_fifo *fifo = file->private_data;
  struct hififo_ioctl tmp;
  s32 bytes_requested_initial = fifo->bytes_requested;
  if(command == _IOWR(HIFIFO_IOC_MAGIC, IOC_INFO, struct hififo_ioctl)){
    if(copy_from_user(&tmp, (void *) arg, sizeof(struct hififo_ioctl)) != 0)
      return -EFAULT;
    if((tmp.length != 0) && (tmp.buf != NULL))
      hififo_queue_request(fifo, tmp.buf, tmp.length);
    if(tmp.wait < 1) // 0 flushes all, -1 flushes all but this request
      hififo_wait(fifo, fifo->n_requested + tmp.wait);
    if(tmp.abort)
      hififo_abort(fifo);
    tmp.length = fifo->bytes_requested - bytes_requested_initial;
    if(copy_to_user((void *) arg, &tmp, sizeof(struct hififo_ioctl)) != 0)
      return -EFAULT;
    return 0;
  }
  return -ENOTTY;
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

static irqreturn_t hififo_interrupt(int irq, void *dev_id, struct pt_regs *regs){
  struct hififo_dev *drvdata = dev_id;
  u32 sr = readreg(drvdata, REG_INTERRUPT);
  int i;
  printk("hififo interrupt: sr = %x\n", sr);
  for(i=0; i<MAX_FIFOS; i++){
    if(drvdata->fifo[i] != NULL){
      if(sr & (0x1<<(2*i)))
	wake_up_all(&drvdata->fifo[i]->queue_count);
      if(sr & (0x2<<(2*i)))
	wake_up_all(&drvdata->fifo[i]->queue_dma);
    }
  }
  return IRQ_HANDLED;
}

static int hififo_probe(struct pci_dev *pdev, const struct pci_device_id *id){
  int i;
  int rc;
  dev_t dev = 0;
  char tmpstr[16];
  struct hififo_dev *drvdata;
  struct hififo_fifo * fifo;

  drvdata = devm_kzalloc(&pdev->dev, sizeof(struct hififo_dev), GFP_KERNEL);
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

  printk(KERN_INFO DEVICE_NAME ": Found Harmon Instruments PCI Express interface board\n");
  pci_set_drvdata(pdev, drvdata);

  pci_set_master(pdev); /* returns void */

  pci_set_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

  pci_set_consistent_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

  rc = pci_enable_msi(pdev);
  if(rc < 0){
    printk(KERN_ERR DEVICE_NAME ": pci_enable_msi() failed\n");
    return rc;
  }

  rc = devm_request_irq(&pdev->dev, pdev->irq, (irq_handler_t) hififo_interrupt, 0 /* flags */, DEVICE_NAME, drvdata);
  if(rc){
    printk(KERN_ERR DEVICE_NAME ": request_irq() failed\n");
    return rc;
  }

  drvdata->pio_reg_base = (u64 *) pcim_iomap(pdev, 0, 0);
  printk(KERN_INFO DEVICE_NAME ": pci_resource_start(dev, 0) = 0x%.8llx, virt = 0x%.16llx\n", (u64) pci_resource_start(pdev, 0), (u64) drvdata->pio_reg_base);
  
  for(i=0; i<8; i++)
    printk("bar0[%d] = %.8x\n", i, (u32) readreg(drvdata, i));
  drvdata->idreg = readreg(drvdata, REG_ID);
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
      continue;
    fifo = devm_kzalloc(&pdev->dev, sizeof(struct hififo_fifo), GFP_KERNEL);
    if (!fifo){
      printk(KERN_ERR DEVICE_NAME "failed to alloc hififo_fifo\n");
      return -ENOMEM;
    }
    drvdata->fifo[i] = fifo;
    if(i<MAX_FIFOS/2){
      cdev_init(&fifo->cdev, &fops_fpc); // void
      fifo->cdev.ops = &fops_fpc;
    }
    else{
      cdev_init(&fifo->cdev, &fops_tpc); // void
      fifo->cdev.ops = &fops_tpc;
    }

    fifo->cdev.owner = THIS_MODULE;

    rc = cdev_add (&fifo->cdev, MKDEV(MAJOR(dev), i), 1);
    if (rc){
      printk(KERN_NOTICE DEVICE_NAME ": Error %d adding cdev\n", rc);
      return rc;
    }
    sprintf(tmpstr, "hififo_%d_%d", hififo_count, i);
    device_create(hififo_class, NULL, MKDEV(MAJOR(dev), i), NULL, tmpstr);
    fifo->n = i;
    spin_lock_init(&fifo->lock_open);
    fifo->pdev = pdev;
    //fifo->pio_reg_base = drvdata->pio_reg_base;
    fifo->local_base = drvdata->pio_reg_base+8+i;
    init_waitqueue_head(&fifo->queue_dma);
    init_waitqueue_head(&fifo->queue_count);
    fifo->bytes_requested = 0;
    hififo_abort(fifo);
  }
  hififo_count++;
  /* enable interrupts */
  writereg(drvdata, 0xFFFF, REG_INTERRUPT);
  for(i=0; i<32; i++)
    printk("bar0[%d] = %.8x\n", i, readreg(drvdata, i));
  udelay(10000); 
  for(i=0; i<32; i++)
    printk("bar0[%d] = %.8x\n", i, readreg(drvdata, i));
  /* enable it */
  return 0;
}

static void hififo_remove(struct pci_dev *pdev){
  struct hififo_dev *drvdata = pci_get_drvdata(pdev);
  int i;
  writereg(drvdata, 0xFF, REG_RESET_SET);
  for(i=0; i<MAX_FIFOS; i++){
    if(drvdata->fifo[i] != NULL)
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
  printk ("Loading hififo kernel module\n");
  hififo_count = 0;
  hififo_class = class_create(THIS_MODULE, "hififo");
  if (IS_ERR(hififo_class)) {
    printk(KERN_ERR "Error creating hififo class.\n");
    //goto error;
  }
  return pci_register_driver(&hififo_driver);
}

static void __exit hififo_exit(void){
  pci_unregister_driver(&hififo_driver);
  class_destroy(hififo_class);
  printk ("Unloading hififo kernel module.\n");
  return;
}

module_init(hififo_init);
module_exit(hififo_exit);

MODULE_LICENSE("GPL");
