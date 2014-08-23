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

#define MAX_FIFOS 8

static struct pci_device_id hififo_pci_table[] = {
  {VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0 ,0},
  {0,}
};

#define BUFFER_SIZE (4096*1024)
#define REQ_SIZE 4096

#define REG_INTERRUPT 0
#define REG_ID 1
#define REG_COUNT 2
#define REG_RESET 3
#define REG_RESET_SET 3
#define REG_RESET_CLEAR 4

#define writereg(s, data, addr) (writeq(cpu_to_le64(data), &s->pio_reg_base[(addr)]))
#define readreg(s, addr) le32_to_cpu(readl(&s->pio_reg_base[(addr)]))

#define write_count(count) (writereg(fifo, count, REG_COUNT))
#define write_status(status) (writeq(cpu_to_le64((status) | 3), fifo->local_base))
#define write_req(s, req) (writeq(cpu_to_le64(req), s->local_base))
#define read_status() (le32_to_cpu(readl(fifo->local_base)))

static struct class *hififo_class;
static int hififo_count;

struct hififo_fifo {
  dma_addr_t dma_addr;
  u64 *buffer;
  dma_addr_t req_dma_addr;
  u64 *req;
  u64 *pio_reg_base;
  u64 *local_base;
  struct cdev cdev;
  wait_queue_head_t queue;
  u32 pointer_in;
  u32 pointer_out;
  u32 bytes_requested;
  u32 bytes_available;
  int is_open;
  int n; // fifo number
};

struct hififo_dev {
  struct hififo_fifo * fifo[MAX_FIFOS];
  u64 *pio_reg_base;
  int major;
  int nfifos;
  int idreg;
};

// TODO: make this thread safe

static int hififo_open(struct inode *inode, struct file *filp){
  struct hififo_fifo *fifo = container_of(inode->i_cdev, struct hififo_fifo, cdev);
  filp->private_data = fifo;
  printk("hififo: open\n");
  if (fifo->is_open)
    return -EBUSY;
  fifo->is_open++;
  try_module_get(THIS_MODULE); // increment the usage count
  return 0;
}

static int hififo_release(struct inode *inode, struct file *filp){
  struct hififo_fifo *fifo = filp->private_data;
  fifo->is_open--;
  module_put(THIS_MODULE); // decrement the usage count
  printk("hififo: close\n");
  return 0;
}

static u32 get_bytes_in_ring_to_pc(struct hififo_fifo *fifo){
  u32 status = read_status();
  u32 bytes_outstanding = fifo->bytes_requested - (status & 0xFFFFFFF8);
  u32 bytes_in_buffer = (BUFFER_SIZE - 1) & (fifo->pointer_in - (fifo->pointer_out + bytes_outstanding));
  printk("tpc: status = %.8x, bytes available = %x, outstanding = %x\n", status, bytes_in_buffer, bytes_outstanding);
  return bytes_in_buffer;
}

static u32 wait_bytes_in_ring_to_pc(struct hififo_fifo *fifo, u32 desired_bytes){
  u32 bytes_in_buffer = get_bytes_in_ring_to_pc(fifo);
  u32 bytes_free_in_buffer = (BUFFER_SIZE - ((BUFFER_SIZE - 1) & (fifo->pointer_in - fifo->pointer_out)));
  int retval, i;
  if(bytes_in_buffer >= desired_bytes)
    return bytes_in_buffer;
  if(bytes_free_in_buffer > BUFFER_SIZE/8)
    {
      bytes_free_in_buffer -= 1024;
      bytes_free_in_buffer = hififo_min(bytes_free_in_buffer, BUFFER_SIZE - fifo->pointer_in);
      bytes_free_in_buffer = hififo_min(bytes_free_in_buffer, BUFFER_SIZE/2);
      printk("read: %x, length = %x\n", fifo->pointer_in, (u32) bytes_free_in_buffer);
      fifo->req[0] = 1 | bytes_free_in_buffer;
      fifo->req[1] = 2 | (fifo->dma_addr + fifo->pointer_in);
      for(i=2; i<64; i++)
	fifo->req[i] = 0;
      write_req(fifo, fifo->req_dma_addr | 0x04);
      fifo->bytes_requested += bytes_free_in_buffer;
      fifo->pointer_in += bytes_free_in_buffer;
      fifo->pointer_in &= (BUFFER_SIZE - 1);
    }
  write_status(3 | (fifo->bytes_requested + desired_bytes - ((BUFFER_SIZE-1) & (fifo->pointer_in - fifo->pointer_out))));
  retval = wait_event_interruptible_timeout(fifo->queue, (get_bytes_in_ring_to_pc(fifo) >= desired_bytes), HZ/4); // retval: 0 if timed out, else number of jiffies remaining
  bytes_in_buffer = get_bytes_in_ring_to_pc(fifo);
  printk("tpc wait, %d jiffies remain, %u bytes avail\n", retval, bytes_in_buffer);
  return bytes_in_buffer;
}

static ssize_t hififo_read(struct file *filp,
			    char *buffer,
			    size_t length,
			    loff_t * offset){
  struct hififo_fifo *fifo = filp->private_data;
  size_t count = 0;
  u32 bytes_in_buffer;
  size_t copy_length;
  int retval;
  if((length&0x1FF) != 0){ /* reads must be a multiple of 8 bytes */
    printk("hififo_read: invalid length");
    return -EINVAL;
  }
  while(count != length){
    copy_length = hififo_min((BUFFER_SIZE/2), BUFFER_SIZE - fifo->pointer_out);
    copy_length = hififo_min(copy_length, length-count);
    bytes_in_buffer = wait_bytes_in_ring_to_pc(fifo, copy_length);
    if(copy_length > bytes_in_buffer){
      printk("%u bytes in buffer, copy length = %zu\n", bytes_in_buffer, copy_length);
      return 0;
    }
    retval = copy_to_user(buffer + count, fifo->pointer_out + (char *) fifo->buffer, copy_length);
    if(retval != 0){
      printk(KERN_ERR DEVICE_NAME "copy_to_user failed with %d\n", retval);
      return -EFAULT;
    }
    count += copy_length;
    fifo->pointer_out += copy_length;
    fifo->pointer_out &= (BUFFER_SIZE - 1);
  }
  return length;
}

static int get_bytes_in_ring_from_pc(struct hififo_fifo *fifo, u32 desired_bytes){
  u32 status;
  if(fifo->bytes_available >= desired_bytes)
    return 1;
  status = read_status();
  if((status & 0x04) != 0){
    status = read_status();
    printk("retry: REG_FROM_PC_COUNT = %x\n", status);
  }
  fifo->bytes_available = (BUFFER_SIZE - 1024) - (fifo->bytes_requested - (status & 0xFFFFFFF8));
  printk("fpc: status = %x, bytes available = %x, bytes_requested = %x\n", status, fifo->bytes_available, fifo->bytes_requested);
  return (fifo->bytes_available >= desired_bytes);
}

static u32 wait_ring_from_pc(struct hififo_fifo *fifo, u32 desired_bytes){
  int retval;
  u32 tmp;
  if(get_bytes_in_ring_from_pc(fifo, desired_bytes))
    return fifo->bytes_available;
  tmp = (3 | (fifo->bytes_requested + desired_bytes - (BUFFER_SIZE - 1024)));
  printk("fpc: setting interrupt for %.8x\n", tmp);
  write_status(tmp);
  retval = wait_event_interruptible_timeout(fifo->queue, get_bytes_in_ring_from_pc(fifo, desired_bytes), HZ/4); // retval: 0 if timed out, else number of jiffies remaining
  printk("fpc: wait, %d jiffies remain, %.8x bytes avail\n", retval, fifo->bytes_available);
  return fifo->bytes_available;
}

static ssize_t hififo_write(struct file *filp,
			     const char *buf,
			     size_t length,
			     loff_t * off){
  struct hififo_fifo *fifo = filp->private_data;
  size_t count = 0;
  size_t copy_length = 0;
  int retval, i;
  printk("fpc: write, %x bytes, addr = 0x%p\n", (int) length, buf);
  if((length&0x7) != 0) /* writes must be a multiple of 8 bytes */
    return -EINVAL;
  while(count != length){
    copy_length = hififo_min(BUFFER_SIZE/2, BUFFER_SIZE - fifo->pointer_in);
    copy_length = hififo_min(copy_length, length-count);
    retval = wait_ring_from_pc(fifo, copy_length);
    if(retval < copy_length){
      printk("wait from user failed\n");
      return count;
    }
    retval = copy_from_user(((void *) fifo->buffer) + fifo->pointer_in, buf + count, copy_length);
    if(retval != 0){
      printk("copy from user failed\n");
      return -EFAULT;
    }
    count += copy_length;
    fifo->req[0] = 1 | copy_length;
    fifo->req[1] = 2 | (fifo->dma_addr + fifo->pointer_in);
    for(i=2; i<64; i++)
      fifo->req[i] = 0;
    write_req(fifo, fifo->req_dma_addr | 4);
    printk("fpc: hwrite: %x, length = %x\n", fifo->pointer_in, (u32) copy_length); 
    fifo->bytes_available -= copy_length;
    fifo->pointer_in += copy_length;
    fifo->pointer_in &= (BUFFER_SIZE - 1);
    fifo->bytes_requested += copy_length;
  }
  return length;
}

static long hififo_ioctl (struct file *file, unsigned int command, unsigned long arg){
  //struct hififo_fifo *fifo = file->private_data;
  u64 tmp[8];
  switch(command) {
    // get info
  case _IOR(HIFIFO_IOC_MAGIC, IOC_INFO ,u64[8]):
    tmp[0] = BUFFER_SIZE;
    tmp[1] = BUFFER_SIZE;
    //tmp[2] = fifo->pointer_in;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    tmp[6] = 0;
    tmp[7] = 0;
    if(copy_to_user((void *) arg, tmp, 8*sizeof(u64)) != 0)
      return -EFAULT;
    return 0;
    // get bytes available in to PC FIFO
  case _IO(HIFIFO_IOC_MAGIC, IOC_GET_TO_PC):
    return 0;//wait_bytes_in_ring_to_pc(fifo, arg);
    // accept bytes from to PC FIFO
  case _IO(HIFIFO_IOC_MAGIC, IOC_PUT_TO_PC):
    //fifo->pointer += arg;
    //local_writereg(fifo->pointer + (BUFFER_SIZE - 128), REG_LOCAL_COUNT);
    return 0;
    // get bytes available in from PC FIFO
  case _IO(HIFIFO_IOC_MAGIC, IOC_GET_FROM_PC):
    return 0;//wait_ring_from_pc(fifo, arg);
    // commit bytes to from PC FIFO
  case _IO(HIFIFO_IOC_MAGIC, IOC_PUT_FROM_PC):
    //fifo->pointer += arg;
    //local_writereg(fifo->pointer, REG_LOCAL_COUNT);
    return 0;
  }
  return -ENOTTY;
}

static int hififo_mmap(struct file *file, struct vm_area_struct *vma)
{
  struct hififo_fifo *fifo = file->private_data;
  unsigned long size;
  int retval, i;

  if (!(vma->vm_flags & VM_SHARED))
    return -EINVAL;
  if (vma->vm_start & ~PAGE_MASK)
    return -EINVAL;

  size = vma->vm_end - vma->vm_start;

  if (size != 2 * BUFFER_SIZE)
    return -EINVAL;

  for(i=0; i<2; i++){
    retval = remap_pfn_range(vma,
			     vma->vm_start + BUFFER_SIZE * i, 
			     fifo->dma_addr >> PAGE_SHIFT,
			     BUFFER_SIZE,
			     vma->vm_page_prot);
    if(retval)
      return retval;
  }
  return 0;
}

static struct file_operations fops_tpc = {
 .read = hififo_read,
 .unlocked_ioctl = hififo_ioctl,
 .mmap = hififo_mmap,
 .open = hififo_open,
 .release = hififo_release
};

static struct file_operations fops_fpc = {
 .write = hififo_write,
 .unlocked_ioctl = hififo_ioctl,
 .mmap = hififo_mmap,
 .open = hififo_open,
 .release = hififo_release
};

static irqreturn_t hififo_interrupt(int irq, void *dev_id, struct pt_regs *regs){
  struct hififo_dev *drvdata = dev_id;
  u32 sr = readreg(drvdata, REG_INTERRUPT);
  int i;
  printk("hififo interrupt: sr = %x\n", sr);
  for(i=0; i<MAX_FIFOS; i++){
    if((sr & (0x3<<(2*i))) && (drvdata->fifo[i] != NULL))
      wake_up_all(&drvdata->fifo[i]->queue);
  }
  return IRQ_HANDLED;
}

static int hififo_probe(struct pci_dev *pdev, const struct pci_device_id *id){
  int i;
  int retval;
  dev_t dev = 0;
  char tmpstr[16];
  struct hififo_dev *drvdata;
  struct hififo_fifo * fifo;

  drvdata = devm_kzalloc(&pdev->dev, sizeof(struct hififo_dev), GFP_KERNEL);
  if (!drvdata){
    printk(KERN_ERR DEVICE_NAME "failed to alloc drvdata\n");
    return -ENOMEM;
  }
    
  retval = pcim_enable_device(pdev);
  if(retval){
    printk(KERN_ERR DEVICE_NAME ": pcim_enable_device() failed\n");
    return retval;
  }

  retval = pci_request_regions(pdev, DEVICE_NAME);
  if(retval < 0){
    printk(KERN_ERR DEVICE_NAME ": pci_request_regions() failed\n");
    return retval;
  }

  printk(KERN_INFO DEVICE_NAME ": Found Harmon Instruments PCI Express interface board\n");
  pci_set_drvdata(pdev, drvdata);

  pci_set_master(pdev); /* returns void */

  pci_set_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

  pci_set_consistent_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

  retval = pci_enable_msi(pdev);
  if(retval < 0){
    printk(KERN_ERR DEVICE_NAME ": pci_enable_msi() failed\n");
    return retval;
  }

  retval = devm_request_irq(&pdev->dev, pdev->irq, (irq_handler_t) hififo_interrupt, 0 /* flags */, DEVICE_NAME, drvdata);
  if(retval){
    printk(KERN_ERR DEVICE_NAME ": request_irq() failed\n");
    return retval;
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

  retval = alloc_chrdev_region(&dev, 0, drvdata->nfifos, DEVICE_NAME);
  if (retval) {
    printk(KERN_ERR DEVICE_NAME ": alloc_chrdev_region() failed\n");
    return retval;
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

    retval = cdev_add (&fifo->cdev, MKDEV(MAJOR(dev), i), 1);
    if (retval){
      printk(KERN_NOTICE DEVICE_NAME ": Error %d adding cdev\n", retval);
      return retval;
    }
    sprintf(tmpstr, "hififo_%d_%d", hififo_count, i);
    device_create(hififo_class, NULL, MKDEV(MAJOR(dev), i), NULL, tmpstr);
    fifo->n = i;
    fifo->pio_reg_base = drvdata->pio_reg_base;
    fifo->local_base = drvdata->pio_reg_base+8+i;
    init_waitqueue_head(&fifo->queue);
    fifo->pointer_in = 0;
    fifo->pointer_out = 0;
    fifo->bytes_requested = 0;
    fifo->buffer = pci_alloc_consistent(pdev, BUFFER_SIZE, &fifo->dma_addr);
    if(fifo->buffer == NULL){
      printk("Failed to allocate DMA buffer\n");
      return -ENOMEM;
    }
    fifo->req = pci_alloc_consistent(pdev, REQ_SIZE, &fifo->req_dma_addr);
    if(fifo->req == NULL){
      printk("Failed to allocate request DMA buffer\n");
      return -ENOMEM;
    }
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
