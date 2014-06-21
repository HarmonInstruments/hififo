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

#define VENDOR_ID 0x10EE
#define DEVICE_ID 0x7024
#define SUCCESS 0
#define DEVICE_NAME "vna_dsp"

#define HIFIFO_IOC_MAGIC 'f'

#define MAX_FIFOS 8

static struct pci_device_id vna_dsp_pci_table[] = {
  {VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0 ,0},
  {0,}
};

#define BUFFER_PAGES_FROM_PC 1
#define BUFFER_PAGES_TO_PC 1
#define BUFFER_MAX_PAGES 32
#define BUFFER_PAGE_SIZE (4096*1024) // this is the size of a page in the card's page table
#define BUFFER_SIZE_TO_PC (BUFFER_PAGE_SIZE*BUFFER_PAGES_TO_PC)
#define BUFFER_SIZE_FROM_PC (BUFFER_PAGE_SIZE*BUFFER_PAGES_FROM_PC)

#define REG_INTERRUPT 0

#define REG_TO_PC_COUNT 3
#define REG_TO_PC_STOP 8
#define REG_TO_PC_MATCH 9
#define REG_TO_PC_ENABLE 10
#define REG_FROM_PC_ENABLE 11
#define REG_FROM_PC_DISABLE 12
#define REG_FROM_PC_MATCH 13
#define REG_FROM_PC_PAGE_TABLE_BASE 64
#define REG_TO_PC_PAGE_TABLE_BASE 512

static struct class *vna_class;

struct hififo_fifo1 {
  dma_addr_t dma_addr[BUFFER_MAX_PAGES];
  u64 *buffer[BUFFER_MAX_PAGES];
  u64 buffer_size;
  u64 pointer;
  int enabled;
};

struct hififo_fifo {
  struct hififo_fifo1 to_pc;
  struct hififo_fifo1 from_pc;
  struct cdev cdev;
};

struct hififo_dev {
  struct hififo_fifo *fifo[MAX_FIFOS];
  struct cdev hififo_cdev;
  dma_addr_t to_pc_dma_addr[BUFFER_MAX_PAGES];
  u64 to_pc_pointer;
  dma_addr_t from_pc_dma_addr[BUFFER_MAX_PAGES];
  int to_pc_buffer_pages;
  int from_pc_buffer_pages;
  u64 *to_pc_buffer[BUFFER_MAX_PAGES];
  u64 *from_pc_buffer[BUFFER_MAX_PAGES];
  u64 *pio_reg_base;
  wait_queue_head_t queue_read;
  int is_open;
  int a;
  int major;
};

#define fifo_writereg(data, addr) (writeq(cpu_to_le64(data), &drvdata->pio_reg_base[addr]))
#define fifo_readreg(addr) le64_to_cpu(readq(&drvdata->pio_reg_base[addr]))

static int vna_dsp_open(struct inode *inode, struct file *filp){
  struct hififo_dev *drvdata = container_of(inode->i_cdev, struct hififo_dev, hififo_cdev);
  filp->private_data = drvdata;
  printk("hififo: open, a = %d\n", drvdata->a);
  if (drvdata->is_open)
    return -EBUSY;
  drvdata->is_open++;
  try_module_get(THIS_MODULE); // increment the usage count
  return SUCCESS;
}

static int vna_dsp_release(struct inode *inode, struct file *filp){
  struct hififo_dev *drvdata = filp->private_data;
  drvdata->is_open--;/* We're now ready for our next caller */
  module_put(THIS_MODULE); // decrement the usage count
  printk("hififo: close, a = %d\n", drvdata->a);
  return SUCCESS;
}

static u64 get_bytes_in_ring_to_pc(struct hififo_dev *drvdata){
  u64 bytes_in_buffer = (fifo_readreg(REG_TO_PC_COUNT) - drvdata->to_pc_pointer) & (BUFFER_SIZE_TO_PC - 1);
  return bytes_in_buffer;
}

static u64 wait_bytes_in_ring_to_pc(struct hififo_dev *drvdata, u64 desired_bytes){
  u64 bytes_in_buffer = get_bytes_in_ring_to_pc(drvdata);
  int retval;
  if(bytes_in_buffer >= desired_bytes)
    return bytes_in_buffer;
  fifo_writereg(drvdata->to_pc_pointer + desired_bytes, REG_TO_PC_MATCH);
  bytes_in_buffer = get_bytes_in_ring_to_pc(drvdata);
  if(bytes_in_buffer >= desired_bytes)
    return bytes_in_buffer;
  retval = wait_event_interruptible_timeout(drvdata->queue_read, (get_bytes_in_ring_to_pc(drvdata) >= desired_bytes), HZ/4); // retval: 0 if timed out, else number of jiffies remaining
  bytes_in_buffer = get_bytes_in_ring_to_pc(drvdata);
  return bytes_in_buffer;
}

static ssize_t vna_dsp_read(struct file *filp,
			    char *buffer,
			    size_t length,
			    loff_t * offset){
  struct hififo_dev *drvdata = filp->private_data;
  size_t count = 0;
  size_t bytes_in_buffer;
  size_t copy_length;
  int retval;
  if((length&0x7) != 0) /* reads must be a multiple of 8 bytes */
    return -EINVAL;
  while(count != length){
    copy_length = BUFFER_PAGE_SIZE/2 - (drvdata->to_pc_pointer & (BUFFER_PAGE_SIZE/2-1));
    if(copy_length > (length-count))
      copy_length = (length-count);
    bytes_in_buffer = wait_bytes_in_ring_to_pc(drvdata, copy_length);
    if(copy_length > bytes_in_buffer){
      printk("%zu bytes in buffer, copy length = %zu\n", bytes_in_buffer, copy_length);
      return 0;
    }
    retval = copy_to_user(buffer+count, (drvdata->to_pc_pointer & (BUFFER_SIZE_TO_PC -1)) + (char *) drvdata->to_pc_buffer[0], copy_length);
    if(retval != 0){
      printk(KERN_ERR DEVICE_NAME "copy_to_user failed with %d\n", retval);
      return -EFAULT;
    }
    count += copy_length;
    drvdata->to_pc_pointer += copy_length;
    fifo_writereg(drvdata->to_pc_pointer + (BUFFER_SIZE_TO_PC - 128), REG_TO_PC_STOP);
  }
  
  return length;
}

static ssize_t vna_dsp_write(struct file *filp,
			     const char *buf,
			     size_t length,
			     loff_t * off){
  struct hififo_dev *drvdata = filp->private_data;
  int retval;
  printk("hififo: write, a = %d, %d bytes\n", drvdata->a++, (int) length);
  length &= 0xFFFFFFFFF8;
  if((length&0x7) != 0) /* writes must be a multiple of 8 bytes */
    return -EINVAL;
  retval = copy_from_user(drvdata->from_pc_buffer[0], buf, length);
  fifo_writereg(length >> 3, REG_FROM_PC_ENABLE);
  return length;//-EINVAL;
}

struct vna_dsp_ioctl_fifostat{
  u64 requested_bytes;
  u64 flags;
  u64 available_bytes;
  u64 mmap_offset;
};

static long vna_dsp_ioctl (struct file *file, unsigned int command, unsigned long arg){
  struct hififo_dev *drvdata = file->private_data;
  //struct vna_dsp_ioctl_fifostat fifostat;
  u64 tmp[4];
  
  switch(command) {
    // get mmap size
  case _IOR(HIFIFO_IOC_MAGIC,0x10,u64):
    tmp[0] = 2 * BUFFER_PAGE_SIZE * (BUFFER_PAGES_TO_PC + BUFFER_PAGES_FROM_PC);
    if(copy_to_user((void *) arg, tmp, sizeof(u64)) != 0)
      return -EFAULT;
    return 0;
    // get bytes available in to PC FIFO
  case _IO(HIFIFO_IOC_MAGIC,0x21):
    return wait_bytes_in_ring_to_pc(drvdata, arg);
    // get to PC FIFO pointer
  case _IO(HIFIFO_IOC_MAGIC,0x22):
    return drvdata->to_pc_pointer & (BUFFER_SIZE_TO_PC - 1);
    // accept bytes from to PC FIFO
  case _IO(HIFIFO_IOC_MAGIC,0x12):
    drvdata->to_pc_pointer += arg;
    fifo_writereg(drvdata->to_pc_pointer + (BUFFER_SIZE_TO_PC - 128), REG_TO_PC_STOP);
    return 0;
    // get bytes available in from PC FIFO, pointer
  case _IOWR(HIFIFO_IOC_MAGIC,0x13, u64[2]):
    if(copy_from_user(tmp, (void *) arg, sizeof(u64)) != 0)
      return -EFAULT;
    
    if(copy_from_user(tmp, (void *) arg, sizeof(u64)) != 0)
      return -EFAULT;
    drvdata->to_pc_pointer += tmp[0];
    fifo_writereg(drvdata->to_pc_pointer + (BUFFER_SIZE_TO_PC - 128), REG_TO_PC_STOP);
    return 0;
    // accept bytes from to PC FIFO
  case _IO(HIFIFO_IOC_MAGIC,0x14):
    drvdata->to_pc_pointer += arg;
    fifo_writereg(drvdata->to_pc_pointer + (BUFFER_SIZE_TO_PC - 128), REG_TO_PC_STOP);
    return 0;
  }
  return -ENOTTY;
}

static int vna_dsp_mmap(struct file *file, struct vm_area_struct *vma)
{
  struct hififo_dev *drvdata = file->private_data;
  unsigned long size;
  int retval, i, j;
  off_t uaddr = 0;

  if (!(vma->vm_flags & VM_SHARED))
    return -EINVAL;
  if (vma->vm_start & ~PAGE_MASK)
    return -EINVAL;

  size = vma->vm_end - vma->vm_start;

  if (size != 2 * BUFFER_PAGE_SIZE * (BUFFER_PAGES_TO_PC + BUFFER_PAGES_FROM_PC))
    return -EINVAL;

  /* to PC */
  for(i=0; i<2; i++){
    for(j=0; j<BUFFER_PAGES_TO_PC; j++){
      retval = remap_pfn_range(vma,
			       vma->vm_start + uaddr, 
			       drvdata->to_pc_dma_addr[j] >> PAGE_SHIFT,
			       BUFFER_PAGE_SIZE,
			       vma->vm_page_prot);
      if(retval)
	return retval;
      uaddr += BUFFER_PAGE_SIZE;
    }
  }
  /* from PC */
  for(i=0; i<2; i++){
    for(j=0; j<BUFFER_PAGES_FROM_PC; j++){
      retval = remap_pfn_range(vma,
			       vma->vm_start + uaddr, 
			       drvdata->from_pc_dma_addr[j] >> PAGE_SHIFT,
			       BUFFER_PAGE_SIZE,
			       vma->vm_page_prot);
      if(retval)
	return retval;
      uaddr += BUFFER_PAGE_SIZE;
    }
  }
  return 0;
}

static struct file_operations fops = {
 .read = vna_dsp_read,
 .write = vna_dsp_write,
 .unlocked_ioctl = vna_dsp_ioctl,
 .mmap = vna_dsp_mmap,
 .open = vna_dsp_open,
 .release = vna_dsp_release
};

static irqreturn_t vna_interrupt(int irq, void *dev_id, struct pt_regs *regs){
  struct hififo_dev *drvdata = dev_id;
  u64 sr = fifo_readreg(REG_INTERRUPT);
  if(sr & 0x0001)
    wake_up_all(&drvdata->queue_read);
  //printk("VNA interrupt: sr = %llx\n", sr);
  return IRQ_HANDLED;
}

static int vna_dsp_probe(struct pci_dev *pdev, const struct pci_device_id *id){
  int i, j;
  int retval;
  dev_t dev = 0;
  struct hififo_dev *drvdata;

  drvdata = devm_kzalloc(&pdev->dev, sizeof(struct hififo_dev), GFP_KERNEL);
  if (!drvdata){
    printk(KERN_ERR DEVICE_NAME "failed to alloc drvdata\n");
    return -ENOMEM;
  }

  retval = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
  if (retval) {
    printk(KERN_ERR DEVICE_NAME ": alloc_chrdev_region() failed\n");
    return retval;
  }
  
  drvdata->major = MAJOR(dev);
  cdev_init(&drvdata->hififo_cdev, &fops);
  drvdata->hififo_cdev.owner = THIS_MODULE;
  drvdata->hififo_cdev.ops = &fops;
  retval = cdev_add (&drvdata->hififo_cdev, MKDEV(MAJOR(dev), 0), 1);
  if (retval){
    printk(KERN_NOTICE DEVICE_NAME ": Error %d adding cdev\n", retval);
    return retval;
  }
  device_create(vna_class, NULL, MKDEV(MAJOR(dev), 0), NULL, "vna");
  
  init_waitqueue_head(&drvdata->queue_read);
  
  retval = pcim_enable_device(pdev);
  if(retval){
    printk(KERN_ERR DEVICE_NAME ": pcim_enable_device() failed\n");
    return retval;
  }

  retval = pci_request_regions(pdev, DEVICE_NAME);
  if(retval < 0){
    printk(KERN_ERR DEVICE_NAME ": pci_request_regions() failed\n");
    pci_disable_device(pdev);
    return retval;
  }

  printk(KERN_INFO DEVICE_NAME ": Found Harmon Instruments PCI Express interface board\n");
  pci_set_drvdata(pdev, drvdata);

  pci_set_master(pdev); /* returns void */

  pci_set_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

  pci_set_consistent_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

  for(i=0; i<BUFFER_PAGES_FROM_PC; i++){
    drvdata->from_pc_buffer[i] = pci_alloc_consistent(pdev, BUFFER_PAGE_SIZE, &drvdata->from_pc_dma_addr[i]);
    //printk("allocated drvdata->from_pc_buffer[%d] at %16llx phys: %16llx\n", i, (u64) drvdata->from_pc_buffer[i], drvdata->from_pc_dma_addr[i]);
    if(drvdata->from_pc_buffer[i] == NULL){
      printk("Failed to allocate out buffer %d\n", i);
      return -ENOMEM;
    }
    else{
      for(j=0; j<BUFFER_PAGE_SIZE/8; j++)
	drvdata->from_pc_buffer[i][j] = j + i*512*1024;
    }
  }

  for(i=0; i<BUFFER_PAGES_TO_PC; i++){
    drvdata->to_pc_buffer[i] = pci_alloc_consistent(pdev, BUFFER_PAGE_SIZE, &drvdata->to_pc_dma_addr[i]);
    //printk("allocated drvdata->to_pc_buffer[%d] at %llx phys: %llx\n", i, (u64) drvdata->to_pc_buffer[i], drvdata->to_pc_dma_addr[i]);
    if(drvdata->to_pc_buffer[i] == NULL){
      printk("Failed to allocate in buffer %d\n", i);
      return -ENOMEM;
    }
    else{
      for(j=0; j<BUFFER_PAGE_SIZE/8; j++)
	drvdata->to_pc_buffer[i][j] = 0xDEADAAAA + ((u64) j << 32);
    }
  }

  retval = pci_enable_msi(pdev);
  if(retval < 0){
    printk(KERN_ERR DEVICE_NAME ": pci_enable_msi() failed\n");
    return retval;
  }

  retval = devm_request_irq(&pdev->dev, pdev->irq, (irq_handler_t) vna_interrupt, 0 /* flags */, DEVICE_NAME, drvdata);
  if(retval){
    printk(KERN_ERR DEVICE_NAME ": request_irq() failed\n");
    return retval;
  }

  drvdata->pio_reg_base = (u64 *) pcim_iomap(pdev, 0, 65536);
  printk(KERN_INFO DEVICE_NAME ": pci_resource_start(dev, 0) = 0x%.8llx, virt = 0x%.16llx\n", (u64) pci_resource_start(pdev, 0), (u64) drvdata->pio_reg_base);
  
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (u64) fifo_readreg(i));

  /* disable it */
  fifo_writereg(0, REG_FROM_PC_DISABLE);
  fifo_writereg(0, REG_TO_PC_ENABLE);
  udelay(10); /* wait for completion of anything that was running */
  /* set interrupt match registers */
  fifo_writereg(0, REG_TO_PC_MATCH);
  fifo_writereg(0, REG_FROM_PC_MATCH); 
  /* enable interrupts */
  fifo_writereg(0xF, REG_INTERRUPT);
  /* load the card page tables */
  for(i=0; i<32; i++){
    fifo_writereg(drvdata->to_pc_dma_addr[i%BUFFER_PAGES_TO_PC], i+REG_TO_PC_PAGE_TABLE_BASE);
    fifo_writereg(drvdata->from_pc_dma_addr[i%BUFFER_PAGES_FROM_PC], i+REG_FROM_PC_PAGE_TABLE_BASE);
  }
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (u64) fifo_readreg(i));
  /* enable it */
  fifo_writereg(BUFFER_SIZE_TO_PC-128, REG_TO_PC_STOP);
  fifo_writereg(1, REG_TO_PC_ENABLE);
  fifo_writereg(4*1024*1024, REG_FROM_PC_ENABLE); // enable
  drvdata->to_pc_pointer = 0;
  return 0;
}

static void vna_dsp_remove(struct pci_dev *pdev){
  struct hififo_dev *drvdata = pci_get_drvdata(pdev);
  fifo_writereg(0, REG_TO_PC_ENABLE);
  device_destroy(vna_class, MKDEV(drvdata->major, 0));
}

static struct pci_driver vna_dsp_driver = {
  .name = "vna_dsp",
  .id_table = vna_dsp_pci_table,
  .probe = vna_dsp_probe,
  .remove = vna_dsp_remove,
  // see Documentation/PCI/pci.txt - pci_register_driver call
};

static int __init vna_dsp_init(void){
  printk ("Loading vna_dsp kernel module\n");
  vna_class = class_create(THIS_MODULE, "vna");
  if (IS_ERR(vna_class)) {
    printk(KERN_ERR "Error creating vna class.\n");
    //goto error;
  }
  return pci_register_driver(&vna_dsp_driver);
}

static void __exit vna_dsp_exit(void){
  pci_unregister_driver(&vna_dsp_driver);
  class_destroy(vna_class);
  printk ("Unloading vna_dsp kernel module.\n");
  return;
}

module_init(vna_dsp_init);
module_exit(vna_dsp_exit);

MODULE_LICENSE("GPL");
