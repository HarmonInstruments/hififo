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

#define VENDOR_ID 0x10EE
#define DEVICE_ID 0x7024
#define SUCCESS 0
#define DEVICE_NAME "vna_dsp"

static struct pci_device_id vna_dsp_pci_table[] = {
  {VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0 ,0},
  {0,}
};

#define BUFFER_PAGES_FROM_PC 4
#define BUFFER_PAGES_TO_PC 16
#define BUFFER_MAX_PAGES 32
#define BUFFER_PAGE_SIZE (4096*1024) // this is the size of a page in the card's page table

#define REG_INTERRUPT 0

#define REG_TO_PC_ENABLE 8
#define REG_TO_PC_DISABLE 9
#define REG_TO_PC_MATCH 10
#define REG_FROM_PC_ENABLE 11
#define REG_FROM_PC_DISABLE 12
#define REG_FROM_PC_MATCH 13
#define REG_FROM_PC_PAGE_TABLE_BASE 64
#define REG_TO_PC_PAGE_TABLE_BASE 512

static int vna_dsp_major = 0;

static struct class *vna_class;

struct hififo_dev {
  struct cdev hififo_cdev;
  dma_addr_t to_pc_dma_addr[BUFFER_MAX_PAGES];
  dma_addr_t from_pc_dma_addr[BUFFER_MAX_PAGES];
  u64 *to_pc_buffer[BUFFER_MAX_PAGES];
  u64 *from_pc_buffer[BUFFER_MAX_PAGES];
  u64 *pio_reg_base;
  wait_queue_head_t queue_read;
  int is_open;
  int a;
};

static int vna_dsp_open(struct inode *inode, struct file *filp){
  struct hififo_dev *dev = container_of(inode->i_cdev, struct hififo_dev, hififo_cdev);
  filp->private_data = dev;
  printk("hififo: open, a = %d\n", dev->a);
  if (dev->is_open)
    return -EBUSY;
  dev->is_open++;
  try_module_get(THIS_MODULE); // increment the usage count
  return SUCCESS;
}

static int vna_dsp_release(struct inode *inode, struct file *filp){
  struct hififo_dev *dev = filp->private_data;
  dev->is_open--;/* We're now ready for our next caller */
  module_put(THIS_MODULE); // decrement the usage count
  printk("hififo: close, a = %d\n", dev->a);
  return SUCCESS;
}

static ssize_t vna_dsp_read(struct file *filp,
			    char *buffer,
			    size_t length,
			    loff_t * offset){
  struct hififo_dev *dev = filp->private_data;
  length &= 0xFFFFFFFFF8;
  if(length > 1024)
    length = 1024;
  if((length&0x7) != 0) /* reads must be a multiple of 8 bytes */
    return -EINVAL;
  copy_to_user(buffer, dev->to_pc_buffer[0], length);
  //status = wait_event_interruptible_timeout(queue_read, condition, HZ/4); // retval: 0 if timed out, else number of jiffies remaining
  printk("hififo: read, a = %d, %d bytes\n", dev->a++, length);
  return length;
}

static ssize_t vna_dsp_write(struct file *filp,
			    const char *buf,
			    size_t length,
			    loff_t * off){
  struct hififo_dev *dev = filp->private_data;
  int retval;
  printk("hififo: write, a = %d, %d bytes\n", dev->a++, length);
  length &= 0xFFFFFFFFF8;
  if((length&0x7) != 0) /* writes must be a multiple of 8 bytes */
    return -EINVAL;
  if(length > (4*1024*1024))
    length = 4*1024*1024;
  retval = copy_from_user(dev->from_pc_buffer[0], buf, length);
  writeq(length >> 3, &dev->pio_reg_base[REG_FROM_PC_ENABLE]);
  return length;//-EINVAL;
}

static struct file_operations fops = {
 .read = vna_dsp_read,
 .write = vna_dsp_write,
 .open = vna_dsp_open,
 .release = vna_dsp_release
};

static irqreturn_t vna_interrupt(int irq, void *dev_id, struct pt_regs *regs){
  struct hififo_dev *dev = dev_id;
  wake_up_all(&dev->queue_read);
  printk("VNA interrupt: sr = %llx, a = %d\n", (uint64_t) readq(&dev->pio_reg_base[REG_INTERRUPT]), dev->a);
  return IRQ_HANDLED;
}

static void check_buffer(u64 *data[])
{
  int i;
  uint64_t prev = 0;
  uint64_t cur;
  int printcount = 0;
  for(i=0; i<512*1024; i++){
    cur = readq(&data[0][i]);
    if(cur != prev + 1){
      if(printcount < 200)
	printk("buf_in[%d] = %llx\n", i, (uint64_t) cur);
      printcount++;
    }
    prev = cur;
  }
  printk("found %d errors\n", printcount);
}

static int vna_dsp_probe(struct pci_dev *pdev, const struct pci_device_id *id){
  int i, j, err, devno;
  int retval;
  dev_t dev = 0;
  struct hififo_dev *drvdata;

  retval = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
  vna_dsp_major = MAJOR(dev);
  if (retval < 0) {
    printk(KERN_WARNING "vna_dsp: can't get major %d\n", vna_dsp_major);
    return retval;
  }
  drvdata = devm_kzalloc(&pdev->dev, sizeof(struct hififo_dev), GFP_KERNEL);
  if (!drvdata){
    printk(KERN_ERR "failed to alloc drvdata\n");
    return -ENOMEM;
  }
  devno = MKDEV(vna_dsp_major, 0);
  cdev_init(&drvdata->hififo_cdev, &fops);
  drvdata->hififo_cdev.owner = THIS_MODULE;
  drvdata->hififo_cdev.ops = &fops;
  err = cdev_add (&drvdata->hififo_cdev, devno, 1);
  if (err)
    printk(KERN_NOTICE "Error %d adding cdev\n", err);
  device_create(vna_class, NULL, MKDEV(vna_dsp_major, 0), NULL, "vna");

  
  init_waitqueue_head(&drvdata->queue_read);
  drvdata->a = 3;

  retval = pcim_enable_device(pdev);
  if(retval < 0) return retval;
  retval = pci_request_regions(pdev, DEVICE_NAME);
  if(retval < 0){
    pci_disable_device(pdev);
    return retval;
  }
  printk("Found Harmon Instruments PCI Express interface board\n");
  pci_set_drvdata(pdev, drvdata);

  pci_set_master(pdev); // check return values on these
  pci_set_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);
  pci_set_consistent_dma_mask(pdev, 0xFFFFFFFFFFFFFFFF);

  //pci_set_dma_mask(dev, 0xFFFFFFFF); // if this fails, try 32
  //pci_set_consistent_dma_mask(dev, 0xFFFFFFFF);
   
  for(i=0; i<BUFFER_PAGES_FROM_PC; i++){
    drvdata->from_pc_buffer[i] = pci_alloc_consistent(pdev, BUFFER_PAGE_SIZE, &drvdata->from_pc_dma_addr[i]);
    printk("allocated drvdata->from_pc_buffer[%d] at %16llx phys: %16llx\n", i, (u64) drvdata->from_pc_buffer[i], drvdata->from_pc_dma_addr[i]);
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
    printk("allocated drvdata->to_pc_buffer[%d] at %llx phys: %llx\n", i, (u64) drvdata->to_pc_buffer[i], drvdata->to_pc_dma_addr[i]);
    if(drvdata->to_pc_buffer[i] == NULL){
      printk("Failed to allocate in buffer %d\n", i);
      return -ENOMEM;
    }
    else{
      for(j=0; j<BUFFER_PAGE_SIZE/8; j++)
	drvdata->to_pc_buffer[i][j] = 0xDEADAAAA + ((u64) j << 32);
    }
  }

  if(pci_enable_msi(pdev) < 0){
    printk(DEVICE_NAME ": pci_enable_msi() failed\n");
    return -1;
  }
  if(devm_request_irq(&pdev->dev, pdev->irq, (irq_handler_t) vna_interrupt, 0 /* flags */, DEVICE_NAME, drvdata) != 0){
    printk(DEVICE_NAME ": request_irq() failed\n");
    return -1;
  }
  drvdata->pio_reg_base = (uint64_t *) pcim_iomap(pdev, 0, 65536);
  printk("pci_resource_start(dev, 0) = %llx\n", (u64) pci_resource_start(pdev, 0));
  printk("BAR 0 base (remapped) = %llx\n", (u64) drvdata->pio_reg_base);

  for(i=0; i<8; i++)
    printk("buf_in[%d] = %llx\n", i, (uint64_t) readq(&drvdata->to_pc_buffer[0][i]));
  for(i=0; i<8; i++)
    printk("buf_out[%d] = %llx\n", i, (uint64_t) readq(&drvdata->from_pc_buffer[0][i]));
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (u64) readq(&drvdata->pio_reg_base[i]));

  /* disable it */
  writeq(cpu_to_le64(0x0), &drvdata->pio_reg_base[REG_FROM_PC_DISABLE]);
  writeq(cpu_to_le64(0x0), &drvdata->pio_reg_base[REG_TO_PC_DISABLE]);
  /* set interrupt match registers */
  writeq(0x1000 >> 2, &drvdata->pio_reg_base[REG_TO_PC_MATCH]);
  writeq(256*1024, &drvdata->pio_reg_base[REG_FROM_PC_MATCH]); 
  /* enable interrupts */
  writeq(cpu_to_le64(0xF), &drvdata->pio_reg_base[REG_INTERRUPT]);
  /* load the card page tables */
  for(i=0; i<32; i++){
    writeq(cpu_to_le64(drvdata->to_pc_dma_addr[i%BUFFER_PAGES_TO_PC]), &drvdata->pio_reg_base[i+REG_TO_PC_PAGE_TABLE_BASE]);
    writeq(cpu_to_le64(drvdata->from_pc_dma_addr[i%BUFFER_PAGES_FROM_PC]), &drvdata->pio_reg_base[i+REG_FROM_PC_PAGE_TABLE_BASE]);
  }
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&drvdata->pio_reg_base[i]));
  /* enable it */
  writeq(0x1000, &drvdata->pio_reg_base[REG_TO_PC_ENABLE]);
  writeq(4*1024*1024, &drvdata->pio_reg_base[REG_FROM_PC_ENABLE]); // enable
  udelay(1000);
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&drvdata->pio_reg_base[i]));
  udelay(1000);
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&drvdata->pio_reg_base[i]));
  for(i=0; i<32; i++)
    printk("buf_in[%d] = %llx\n", i, (uint64_t) readq(&drvdata->to_pc_buffer[0][i]));
  udelay(10000);
  printk("check buf\n");
  check_buffer(drvdata->from_pc_buffer);
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&drvdata->pio_reg_base[i]));
  return 0;
}

static void vna_dsp_remove(struct pci_dev *pdev){
  //unregister_chrdev(vna_dsp_major, DEVICE_NAME);
  device_destroy(vna_class, MKDEV(vna_dsp_major, 0));
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
