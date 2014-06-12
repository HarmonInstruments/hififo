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

#define BUFFER_PAGES_OUT 4
#define BUFFER_PAGES_IN 16
#define BUFFER_MAX_PAGES 32
#define BUFFER_PAGE_SIZE (4096*1024) // this is the size of a page in the card's page table

#define REG_INTERRUPT 0

static dma_addr_t buffer_out_dma_addr[BUFFER_PAGES_OUT];
static uint64_t *buffer_out[BUFFER_PAGES_OUT];

static dma_addr_t buffer_in_dma_addr[BUFFER_PAGES_IN];
static uint64_t *buffer_in[BUFFER_PAGES_IN];

volatile static uint64_t *bar0base = 0;
static int vna_dsp_major = 0;
static int vna_dsp_is_open = 0;
static struct class *vna_class;

struct hififo_dev {
  struct cdev hififo_cdev;
  dma_addr_t to_pc_dma_addr[BUFFER_MAX_PAGES];
  dma_addr_t from_pc_dma_addr[BUFFER_MAX_PAGES];
  u64 *to_pc_buffer[BUFFER_MAX_PAGES];
  u64 *from_pc_buffer[BUFFER_MAX_PAGES];
  wait_queue_head_t queue_read;
  int a;
};

static int vna_dsp_open(struct inode *inode, struct file *filp){
  struct hififo_dev *dev = container_of(inode->i_cdev, struct hififo_dev, hififo_cdev);
  filp->private_data = dev;
  printk("hififo: open, a = %d\n", dev->a);
  if (vna_dsp_is_open)
    return -EBUSY;
  vna_dsp_is_open++;
  try_module_get(THIS_MODULE); // increment the usage count

  return SUCCESS;
}

static int vna_dsp_release(struct inode *inode, struct file *filp){
  struct hififo_dev *dev = filp->private_data;
  vna_dsp_is_open--;/* We're now ready for our next caller */
  module_put(THIS_MODULE); // decrement the usage count
  printk("hififo: close, a = %d\n", dev->a);
  return SUCCESS;
}

static ssize_t vna_dsp_read(struct file *filp,/* see include/linux/fs.h   */
			    char *buffer,/* buffer to fill with data */
			    size_t length,/* length of the buffer     */
			    loff_t * offset){
  struct hififo_dev *dev = filp->private_data;
  if((length&0x7) != 0) /* reads must be a multiple of 8 bytes */
    return -EINVAL;
  //bytes_left = copy_to_user(buffer, vna_dsp_buffer, length);
  //status = wait_event_interruptible_timeout(queue_read, condition, HZ/4); // retval: 0 if timed out, else number of jiffies remaining
  printk("hififo: read, a = %d\n", dev->a++);
  return 0; // bytes remaining
}

static ssize_t vna_dsp_write(struct file *filp,
			    const char *buf,
			    size_t length,
			    loff_t * off){
  struct hififo_dev *dev = filp->private_data;
  int retval;
  printk("hififo: write, a = %d\n", dev->a++);
  length &= 0xFFFFFFFFF8;
  if((length&0x7) != 0) /* writes must be a multiple of 8 bytes */
    return -EINVAL;
  retval = copy_from_user(buffer_out, buf, length);
  writeq(length >> 3, &bar0base[11]); // enable
  printk(KERN_ALERT "Sorry, this operation isn't supported.\n");
  return -EINVAL;
}

static struct file_operations fops = {
 .read = vna_dsp_read,
 .write = vna_dsp_write,
 .open = vna_dsp_open,
 .release = vna_dsp_release
};

static irqreturn_t vna_interrupt(int irq, void *dev_id, struct pt_regs *regs){
  struct hififo_dev *dev = dev_id;
  //wake_up_all(&dev->queue_read);
  printk("VNA interrupt: sr = %llx, a = %d\n", (uint64_t) readq(&bar0base[REG_INTERRUPT]), dev->a);
  return IRQ_HANDLED;
}

static void check_buffer(void)
{
  int i;
  uint64_t prev = 0;
  uint64_t cur;
  int printcount = 0;
  for(i=0; i<512*1024; i++){
    cur = readq(&buffer_in[0][i]);
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
   
  for(i=0; i<BUFFER_PAGES_OUT; i++){
    buffer_out[i] = pci_alloc_consistent(pdev, BUFFER_PAGE_SIZE, &buffer_out_dma_addr[i]);
    printk("allocated buffer_out[%d] at %16llx phys: %16llx\n", i, (uint64_t) buffer_out[i], buffer_out_dma_addr[i]);
    if(buffer_out[i] == NULL){
      printk("Failed to allocate out buffer %d\n", i);
      return -ENOMEM;
    }
    else{
      for(j=0; j<BUFFER_PAGE_SIZE/8; j++)
	buffer_out[i][j] = j + i*512*1024;
    }
  }
  for(i=0; i<BUFFER_PAGES_IN; i++){
    buffer_in[i] = pci_alloc_consistent(pdev, BUFFER_PAGE_SIZE, &buffer_in_dma_addr[i]);
    printk("allocated buffer_in[%d] at %llx phys: %llx\n", i, (uint64_t) buffer_in[i], buffer_in_dma_addr[i]);
    if(buffer_in[i] == NULL){
      printk("Failed to allocate in buffer %d\n", i);
      return -ENOMEM;
    }
    else{
      for(j=0; j<BUFFER_PAGE_SIZE/8; j++)
	buffer_in[i][j] = 0xAAAAAAAA + ((u64) j << 32);
    }
  }

  if(pci_enable_msi(pdev) < 0)
    printk("VNA: pci_enable_msi() failed\n"); // check retval
  if(devm_request_irq(&pdev->dev, pdev->irq, (irq_handler_t) vna_interrupt, 0 /* flags */, DEVICE_NAME, drvdata) != 0)
    printk("VNA: request_irq() failed\n");
  
  bar0base = (uint64_t *) pcim_iomap(pdev, 0, 65536);
  printk("pci_resource_start(dev, 0) = %llx\n", (uint64_t) pci_resource_start(pdev, 0));
  printk("BAR 0 base (remapped) = %llx\n", (uint64_t) bar0base);
  
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&bar0base[i]));
  
  // set in match
  writeq(0x1000 >> 2, &bar0base[10]);
  // set out match
  writeq(256*1024, &bar0base[13]); 
  // enable interrupts
  writeq(0xF, &bar0base[0]);
  writeq(0, &bar0base[12]);
  // load the page tables
  for(i=0; i<32; i++){
    writeq(cpu_to_le64(buffer_in_dma_addr[i%BUFFER_PAGES_IN]), &bar0base[i+512]);
    writeq(buffer_out_dma_addr[i%BUFFER_PAGES_OUT], &bar0base[i+64]);
  }
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&bar0base[i]));
  // disable it
  writeq(0, &bar0base[9]);
  udelay(1000);
  // enable it
  writeq(0, &bar0base[8]);
  // write some data to the FIFO
  udelay(1000);
  
  for(i=0; i<8; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&bar0base[i]));
  for(i=0; i<8; i++)
    printk("buf_in[%d] = %llx\n", i, (uint64_t) readq(&buffer_in[0][i]));
  for(i=0; i<8; i++)
    printk("buf_out[%d] = %llx\n", i, (uint64_t) readq(&buffer_out[0][i]));
  
  writeq(4*1024*1024, &bar0base[11]); // enable
  udelay(1000);
  for(i=0; i<32; i++)
    printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&bar0base[i]));
  for(i=0; i<64; i++)
    printk("buf_in[%d] = %llx\n", i, (uint64_t) readq(&buffer_in[0][i]));
  udelay(10000);
  printk("check buf 2\n");
  check_buffer();
  return 0;
}

static void vna_dsp_remove(struct pci_dev *pdev){
  unregister_chrdev(vna_dsp_major, DEVICE_NAME);
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
