#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>

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
#define BUFFER_PAGE_SIZE (4096*1024) // this is the size of a page in the card's page table

static dma_addr_t buffer_out_dma_addr[BUFFER_PAGES_OUT];
static uint64_t *buffer_out[BUFFER_PAGES_OUT];

static dma_addr_t buffer_in_dma_addr[BUFFER_PAGES_IN];
static uint64_t *buffer_in[BUFFER_PAGES_IN];

volatile static uint64_t *bar0base = 0;
static int vna_dsp_major = 0;
static int vna_dsp_is_open = 0;

static wait_queue_head_t queue_read;

static int vna_dsp_open(struct inode *inode, struct file *file){
  if (vna_dsp_is_open)
    return -EBUSY;
  vna_dsp_is_open++;
  try_module_get(THIS_MODULE); // increment the usage count
  return SUCCESS;
}

static int vna_dsp_release(struct inode *inode, struct file *file){
  vna_dsp_is_open--;/* We're now ready for our next caller */
  module_put(THIS_MODULE); // decrement the usage count
  return SUCCESS;
}

static ssize_t vna_dsp_read(struct file *filp,/* see include/linux/fs.h   */
			    char *buffer,/* buffer to fill with data */
			    size_t length,/* length of the buffer     */
			    loff_t * offset){
  //bytes_left = copy_to_user(buffer, vna_dsp_buffer, length);
  //status = wait_event_timeout(queue_read, condition, HZ/4); // retval: 0 if timed out, else number of jiffies remaining

  return 0; // bytes remaining
}

static ssize_t vna_dsp_write(struct file *filp,
			    const char *buff,
			    size_t len,
			    loff_t * off){
  //copy_from_user(dest, buff, count);

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
  //struct sample_dev *dev = dev_id;
  wake_up_all(&queue_read);
  printk("VNA interrupt: sr = %llx\n", (uint64_t) readq(&bar0base[0]));
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

static int vna_dsp_probe(struct pci_dev *dev, const struct pci_device_id *id){
  int i, j;
  int retval = pci_enable_device(dev);
  if(retval < 0) return retval;
  retval = pci_request_regions(dev, DEVICE_NAME);
  if(retval < 0){
    pci_disable_device(dev);
    return retval;
  }
  printk("Found Harmon Instruments PCI Express interface board\n");

  pci_set_master  (dev);
  pci_set_dma_mask(dev, 0xFFFFFFFFFFFFFFFF);
  pci_set_consistent_dma_mask(dev, 0xFFFFFFFFFFFFFFFF);

  //pci_set_dma_mask(dev, 0xFFFFFFFF); // if this fails, try 32
  //pci_set_consistent_dma_mask(dev, 0xFFFFFFFF);
  
  init_waitqueue_head(&queue_read);
  
  for(i=0; i<BUFFER_PAGES_OUT; i++){
    buffer_out[i] = pci_alloc_consistent(dev, BUFFER_PAGE_SIZE, &buffer_out_dma_addr[i]);
    printk("allocated buffer_out[%d] at %16llx phys: %16llx\n", i, (uint64_t) buffer_out[i], buffer_out_dma_addr[i]);
    if(buffer_out[i] == NULL)
      printk("Failed to allocate out buffer %d\n", i);
    else{
      for(j=0; j<BUFFER_PAGE_SIZE/8; j++)
	buffer_out[i][j] = j + i*512*1024;
    }
  }
  for(i=0; i<BUFFER_PAGES_IN; i++){
    buffer_in[i] = pci_alloc_consistent(dev, BUFFER_PAGE_SIZE, &buffer_in_dma_addr[i]);
    printk("allocated buffer_in[%d] at %llx phys: %llx\n", i, (uint64_t) buffer_in[i], buffer_in_dma_addr[i]);
    if(buffer_in[i] == NULL)
      printk("Failed to allocate in buffer %d\n", i);
    else{
      for(j=0; j<BUFFER_PAGE_SIZE/8; j++)
	buffer_in[i][j] = 0xAAAAAAAA + ((u64) j << 32);
    }
  }

  if(pci_enable_msi(dev) < 0)
    printk("VNA: pci_enable_msi() failed\n"); // check retval
  if(request_irq(dev->irq, (irq_handler_t) vna_interrupt, 0 /* flags */, DEVICE_NAME, dev) != 0)
    printk("VNA: request_irq() failed\n");
  
  bar0base = (uint64_t *) ioremap(pci_resource_start(dev, 0), 65536);
  printk("pci_resource_start(dev, 0) = %llx\n", (uint64_t) pci_resource_start(dev, 0));
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
  for(i=0; i<32; i++)
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

static void vna_dsp_remove(struct pci_dev *dev){
  int i;
  iounmap(bar0base);
  free_irq(dev->irq, dev); // void
  pci_disable_msi(dev); // check retval?
  for(i=0; i<BUFFER_PAGES_OUT; i++){
    if(buffer_out[i])
      pci_free_consistent(dev, BUFFER_PAGE_SIZE, (void *) buffer_out[i], buffer_out_dma_addr[i]);
  }
  for(i=0; i<BUFFER_PAGES_IN; i++){
    if(buffer_in[i])
      pci_free_consistent(dev, BUFFER_PAGE_SIZE, (void *) buffer_in[i], buffer_in_dma_addr[i]);
  }
  pci_release_regions(dev);
  pci_disable_device(dev);
}

static struct pci_driver vna_dsp_driver = {
  .name = "vna_dsp",
  .id_table = vna_dsp_pci_table,
  .probe = vna_dsp_probe,
  .remove = vna_dsp_remove,
};

static struct class *vna_class;

static int __init vna_dsp_init(void){
  printk ("Loading vna_dsp kernel module\n");
  vna_dsp_major = register_chrdev(0, DEVICE_NAME, &fops);
  if(vna_dsp_major < 0){
    printk(KERN_ALERT "Registering char device failed with %d\n", vna_dsp_major);
    return vna_dsp_major;
  }
  vna_class = class_create(THIS_MODULE, "vna");
  if (IS_ERR(vna_class)) {
    printk(KERN_ERR "Error creating vna class.\n");
    //goto error;
  }
  device_create(vna_class, NULL, MKDEV(vna_dsp_major, 0), NULL, "vna");
  
  return pci_register_driver(&vna_dsp_driver);
}

static void __exit vna_dsp_exit(void){
  pci_unregister_driver(&vna_dsp_driver);
  unregister_chrdev(vna_dsp_major, DEVICE_NAME);
  device_destroy(vna_class, MKDEV(vna_dsp_major, 0));
  class_destroy(vna_class);
  printk ("Unloading vna_dsp kernel module.\n");
  return;
}

module_init(vna_dsp_init);
module_exit(vna_dsp_exit);

MODULE_LICENSE("GPL");
