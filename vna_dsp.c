#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>

#define VENDOR_ID 0x10EE
#define DEVICE_ID 0x7024
#define SUCCESS 0
#define DEVICE_NAME "vna_dsp"

static struct pci_device_id vna_dsp_pci_table[] = {
  {VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0 ,0},
  {0,}
};

static dma_addr_t dmahandle_buffer;
static uint64_t *vna_dsp_buffer;
volatile static uint64_t *bar0base = 0;
static int vna_dsp_major = 0;
static int vna_dsp_is_open = 0;

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
  return 0;
}

static ssize_t vna_dsp_read(struct file *filp,/* see include/linux/fs.h   */
			    char *buffer,/* buffer to fill with data */
			    size_t length,/* length of the buffer     */
			    loff_t * offset){
  unsigned long bytes_left;
  if(length > 4*1024*1024)
    length = 4*1024*1024;
  bytes_left = copy_to_user(buffer, vna_dsp_buffer, length);
  return length - bytes_left;
}

static ssize_t vna_dsp_write(struct file *filp,
			    const char *buff,
			    size_t len,
			    loff_t * off){
  printk(KERN_ALERT "Sorry, this operation isn't supported.\n");
  bar0base[1] = 0;
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
  /* now `dev' points to the right hardware item */
  /* .... */
  printk("VNA interrupt\n");
  return IRQ_HANDLED;
}


static int vna_dsp_probe(struct pci_dev *dev, const struct pci_device_id *id){
  int i;
  int retval = pci_enable_device(dev);
  if(retval < 0) return retval;
  retval = pci_request_regions(dev, DEVICE_NAME);
  if(retval < 0){
    pci_disable_device(dev);
    return retval;
  }
  pci_set_master  (dev);
  pci_set_dma_mask(dev, 0xFFFFFFFFFFFFFFFF); // if this fails, try 32
  pci_set_consistent_dma_mask(dev, 0xFFFFFFFFFFFFFFFF);
  if(pci_enable_msi(dev) < 0)
    printk("VNA: pci_enable_msi() failed\n"); // check retval
  if(request_irq(dev->irq, (irq_handler_t) vna_interrupt, 0 /* flags */, DEVICE_NAME, dev) != 0)
    printk("VNA: request_irq() failed\n");
  vna_dsp_buffer = (uint64_t *) pci_alloc_consistent(dev, 4*1024*1024, &dmahandle_buffer);
  if(vna_dsp_buffer == NULL)
    printk("Failed to allocate rx buffer\n");
  printk("Allocated a buffer at %llx %llx\n", (uint64_t) vna_dsp_buffer, (uint64_t) dmahandle_buffer);
  printk("Found PCI Express interface board\n");
  bar0base = (uint64_t *) ioremap(pci_resource_start(dev, 0), 65536);
  printk("pci_resource_start(dev, 0) = %llx\n", (uint64_t) pci_resource_start(dev, 0));
  printk("BAR 0 base (remapped) = %llx\n", (uint64_t) bar0base);
  writeq(0x0BADC0DE, &bar0base[1]);
  udelay(1000);
  for(i=0; i<15; i++)
    {
      printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&bar0base[i]));
      writeq(i, &bar0base[15]);
    }
  writeq(0xDEADBEEF0BADC0DE, &bar0base[0]);
  for(i=0; i<256; i++)
    vna_dsp_buffer[i] = 0xBEEF00000000 | i;
  for(i=0; i<8; i++)
    {
      printk("dmabuf[%d] = %llx\n", i, (uint64_t) readq(&vna_dsp_buffer[i]));
      // vna_dsp_buffer[i] = 0xDEAD;
    }
  writeq(dmahandle_buffer, &bar0base[101]);
  udelay(1000);
  writeq(dmahandle_buffer, &bar0base[100]);
  udelay(1000);
  for(i=0; i<32; i++)
    {
      printk("dmabuf[%d] = %llx\n", i, (uint64_t) readq(&vna_dsp_buffer[i]));
    }
  for(i=0; i<16; i++)
    {
      //if((readq(&bar0base[i]) & (uint64_t) 0x0FFFFFFFF) != 0xDEADBEEF)
      printk("bar0[%d] = %llx\n", i, (uint64_t) readq(&bar0base[i]));
    }
  for(i=0; i<16; i++)
    {
      writeq(0x1234, &bar0base[0]);
      //if((readq(&bar0base[i]) & (uint64_t) 0x0FFFFFFFF) != 0xDEADBEEF)
      printk("bar0[4] = %llx\n", (uint64_t) readq(&bar0base[3]));
    }
  writeq(dmahandle_buffer, &bar0base[102]);
  return 0;
}

static void vna_dsp_remove(struct pci_dev *dev){
  iounmap(bar0base);
  free_irq(dev->irq, dev); // void
  pci_disable_msi(dev); // check retval?
  pci_free_consistent(dev, 4*1024*1024, vna_dsp_buffer, dmahandle_buffer);
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
  printk ("Assigned major = %d\n", vna_dsp_major);
  vna_class = class_create(THIS_MODULE, "vna");
  if (IS_ERR(vna_class)) {
    printk(KERN_ERR "Error creating vna class.\n");
    //goto error;
  }
  device_create(vna_class, NULL, MKDEV(vna_dsp_major, 0), NULL, "vna");
  
  //devfs_register_chrdev(vna_dsp_major, DEVICE_NAME, &fops);
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
