// dummy raw1394 kernel module source code
// see readme.txt for information on how to use it

#include <linux/module.h>
#include <linux/kernel.h>

int init_module (void)
{
  printk (KERN_INFO "Loaded dummy raw1394 module\n");

  return 0;
}
