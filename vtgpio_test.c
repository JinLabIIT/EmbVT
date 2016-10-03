/*
    Virtual Time Kernel Module
    For Heterogeneous Distributed 
	Embedded Linux Environment

    Author: Christopher Hannon
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>        // :)
#include <linux/gpio.h>          // gpio stuff
#include <linux/interrupt.h>     // irq  code
#include <linux/kobject.h>
#include <linux/kthread.h>
//#include <linux/time.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Christopher Hannon");
MODULE_DESCRIPTION("Test for sync between two emb-lins for virtual time coordination");
MODULE_VERSION("0.1");

static unsigned int gpioSIG = 21; // pin for talking // gpio21 on model b+ 
// gpio 21 on 2B static 
unsigned int active = 1; //
//static unsigned int last_triggered; 
static unsigned int num_ints = 0; 
static unsigned int irqNumber;

static irq_handler_t vtgpio_irq_handler(unsigned int irq, void *dev_id, 
struct pt_regs *regs);
//static irq_handler_t vtgpio_irq_handler_fall(unsigned int irq, void *dev_id, 
//struct pt_regs *regs);

enum modes { DISABLED, ENABLED };
static enum modes mode = DISABLED;

static char vtName[6] = "vtXXX";

//static int pids[128];

/** @brief A callback function to display the vt mode */
static ssize_t mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
  switch(mode){
  case ENABLED: return sprintf(buf, "freeze\n");
  case DISABLED: return sprintf(buf, "unfreeze\n");
  default: return sprintf(buf, "LKM ERROR\n"); //whoops
  }
}

/** @brief A callback function to store the vt mode using enum*/
static ssize_t mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
  if(strncmp(buf,"freeze",count-1)==0) {
    mode = ENABLED;
    printk(KERN_INFO "VT-GPIO_TEST: pause\n");

    /* vt has been triggered locally */
    /*   we need to quickly  
	 ---------------------------
	 change to output mode,
	 go high on gpio
    */
    gpio_direction_output(gpioSIG,1);

    //printk(KERN_INFO "VT-GPIO_TEST: value of pin: %d\n", gpio_get_value(gpioSIG));
	 /*
	 
	 kickoff freeze
     */

    /* Stay HIGH until unfreeze is called */

    
    /* TODO */
    
  }
  else if (strncmp(buf,"unfreeze",count-1)==0) {
    mode = DISABLED;
    printk(KERN_INFO "VT-GPIO_TEST: resume\n");

    /* chgn cfg
       go low
       kickoff resume
    */
    gpio_direction_output(gpioSIG,0); 

    //printk(KERN_INFO "VT-GPIO_TEST: value of pin: %d\n", gpio_get_value(gpioSIG));

    gpio_direction_input(gpioSIG); 

    //printk(KERN_INFO "VT-GPIO_TEST: value of pin: %d\n", gpio_get_value(gpioSIG));
   
  }
      
  return count;
}

static struct kobj_attribute mode_attr = __ATTR(mode, 0660, mode_show, mode_store);

static struct attribute *vt_attrs[] = {
  &mode_attr.attr,
  NULL,
};

static struct attribute_group attr_group = {
  .name = vtName,
  .attrs = vt_attrs,
};

static struct kobject *vt_kobj;
//static struct task_struct * task


static int __init vtgpio_init(void) {
  int result=0;
  int res = 0;
  
  printk(KERN_INFO "VT-GPIO_TEST: Initializing the Virtual Time GPIO_TEST LKM\n");
  if(!gpio_is_valid(gpioSIG)) {
    printk(KERN_INFO "VT-GPIO_TEST: pin not valid\n");
    return -ENODEV;
  }
  sprintf(vtName, "VT%d", gpioSIG);
  vt_kobj = kobject_create_and_add("vt", kernel_kobj->parent);

  if(!vt_kobj){
    printk(KERN_ALERT "VT-GPIO_TEST: failed to create kobject\n");
    return -ENOMEM;
  }

  res = sysfs_create_group(vt_kobj, &attr_group);
  if(res){
    printk(KERN_ALERT "VT-GPIO_TEST: failed to create sysfs group\n");
    kobject_put(vt_kobj);
    return res;
  }
  
  gpio_request(gpioSIG, "sysfs");
  gpio_direction_input(gpioSIG); // default to input to listen
  gpio_export(gpioSIG, true);    // true = we should be able to change direction

  irqNumber = gpio_to_irq(gpioSIG);
  printk(KERN_INFO "VT-GPIO_TEST: Input signal is mapped to IRQ: %d\n", irqNumber);

  result = request_irq(irqNumber, (irq_handler_t) vtgpio_irq_handler,
		       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "vt_gpio_handler", NULL);
  printk(KERN_INFO "VT-GPIO_TEST: The interrupt rising request result is %d\n", result);

  return result;
}

static void __exit vtgpio_exit(void) {
  printk(KERN_INFO "VT-GPIO_TEST: Exiting LKM\n");
  kobject_put(vt_kobj);
  gpio_unexport(gpioSIG);
  free_irq(irqNumber, NULL);
  gpio_free(gpioSIG);
  printk(KERN_INFO "VT-GPIO_TEST: Successfully leaving LKM\n");
}

static irq_handler_t vtgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
  printk(KERN_INFO "VT-GPIO_TEST: Interrupt! (Im alive!)");
  num_ints ++;
  if(gpio_get_value(gpioSIG)){
    printk(KERN_INFO "VT-GPIO_TEST: Rising Edge detected");
  }
  else{
    printk(KERN_INFO "VT-GPIO_TEST: Falling Edge detected");
  }

  /* we have to sound the trumpets */
  /* read list of pids */



  /* kickoff kthreads to freeze processes */



  return (irq_handler_t) IRQ_HANDLED; // return that we all good
}




module_init(vtgpio_init);
module_exit(vtgpio_exit);

/* el fin */