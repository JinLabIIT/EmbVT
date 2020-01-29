/**
 * Virtual time kernel module for heterogeneous distributed embedded linux.
 * Authors: Christopher Hannon, Jiaqi Yan, Brian Liu.
 */
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/timer.h>

#include "vtgpio.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jin Lab");
MODULE_DESCRIPTION("Distributed virtual time synchronization module");
MODULE_VERSION("0.5");

static unsigned int gpio_sig1 = 7;  // Using CE1 to output high or low
static unsigned int gpio_sig2 = 8;  // Listen to rising edge (pause)
static unsigned int gpio_sig3 = 24; // Listen to falling edge (resume)

static unsigned int irq_num1;
static unsigned int irq_num2;

static enum modes mode = DISABLED;
static int all_pid_nrs[MAX_NUM_PIDS] = {0};

static int sequential_io(enum IO io);
static int sequential_io_round_robin(enum IO io);

// PID variables for tracking processes in VT.
static int pid_01 = 0;
static int pid_02 = 0;
static int pid_03 = 0;
static int pid_04 = 0;
static int pid_05 = 0;
static int pid_06 = 0;
static int pid_07 = 0;
static int pid_08 = 0;
static int pid_09 = 0;
static int pid_10 = 0;
static int pid_11 = 0;
static int pid_12 = 0;
static int pid_13 = 0;
static int pid_14 = 0;
static int pid_15 = 0;
static int pid_16 = 0;

// Variable to ensure round robin pause/resume ops.
unsigned int round_robin = 0;
// Default time dilation factor for clocks (x1000).
static int tdf = 1000;
// Time in nano seconds when processes are frozen.
static s64 freeze_now = 0;
// Name of filesystem accessable from user space.
static char vt_name[6] = "vtXXX";

static uint period = 0;
module_param(period, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(period, "Period in milliseconds for periodic synchronization requests");

struct timer_list vt_timer;
static unsigned long leftover = 0; // Leftover time for rescheduled vt_timer.

/**
 * VT Timer's callback function.
 */
static void vt_timer_handler(unsigned long data) {
  if (mode == DISABLED) {
    mode = ENABLED;
    // so that we know whether we resume from interrupt or periodic event.
    leftover = 0;
    pause();
  }
}

/**
 * @brief Core function for pausing processes.
 */
void pause(void) {
#ifdef BENCHMARK
  unsigned long long oh_secs;
  unsigned long long oh_nsecs;
  struct timespec seconds;
  struct timespec seconds_end;

  getnstimeofday(&seconds);

#ifndef QUIET
  VT_PRINTK("VT_PAUSE\n");
  VT_PRINTK("VT-GPIO_TIME: TIME-RISE: %llu %llu nanoseconds",
            (unsigned long long)seconds.tv_sec,
            (unsigned long long)seconds.tv_nsec);
  VT_PRINTK("VT-GPIO: Rising Edge detected");
#endif // QUIET
#endif // BENCHMARK

#ifdef ROUND_ROBIN
  sequential_io_round_robin(FREEZE);
#else
  sequential_io(FREEZE);
#endif // ROUND_ROBIN

#ifdef BENCHMARK
  getnstimeofday(&seconds_end);

  if (seconds_end.tv_nsec > seconds.tv_nsec) {
    oh_secs = seconds_end.tv_sec - seconds.tv_sec;
    oh_nsecs = seconds_end.tv_nsec - seconds.tv_nsec;
  } else {
    oh_secs = seconds_end.tv_sec - 1 - seconds.tv_sec;
    oh_nsecs = seconds_end.tv_nsec - (SEC_NSEC + seconds.tv_nsec);
  }
  printk(KERN_INFO "VT-GPIO_BENCHMARK: Pause ; %llu ; %llu ",
         ((unsigned long long)oh_secs), ((unsigned long long)oh_nsecs));
#ifndef QUIET
  VT_PRINTK("VT-GPIO_TIME: TIME-RISE: %llu %llu nanoseconds",
            (unsigned long long)seconds_end.tv_sec,
            (unsigned long long)seconds_end.tv_nsec);
  VT_PRINTK("VT-GPIO_TIME: TIME-PAUSE: %llu %llu nanoseconds",
            ((unsigned long long)seconds_end.tv_sec -
             (unsigned long long)seconds.tv_sec),
            ((unsigned long long)seconds_end.tv_nsec -
             (unsigned long long)seconds.tv_nsec));
#endif // QUIET
#endif // BENCHMARK
}

/**
 * @brief Function to resume processes in VT.
 */
void resume(void) {
#ifdef BENCHMARK
  unsigned long long oh_secs;
  unsigned long long oh_nsecs;
  struct timespec seconds;
  struct timespec seconds_end;

  getnstimeofday(&seconds);

#ifndef QUIET
  VT_PRINTK("VT_RESUME\n");
  VT_PRINTK("VT-GPIO_TIME: TIME-FALL: %llu %llu nanoseconds",
            (unsigned long long)seconds.tv_sec,
            (unsigned long long)seconds.tv_nsec);
  VT_PRINTK("VT-GPIO: Falling Edge detected");
#endif // QUIET
#endif // BENCHMARK

#ifdef ROUND_ROBIN
  sequential_io_round_robin(RESUME);
#else
  sequential_io(RESUME);
#endif // ROUND_ROBIN

#ifdef BENCHMARK
  getnstimeofday(&seconds_end);

  if (seconds_end.tv_nsec > seconds.tv_nsec) {
    oh_secs = seconds_end.tv_sec - seconds.tv_sec;
    oh_nsecs = seconds_end.tv_nsec - seconds.tv_nsec;
  } else {
    oh_secs = seconds_end.tv_sec - 1 - seconds.tv_sec;
    oh_nsecs = seconds_end.tv_nsec - (SEC_NSEC + seconds.tv_nsec);
  }
  printk(KERN_INFO "VT-GPIO_BENCHMARK: Resume ; %llu ; %llu ",
         ((unsigned long long)oh_secs), ((unsigned long long)oh_nsecs));
#ifndef QUIET
  VT_PRINTK("VT-GPIO_TIME: TIME-FALL: %llu %llu nanoseconds",
            (unsigned long long)seconds_end.tv_sec,
            (unsigned long long)seconds_end.tv_nsec);
  VT_PRINTK("VT-GPIO_TIME: TIME-RESUME: %llu %llu nanoseconds",
            ((unsigned long long)seconds_end.tv_sec -
             (unsigned long long)seconds.tv_sec),
            ((unsigned long long)seconds_end.tv_nsec -
             (unsigned long long)seconds.tv_nsec));
#endif // QUIET
#endif // BENCHMARK
}

// Helper functions for writing sysfs fields.
static struct file *file_open(const char *path, int flags, int rights) {
  struct file *filp = NULL;
  mm_segment_t oldfs;
  int err = 0;

  oldfs = get_fs();
  set_fs(get_ds());
  filp = filp_open(path, flags, rights);
  set_fs(oldfs);
  if (IS_ERR(filp)) {
    err = PTR_ERR(filp);
    return NULL;
  }
  return filp;
}

static void file_close(struct file *file) {
  filp_close(file, NULL);
}

static int file_write(struct file *file,
               unsigned long long offset,
               unsigned char *data,
               unsigned int size) {
  mm_segment_t oldfs;
  int ret;

  oldfs = get_fs();
  set_fs(get_ds());

  ret = vfs_write(file, data, size, &offset);

  set_fs(oldfs);
  return ret;
}

static int file_sync(struct file *file) {
  vfs_fsync(file, 0);
  return 0;
}

static int write_proc_field(pid_t pid, char *field, char *val) {
  int ret;
  struct file *proc_file;
  char path[PATH_MAX];
  size_t val_len = strlen(val);

  sprintf(path, "/proc/%d/%s", pid, field);
  proc_file = file_open(path, O_WRONLY, 0);
  if (!proc_file) {
    printk(KERN_INFO "VT-GPIO_ERROR: can not open %s\n", path);
    return -1;
  }

  ret = file_write(proc_file, 0, val, val_len);

  if (ret < 0) {
    printk(KERN_INFO "VT-GPIO_ERROR: can not write %s\n", path);
    return -1;
  }
  ret = file_sync(proc_file);
  if (ret < 0) {
    printk(KERN_INFO "VT-GPIO_ERROR: can not sync %s\n", path);
    return -1;
  }
  file_close(proc_file);

  return 0;
}

/**
 * @brief Function to add pids to VT.
 */
static int dilate_proc(int pid) {
  int ret = 0;
  char tdf_str[TDF_STR_LEN];

  ret = sprintf(tdf_str, "%d", tdf);
  write_proc_field((pid_t)pid, "dilation", tdf_str);
  VT_PRINTK("VT-GPIO: Dilating %d\n", pid);
  return ret;
}

/**
 * @brief Hold a read lock, and get pid structs from process numbers.
 * Caller must allocate results with MAX_NUM_PIDS NULLs.
 */
static size_t all_pids_from_nrs(struct pid *results[]) {
  size_t cnt = 0, i = 0;
  rcu_read_lock();
  for (i = 0; i < MAX_NUM_PIDS; ++i) {
    if (all_pid_nrs[i] == 0) break;
    results[cnt] = find_vpid(all_pid_nrs[i]);
    cnt += (results[cnt] != NULL);
  }
  rcu_read_unlock();
  return cnt;
}

static int sequential_io(enum IO io) {
  int rc = 0;
  size_t num_procs, i;
  struct timespec ts;
  struct task_struct *tsk = NULL;
  s64 freeze_duration;
  struct pid *pids[MAX_NUM_PIDS] = {NULL};

  switch (io) {
  case DILATE:
    for (i = 0; i < MAX_NUM_PIDS; ++i) {
      if (all_pid_nrs[i] == 0) break;
      dilate_proc(all_pid_nrs[i]);
    }
    break;
  case FREEZE:
    num_procs = all_pids_from_nrs(pids);
    for (i = 0; i < num_procs; ++i) {
      rc = kill_pid(pids[i], SIGSTOP, 1);
      if (rc != 0) {
        VT_PRINTK("VT-GPIO: Fail to SIGSTOP %d\n", all_pid_nrs[i]);
      }
    }
    __getnstimeofday(&ts);
    freeze_now = timespec_to_ns(&ts);
    break;
  case RESUME:
    __getnstimeofday(&ts);
    freeze_duration = timespec_to_ns(&ts) - freeze_now;
    
    num_procs = all_pids_from_nrs(pids);
    for (i = 0; i < num_procs; ++i) {
      rcu_read_lock();
      tsk = pid_task(pids[i], PIDTYPE_PID);
      rcu_read_unlock();
      if (tsk) tsk->freeze_past_nsec += freeze_duration;
    }
    for (i = 0; i < num_procs; ++i) {
      rc = kill_pid(pids[i], SIGCONT, 1);
      if (rc != 0) {
        VT_PRINTK("VT-GPIO: Fail to SIGCONT %d\n", all_pid_nrs[i]);
      }
    }
    break;
  }
  return 0;
}

static int sequential_io_round_robin(enum IO io) {
  int rc = 0;
  size_t num_procs, i, c;
  struct timespec ts;
  struct task_struct *tsk = NULL;
  s64 freeze_duration;
  struct pid *pids[MAX_NUM_PIDS] = {NULL};

  switch (io) {
  case DILATE:
    for (i = 0; i < MAX_NUM_PIDS; i++) {
      if (all_pid_nrs[i]) dilate_proc(all_pid_nrs[i]);
      else break;
    }
    break;
  case FREEZE:
    num_procs = all_pids_from_nrs(pids);
    for (i = round_robin, c = 0; c < num_procs; i = (i + 1) % num_procs, ++c) {
      rc = kill_pid(pids[i], SIGSTOP, 1);
      if (rc != 0) {
        VT_PRINTK("VT-GPIO: Fail to SIGSTOP %d\n", all_pid_nrs[i]);
      }
    }
    __getnstimeofday(&ts);
    freeze_now = timespec_to_ns(&ts);
    break;
  case RESUME:
    __getnstimeofday(&ts);
    freeze_duration = timespec_to_ns(&ts) - freeze_now;

    num_procs = all_pids_from_nrs(pids);
    for (i = round_robin, c = 0; c < num_procs; i = (i + 1) % num_procs, ++c) {
      rcu_read_lock();
      tsk = pid_task(pids[i], PIDTYPE_PID);
      rcu_read_unlock();
      if (tsk) tsk->freeze_past_nsec += freeze_duration;
    }
    for (i = round_robin, c = 0; c < num_procs; i = (i + 1) % num_procs, ++c) {
      rc = kill_pid(pids[i], SIGCONT, 1);
      if (rc != 0) {
        VT_PRINTK("VT-GPIO: Fail to SIGCONT %d\n", all_pid_nrs[i]);
      }
    }
    round_robin = (round_robin + 1) % num_procs;
    break;
  }
  return 0;
}

/**
 * @brief Callback function to display the vt tdf.
 */
static ssize_t tdf_show(struct kobject *kobj,
                        struct kobj_attribute *attr,
                        char *buf) {
  return sprintf(buf, "%d\n", tdf);
}

/**
 * @brief Callback function to store the vt tdf.
 */
static ssize_t tdf_store(struct kobject *kobj,
                         struct kobj_attribute *attr,
                         const char *buf, size_t count) {
  int ret;
  ret = kstrtoint(buf, 10, &tdf);
  if (ret < 0) return ret;
  sequential_io(DILATE);  // Overwrite any existing tdf.
  return count;
}

static SHOW_HANDLER(01)
static SHOW_HANDLER(02)
static SHOW_HANDLER(03)
static SHOW_HANDLER(04)
static SHOW_HANDLER(05)
static SHOW_HANDLER(06)
static SHOW_HANDLER(07)
static SHOW_HANDLER(08)
static SHOW_HANDLER(09)
static SHOW_HANDLER(10)
static SHOW_HANDLER(11)
static SHOW_HANDLER(12)
static SHOW_HANDLER(13)
static SHOW_HANDLER(14)
static SHOW_HANDLER(15)
static SHOW_HANDLER(16)
static STORE_HANDLER(01, 0)
static STORE_HANDLER(02, 1)
static STORE_HANDLER(03, 2)
static STORE_HANDLER(04, 3)
static STORE_HANDLER(05, 4)
static STORE_HANDLER(06, 5)
static STORE_HANDLER(07, 6)
static STORE_HANDLER(08, 7)
static STORE_HANDLER(09, 8)
static STORE_HANDLER(10, 9)
static STORE_HANDLER(11, 10)
static STORE_HANDLER(12, 11)
static STORE_HANDLER(13, 12)
static STORE_HANDLER(14, 13)
static STORE_HANDLER(15, 14)
static STORE_HANDLER(16, 15)

/** 
 * @brief Callback function to display the vt mode.
 */
static ssize_t mode_show(struct kobject *kobj,
                         struct kobj_attribute *attr,
                         char *buf) {
  switch (mode) {
  case ENABLED:
    return sprintf(buf, "freeze\n");
  case DISABLED:
    return sprintf(buf, "unfreeze\n");
  default:
    return sprintf(buf, "LKM ERROR\n");
  }
}

/**
 * @brief Callback function to store the vt mode.
 */
static ssize_t mode_store(struct kobject *kobj,
                          struct kobj_attribute *attr,
                          const char *buf,
                          size_t count) {
  if (strncmp(buf, "freeze", count - 1) == 0) {
    mode = ENABLED;
#ifndef QUIET
    VT_PRINTK("VT-GPIO: pause\n");
#endif
    // vt has been triggered locally,
    // we need to quickly change to output mode
    gpio_direction_output(gpio_sig1, 1);
  } else if (strncmp(buf, "unfreeze", count - 1) == 0) {
    mode = DISABLED;
#ifndef QUIET
    VT_PRINTK("VT-GPIO: resume\n");
#endif
    // change cfg, go low
    gpio_direction_output(gpio_sig1, 0);
    gpio_direction_input(gpio_sig1);
  }
  return count;
}

static ssize_t syncpause_show(struct kobject *kojb,
                              struct kobj_attribute *attr,
                              char *buf) {
  if (mode == ENABLED && leftover == 0) {
    return sprintf(buf, "1");
  } else {
    return sprintf(buf, "0");
  }
}

static struct kobj_attribute mode_attr = __ATTR(mode, 0660, mode_show, mode_store);
static struct kobj_attribute tdf_attr = __ATTR(tdf, 0660, tdf_show, tdf_store);
static struct kobj_attribute syncpause_attr = __ATTR_RO(syncpause);

DECLARE_PID_ATTR(01);
DECLARE_PID_ATTR(02);
DECLARE_PID_ATTR(03);
DECLARE_PID_ATTR(04);
DECLARE_PID_ATTR(05);
DECLARE_PID_ATTR(06);
DECLARE_PID_ATTR(07);
DECLARE_PID_ATTR(08);
DECLARE_PID_ATTR(09);
DECLARE_PID_ATTR(10);
DECLARE_PID_ATTR(11);
DECLARE_PID_ATTR(12);
DECLARE_PID_ATTR(13);
DECLARE_PID_ATTR(14);
DECLARE_PID_ATTR(15);
DECLARE_PID_ATTR(16);

static struct attribute *vt_attrs[] = {
    &mode_attr.attr, &tdf_attr.attr, &syncpause_attr.attr,
    &pid_01_attr.attr, &pid_02_attr.attr, &pid_03_attr.attr, &pid_04_attr.attr,
    &pid_05_attr.attr, &pid_06_attr.attr, &pid_07_attr.attr, &pid_08_attr.attr,
    &pid_09_attr.attr, &pid_10_attr.attr, &pid_11_attr.attr, &pid_12_attr.attr,
    &pid_13_attr.attr, &pid_14_attr.attr, &pid_15_attr.attr, &pid_16_attr.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .name = vt_name,
    .attrs = vt_attrs,
};

static struct kobject *vt_kobj;

/**
 * @brief Software interrupt handler for rising signal.
 */
static irq_handler_t vtgpio_irq_handler(unsigned int irq, void *dev_id,
                                        struct pt_regs *regs) {
  trace_printk(KERN_INFO "rise\n");

  // Record how long until the timer expires, then cancel it.
  if (timer_pending(&vt_timer) && time_is_after_jiffies(vt_timer.expires)) {
    leftover = vt_timer.expires - jiffies;
    del_timer(&vt_timer);
  }
  gpio_direction_output(gpio_sig1, 1);
  //pause();

  return (irq_handler_t)IRQ_HANDLED;
}

/**
 * @brief Software interrupt handler for falling signal.
 */
static irq_handler_t vtgpio_irq_handler_fall(unsigned int irq, void *dev_id,
                                             struct pt_regs *regs) {
  trace_printk(KERN_INFO "fall\n");

  resume();
  if (period > 0) {
    if (leftover > 0) { // Resumed from interrupt event.
      vt_timer.expires = jiffies + leftover;
    } else { // Resumed from synchronization event.
      vt_timer.expires = jiffies + msecs_to_jiffies(period);
    }
    add_timer(&vt_timer);
  }
  return (irq_handler_t)IRQ_HANDLED;
}

static int __init vtgpio_init(void) {
  int result = 0;
  int res = 0;

  printk(KERN_INFO
         "VT-GPIO: Initializing the Virtual Time GPIO LKM\n");
  if (!gpio_is_valid(gpio_sig1)) {
    printk(KERN_INFO "VT-GPIO: pin %d not valid\n", gpio_sig1);
    return -ENODEV;
  }
  if (!gpio_is_valid(gpio_sig2)) {
    printk(KERN_INFO "VT-GPIO: pin %d not valid\n", gpio_sig2);
    return -ENODEV;
  }
  if (!gpio_is_valid(gpio_sig3)) {
    printk(KERN_INFO "VT-GPIO: pin %d not valid\n", gpio_sig3);
    return -ENODEV;
  }
  sprintf(vt_name, "VT%d", gpio_sig1);
  vt_kobj = kobject_create_and_add("vt", kernel_kobj->parent);

  if (!vt_kobj) {
    printk(KERN_ALERT "VT-GPIO: failed to create kobject\n");
    return -ENOMEM;
  }

  res = sysfs_create_group(vt_kobj, &attr_group);
  if (res) {
    printk(KERN_ALERT "VT-GPIO: failed to create sysfs group\n");
    kobject_put(vt_kobj);
    return res;
  }

  gpio_request(gpio_sig1, "sysfs");
  gpio_request(gpio_sig2, "sysfs");
  gpio_request(gpio_sig3, "sysfs");
  gpio_direction_input(gpio_sig1);  // default to input to listen
  gpio_direction_input(gpio_sig2);
  gpio_direction_input(gpio_sig3);
  gpio_set_debounce(gpio_sig1, DEBOUNCE_TIME);
  gpio_set_debounce(gpio_sig2, DEBOUNCE_TIME);
  gpio_set_debounce(gpio_sig3, DEBOUNCE_TIME);
  gpio_export(gpio_sig1, true);     // true = able to change direction
  gpio_export(gpio_sig2, true);
  gpio_export(gpio_sig3, true);

  irq_num1 = gpio_to_irq(gpio_sig3);
  printk(KERN_INFO "VT-GPIO: Input signal is mapped to IRQ: %d\n",
         irq_num1);
  irq_num2 = gpio_to_irq(gpio_sig2);
  printk(KERN_INFO "VT-GPIO: Input signal is mapped to IRQ: %d\n",
         irq_num2);

  result = request_irq(irq_num1, (irq_handler_t)vtgpio_irq_handler,
                       IRQF_TRIGGER_RISING, "vt_gpio_handler", NULL);
  printk(KERN_INFO "VT-GPIO: The interrupt rising request result is %d\n",
         result);

  result = request_irq(irq_num2, (irq_handler_t)vtgpio_irq_handler_fall,
                       IRQF_TRIGGER_FALLING, "vt_gpio_handler_fall", NULL);
  printk(KERN_INFO "VT-GPIO: The interrupt rising request result is %d\n",
         result);

  if (period > 0) {
    init_timer(&vt_timer);
    printk(KERN_INFO "VT-GPIO: period set to: %d\n", period);
    vt_timer.expires = jiffies + msecs_to_jiffies(period);;
    vt_timer.function = vt_timer_handler;
    vt_timer.data = 0;
    add_timer(&vt_timer);
    printk(KERN_INFO "VT-GPIO: Enable periodic mode with timer\n");
  }

  return result;
}

static void __exit vtgpio_exit(void) {
  printk(KERN_INFO "VT-GPIO: Exiting LKM\n");
  kobject_put(vt_kobj);
  gpio_unexport(gpio_sig1);
  gpio_unexport(gpio_sig2);
  gpio_unexport(gpio_sig3);

  free_irq(irq_num1, NULL);
  gpio_free(gpio_sig1);
  gpio_free(gpio_sig3);
  free_irq(irq_num2, NULL);
  gpio_free(gpio_sig2);

  if (period > 0 && timer_pending(&vt_timer)) {
    del_timer(&vt_timer);
  }

  printk(KERN_INFO "VT-GPIO: Successfully leaving LKM\n");
}

module_init(vtgpio_init);
module_exit(vtgpio_exit);
