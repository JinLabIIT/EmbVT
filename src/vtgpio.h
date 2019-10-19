#ifndef __VTGPIO_H
#define __VTGPIO_H

#define TDF_STR_LEN 8
#define DEBOUNCE_TIME 0.02
#define MAX_NUM_PIDS 16

#define SEC_NSEC 1000000000 // 10^9 nsec in sec.

#ifndef BENCHMARK
#define BENCHMARK
#endif // #ifndef BENCHMARK

// Disable round robin pause/resume scheduling.
#ifdef ROUND_ROBIN
#undef ROUND_ROBIN
#endif

#define FTRACE 0 // Value to toggle ftrace vs regular prints.
#define QUIET 1 // value to surpress real-time pause/resume.

// Functions for pausing resuming processes and clocks.
void pause(void);
void resume(void);

int write_proc_field(pid_t pid, char *field, char *val);

/*support functions for writing to fields */
struct file *file_open(const char *path, int flags, int rights);
void file_close(struct file *file);
int file_read(struct file *file,
              unsigned long long offset,
              unsigned char *data,
              unsigned int size);
int file_write(struct file *file,
               unsigned long long offset,
               unsigned char *data,
               unsigned int size);
int file_sync(struct file *file);

// Varible in sysfs to indicate if processes are frozen.
enum modes { DISABLED, ENABLED };

// Types of operations in sequencial IO.
enum IO { RESUME, FREEZE, DILATE };

// Macros for generating repeated code.
#define VT_PRINTK(fmt, ...)                                         \
  do {                                                              \
    if (FTRACE) {                                                   \
      trace_printk(KERN_INFO fmt, __VA_ARGS__);                     \
    } else {                                                        \
      printk(KERN_INFO fmt, __VA_ARGS__);                           \
    }                                                               \
  } while (0)

#define SHOW_HANDLER(IDX)                                           \
  ssize_t pid_##IDX##_show(struct kobject *kobj,                    \
                           struct kobj_attribute *attr,             \
                           char *buf) {                             \
    return sprintf(buf, "%d\n", pid_##IDX);                         \
  }

#define STORE_HANDLER(IDX, IDXI)                                    \
  ssize_t pid_##IDX##_store(struct kobject *kobj,                   \
                            struct kobj_attribute *attr,            \
                            const char *buf, size_t count) {        \
    int ret;                                                        \
    ret = kstrtoint(buf, 10, &pid_##IDX);                           \
    if (ret < 0) {                                                  \
      return ret;                                                   \
    }                                                               \
    if (pid_##IDX) {                                                \
      all_pid_nrs[IDXI] = pid_##IDX;                                \
      ret = dilate_proc(pid_##IDX);                                 \
    }                                                               \
    if (ret < 0) {                                                  \
      return ret;                                                   \
    }                                                               \
    return count;                                                   \
  }

#define DECLARE_PID_ATTR(IDX)                                       \
  static struct kobj_attribute pid_##IDX##_attr =                   \
      __ATTR(pid_##IDX, 0660, pid_##IDX##_show, pid_##IDX##_store)

#endif // __VTGPIO_H
