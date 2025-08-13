#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/termios.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>

// Define custom IOCTL commands
#define VSERIAL_IOCTL_MAGIC 'V'
#define VSERIAL_SET_PAIRS _IOW(VSERIAL_IOCTL_MAGIC, 0, int)
#define VSERIAL_GET_PAIRS                                                      \
  _IOR(VSERIAL_IOCTL_MAGIC, 1, struct vserial_pairs_info)

// Per-port data structure
struct vserial_port {
  char name[32];            // Pair name identifier
  struct tty_port portA;    // TTY port for end A
  struct tty_port portB;    // TTY port for end B
  struct ktermios termiosA; // Terminal settings for end A
  struct ktermios termiosB; // Terminal settings for end B
  spinlock_t lock;          // Protects access to the port
  int indexA;               // Minor number for port A
  int indexB;               // Minor number for port B
  int open_count;           // Track open count for diagnostics
};

// For VSERIAL_GET_PAIRS IOCTL
struct vserial_pair_info {
  int minorA;
  int minorB;
};

struct vserial_pairs_info {
  int num_pairs;
  struct vserial_pair_info __user *pairs;
};

#define VSERIAL_TERMIOS_MASK (CSIZE | CSTOPB | PARENB | PARODD | CBAUD)

// Global data
static struct tty_driver *tty_driver; // Our TTY driver

// Pair management
static DEFINE_MUTEX(port_mutex);
static struct vserial_port **port_array = NULL;
static int current_num_pairs = 0;

static inline bool vserial_termios_match(const struct ktermios *tx,
                                         const struct ktermios *rx) {
  /* Baud rate check */
  if (tx->c_ospeed != rx->c_ispeed) {
    printk(KERN_DEBUG
           "vserial: BAUD MISMATCH: tx->c_ospeed=%d, rx->c_ispeed=%d\n",
           tx->c_ospeed, rx->c_ispeed);
    return false;
  }

  /* Data format check (mask only relevant bits) */
  if ((tx->c_cflag & VSERIAL_TERMIOS_MASK) !=
      (rx->c_cflag & VSERIAL_TERMIOS_MASK)) {
    printk(KERN_DEBUG "vserial: FORMAT MISMATCH: tx_cflag=0x%x, rx_cflag=0x%x "
                      "(masked: 0x%x)\n",
           tx->c_cflag, rx->c_cflag, VSERIAL_TERMIOS_MASK);
    return false;
  }

  return true;
}

// TTY port operations
static void vserial_port_destruct(struct tty_port *port) {
  printk(KERN_INFO "vserial: port destruct called\n");
}

static const struct tty_port_operations vserial_port_ops = {
    .destruct = vserial_port_destruct,
};

// TTY operations implementation
static int vserial_open(struct tty_struct *tty, struct file *filp) {
  struct vserial_port *port = NULL;
  int minor = tty->index;
  int pair_index = minor / 2;
  int within_pair = minor % 2;
  int ret;

  printk(KERN_INFO "vserial: OPEN request for index=%d (name=%s)\n", minor,
         tty->driver->name);

  mutex_lock(&port_mutex);
  if (pair_index >= current_num_pairs) {
    mutex_unlock(&port_mutex);
    printk(KERN_ERR "vserial: OPEN failed - invalid index=%d\n", minor);
    return -ENODEV;
  }
  port = port_array[pair_index];
  mutex_unlock(&port_mutex);

  if (within_pair == 0) {
    tty->port = &port->portA;
  } else {
    tty->port = &port->portB;
  }
  tty->driver_data = port;

  printk(KERN_INFO "vserial: Found port '%s' for index=%d (A=%d, B=%d)\n",
         port->name, minor, port->indexA, port->indexB);
  // FIXME: potential race condition
  ret = tty_port_open(tty->port, tty, filp);
  if (ret == 0) {
    spin_lock(&port->lock);
    if (within_pair == 0) {
      port->termiosA = tty->termios;
      port->open_count++;
      printk(KERN_INFO "vserial: Port A (%d) OPENED for '%s', count=%d\n",
             port->indexA, port->name, port->open_count);
    } else {
      port->termiosB = tty->termios;
      port->open_count++;
      printk(KERN_INFO "vserial: Port B (%d) OPENED for '%s', count=%d\n",
             port->indexB, port->name, port->open_count);
    }
    spin_unlock(&port->lock);

    // Log the termios settings
    printk(KERN_DEBUG
           "vserial: termios: c_ispeed=%d, c_ospeed=%d, c_cflag=0x%x\n",
           tty->termios.c_ispeed, tty->termios.c_ospeed, tty->termios.c_cflag);
  } else {
    printk(KERN_ERR "vserial: OPEN failed for index=%d with error %d\n", minor,
           ret);
  }

  return ret;
}

static void vserial_close(struct tty_struct *tty, struct file *filp) {
  struct vserial_port *port = tty->driver_data;
  int minor = tty->index;
  int within_pair = minor % 2;

  printk(KERN_INFO "vserial: CLOSE request for index=%d\n", minor);

  if (port) {
    spin_lock(&port->lock);
    if (within_pair == 0) {
      port->open_count--;
      printk(KERN_INFO "vserial: Port A (%d) CLOSED for '%s', count=%d\n",
             port->indexA, port->name, port->open_count);
    } else {
      port->open_count--;
      printk(KERN_INFO "vserial: Port B (%d) CLOSED for '%s', count=%d\n",
             port->indexB, port->name, port->open_count);
    }
    spin_unlock(&port->lock);

    tty_port_close(tty->port, tty, filp);
  } else {
    printk(KERN_WARNING "vserial: CLOSE called with NULL port for index=%d\n",
           minor);
  }
}

static ssize_t vserial_write(struct tty_struct *tty, const u8 *buf,
                             size_t count) {
  struct vserial_port *vserial_port_ = tty->driver_data;
  struct tty_port *dest_port;
  unsigned long flags;
  int ret = 0;
  int index = tty->index;
  int within_pair = index % 2;
  const char *direction;
  char log_buf[64];

  printk(KERN_DEBUG "vserial: WRITE request for index=%d, count=%zu\n", index,
         count);

  if (!vserial_port_) {
    printk(KERN_ERR "vserial: WRITE failed - no port data for index=%d\n",
           index);
    return -ENODEV;
  }

  spin_lock_irqsave(&vserial_port_->lock, flags);
  const struct ktermios *tx_termios, *rx_termios;
  // Determine direction and get termios pointers
  if (within_pair == 0) {
    tx_termios = &vserial_port_->termiosA;
    rx_termios = &vserial_port_->termiosB;
    dest_port = &vserial_port_->portB;
    direction = "A→B";
  } else {
    tx_termios = &vserial_port_->termiosB;
    rx_termios = &vserial_port_->termiosA;
    dest_port = &vserial_port_->portA;
    direction = "B→A";
  }

  // Check configuration compatibility
  if (!vserial_termios_match(tx_termios, rx_termios)) {
    printk(KERN_WARNING
           "vserial: termios mismatch for %s, dropping %zu bytes\n",
           direction, count);
    ret = count; /* Silently drop data if incompatible */
    goto unlock;
  }

  if (!tty->port || !dest_port) {
    printk(KERN_ERR "vserial: NULL port pointer for %s\n", direction);
    ret = -ENODEV;
    goto unlock;
  }

  // Log the data (first 32 bytes)
  memset(log_buf, 0, sizeof(log_buf));
  size_t log_len = min(count, (size_t)32);
  memcpy(log_buf, buf, log_len);
  log_buf[log_len] = '\0';
  printk(KERN_DEBUG "vserial: Writing %zu bytes %s: '%*pE'\n", count, direction,
         (int)log_len, log_buf);
  if (!tty_port_initialized(dest_port)) {
    ret = count;
    printk(KERN_WARNING "vserial: Data dropped (destination is closed)\n");
    goto unlock;
  }
  // Forward data if settings match and port is active
  if (tty_port_initialized(dest_port)) {
    size_t pushed = tty_insert_flip_string(dest_port, buf, count);
    tty_flip_buffer_push(dest_port);
    printk(KERN_DEBUG "vserial: Pushed %zu/%zu bytes %s\n", pushed, count,
           direction);
    ret = pushed;
  } else {
    printk(KERN_WARNING "vserial: Destination port not initialized for %s\n",
           direction);
    ret = -ENODEV;
  }

unlock:
  spin_unlock_irqrestore(&vserial_port_->lock, flags);
  return ret;
}

static void vserial_set_termios(struct tty_struct *tty,
                                const struct ktermios *old) {
  struct vserial_port *port = tty->driver_data;
  int index = tty->index;
  int within_pair = index % 2;
  const char *end = (within_pair == 0) ? "A" : "B";

  if (!port) {
    printk(KERN_WARNING
           "vserial: set_termios called with NULL port for index=%d\n",
           index);
    return;
  }

  printk(KERN_INFO "vserial: SET_TERMIOS for %s (index=%d), ispeed=%d, "
                   "ospeed=%d, cflag=0x%x\n",
         end, index, tty->termios.c_ispeed, tty->termios.c_ospeed,
         tty->termios.c_cflag);

  spin_lock(&port->lock);
  if (within_pair == 0)
    port->termiosA = tty->termios;
  else
    port->termiosB = tty->termios;
  spin_unlock(&port->lock);
}

static int vserial_ioctl(struct tty_struct *tty, unsigned int cmd,
                         unsigned long arg) {
  struct vserial_port *port = tty->driver_data;
  struct serial_struct ss;
  int index = tty->index;
  int within_pair = index % 2;
  const char *end = (within_pair == 0) ? "A" : "B";

  if (!port) {
    printk(KERN_WARNING "vserial: ioctl called with NULL port for index=%d\n",
           index);
    return -ENODEV;
  }

  printk(KERN_DEBUG "vserial: IOCTL 0x%08x for %s (index=%d)\n", cmd, end,
         index);

  switch (cmd) {
  case TIOCGSERIAL:
    // return fake data as its not supported
    memset(&ss, 0, sizeof(ss));
    ss.baud_base = 115200;
    if (copy_to_user((void __user *)arg, &ss, sizeof(ss))) {
      printk(KERN_ERR "vserial: TIOCGSERIAL failed (EFAULT)\n");
      return -EFAULT;
    }
    printk(KERN_DEBUG "vserial: TIOCGSERIAL returned baud_base=%d\n",
           ss.baud_base);
    return 0;

  case TIOCSSERIAL:
    if (copy_from_user(&ss, (void __user *)arg, sizeof(ss))) {
      printk(KERN_ERR "vserial: TIOCSSERIAL failed (EFAULT)\n");
      return -EFAULT;
    }
    printk(KERN_DEBUG "vserial: TIOCSSERIAL received baud_base=%d\n",
           ss.baud_base);
    return 0;
  // TODO: modem control
  case TIOCMGET:
    return put_user(0, (int __user *)arg);

  case TIOCMSET:
    return 0;
    // handled by tty set_termios
    // case TCSETS:
    // case TCSETSW:
    // case TCSETSF:
    //   return 0;
  }
  printk(KERN_DEBUG "vserial: IOCTL 0x%08x not handled for %s\n", cmd, end);
  return -ENOIOCTLCMD;
}
static unsigned int vserial_write_room(struct tty_struct *tty) { return 4096; }

static unsigned int vserial_chars_in_buffer(struct tty_struct *tty) {
  return 0;
}
static const struct tty_operations vserial_ops = {
    .open = vserial_open,
    .close = vserial_close,
    .write = vserial_write,
    .ioctl = vserial_ioctl,
    .set_termios = vserial_set_termios,
    .write_room = vserial_write_room,
    .chars_in_buffer = vserial_chars_in_buffer,
};

// Control device handling
static long vserial_ctl_ioctl(struct file *file, unsigned int cmd,
                              unsigned long arg) {
  struct vserial_pairs_info info;
  struct vserial_pair_info *kern_pairs = NULL;
  int ret = 0;
  int i;

  mutex_lock(&port_mutex);

  switch (cmd) {
  case VSERIAL_SET_PAIRS: {
    int new_num;
    struct vserial_port **new_array = NULL;
    int j;

    if (copy_from_user(&new_num, (void __user *)arg, sizeof(new_num))) {
      ret = -EFAULT;
      goto out;
    }

    if (new_num < 0 || new_num > 64) {
      ret = -EINVAL;
      goto out;
    }

    printk(KERN_INFO "vserial: SET_PAIRS: new_num=%d, current=%d\n", new_num,
           current_num_pairs);

    // Remove existing pairs if any
    if (port_array) {
      // Check for open ports first
      for (i = 0; i < current_num_pairs; i++) {
        struct vserial_port *port = port_array[i];
        spin_lock(&port->lock);
        if (port->open_count > 0) {
          spin_unlock(&port->lock);
          printk(KERN_WARNING
                 "vserial: SET_PAIRS: port '%s' (A=%d,B=%d) still open\n",
                 port->name, port->indexA, port->indexB);
          ret = -EBUSY;
          goto out;
        }
        spin_unlock(&port->lock);
      }

      // Now remove the ports
      for (i = 0; i < current_num_pairs; i++) {
        struct vserial_port *port = port_array[i];
        printk(KERN_INFO
               "vserial: SET_PAIRS: removing port '%s' (A=%d, B=%d)\n",
               port->name, port->indexA, port->indexB);
        tty_port_unregister_device(&port->portA, tty_driver, port->indexA);
        tty_port_unregister_device(&port->portB, tty_driver, port->indexB);
        tty_port_destroy(&port->portA);
        tty_port_destroy(&port->portB);
        kfree(port);
      }
      kfree(port_array);
      port_array = NULL;
      current_num_pairs = 0;
    }

    if (new_num == 0) {
      break;
    }

    new_array = kcalloc(new_num, sizeof(struct vserial_port *), GFP_KERNEL);
    if (!new_array) {
      ret = -ENOMEM;
      goto out;
    }

    for (i = 0; i < new_num; i++) {
      struct vserial_port *port = kzalloc(sizeof(*port), GFP_KERNEL);
      if (!port) {
        ret = -ENOMEM;
        goto free_ports;
      }

      snprintf(port->name, sizeof(port->name), "pair%d", i);
      spin_lock_init(&port->lock);
      port->open_count = 0;
      port->indexA = i * 2;
      port->indexB = i * 2 + 1;

      printk(KERN_INFO
             "vserial: SET_PAIRS: allocating indices A=%d, B=%d for '%s'\n",
             port->indexA, port->indexB, port->name);

      tty_port_init(&port->portA);
      port->portA.ops = &vserial_port_ops;
      printk(KERN_INFO "vserial: Initializing ports for pair %d", i);
      tty_port_init(&port->portB);
      port->portB.ops = &vserial_port_ops;

      // Register devices with proper error checking
      printk(KERN_INFO "vserial: Registering devices for pair %d", i);
      struct device *devA = tty_port_register_device(&port->portA, tty_driver,
                                                     port->indexA, NULL);
      struct device *devB = tty_port_register_device(&port->portB, tty_driver,
                                                     port->indexB, NULL);
      if (IS_ERR(devA) || IS_ERR(devB)) {
        printk(KERN_ERR "vserial: Failed to register devices for pair %d: %d\n",
               i, ret);
        // Unregister any successfully registered device
        if (!IS_ERR(devA))
          tty_port_unregister_device(&port->portA, tty_driver, port->indexA);
        if (!IS_ERR(devB))
          tty_port_unregister_device(&port->portB, tty_driver, port->indexB);

        tty_port_destroy(&port->portA);
        tty_port_destroy(&port->portB);
        kfree(port);

        ret = PTR_ERR(devA) ?: PTR_ERR(devB);
        goto free_ports;
      }
      dev_set_uevent_suppress(devA, false);
      dev_set_uevent_suppress(devB, false);
      printk(KERN_INFO "vserial: No error for pair %d", i);

      port->termiosA = tty_driver->init_termios;
      port->termiosB = tty_driver->init_termios;

      new_array[i] = port;
    }

    port_array = new_array;
    current_num_pairs = new_num;
    break;

  free_ports:
    if (new_array) {
      for (j = 0; j < i; j++) {
        if (new_array[j]) {
          tty_port_unregister_device(&new_array[j]->portA, tty_driver,
                                     new_array[j]->indexA);
          tty_port_unregister_device(&new_array[j]->portB, tty_driver,
                                     new_array[j]->indexB);
          tty_port_destroy(&new_array[j]->portA);
          tty_port_destroy(&new_array[j]->portB);
          kfree(new_array[j]);
        }
      }
      kfree(new_array);
    }
    break;
  }

  case VSERIAL_GET_PAIRS: {
    if (copy_from_user(&info, (void __user *)arg, sizeof(info))) {
      ret = -EFAULT;
      goto out;
    }

    // Write back current pair count
    if (put_user(current_num_pairs,
                 &((struct vserial_pairs_info __user *)arg)->num_pairs)) {
      ret = -EFAULT;
      goto out;
    }

    if (info.pairs && current_num_pairs > 0) {
      int num_to_copy = min_t(int, info.num_pairs, current_num_pairs);
      if (num_to_copy <= 0) {
        ret = -ENODATA;
        goto out;
      }
      kern_pairs = kmalloc_array(num_to_copy, sizeof(struct vserial_pair_info),
                                 GFP_KERNEL);
      if (!kern_pairs) {
        ret = -ENOMEM;
        goto out;
      }

      for (i = 0; i < num_to_copy; i++) {
        kern_pairs[i].minorA = port_array[i]->indexA;
        kern_pairs[i].minorB = port_array[i]->indexB;
      }

      if (copy_to_user(info.pairs, kern_pairs,
                       num_to_copy * sizeof(struct vserial_pair_info))) {
        kfree(kern_pairs);
        ret = -EFAULT;
        goto out;
      }
      kfree(kern_pairs);
    }
    break;
  }
  default:
    printk(KERN_DEBUG "vserial: CTL IOCTL 0x%08x not handled\n", cmd);
    ret = -ENOTTY;
    break;
  }

out:
  mutex_unlock(&port_mutex);
  return ret;
}
static const struct file_operations vserial_ctl_fops = {
    .unlocked_ioctl = vserial_ctl_ioctl,
};

static struct miscdevice ctl_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "vserialctl",
    .fops = &vserial_ctl_fops,
};
// Module initialization
static int __init vserial_init(void) {
  int ret;
  printk(KERN_INFO "vserial: hello\n");
  printk(KERN_INFO "vserial: [INIT] Starting module initialization\n");
  // Allocate driver with MAX_PORTS (64 pairs = 128 ports)
  tty_driver =
      tty_alloc_driver(128, TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
  if (IS_ERR(tty_driver)) {
    ret = PTR_ERR(tty_driver);
    printk(KERN_ERR "vserial: [INIT] alloc driver failed (%d)\n", ret);
    return ret;
  }

  tty_driver->driver_name = "vserial";
  tty_driver->name = "ttySV";
  tty_driver->major = 0;
  tty_driver->minor_start = 0;
  tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
  tty_driver->subtype = SERIAL_TYPE_NORMAL;
  tty_driver->init_termios = tty_std_termios;
  tty_driver->num = 128; // Must match allocation size
  tty_set_operations(tty_driver, &vserial_ops);

  ret = tty_register_driver(tty_driver);
  if (ret) {
    printk(KERN_ERR "vserial: [INIT] register driver failed (%d)\n", ret);
    tty_driver_kref_put(tty_driver);
    return ret;
  }
  printk(KERN_INFO "vserial: [INIT] registered with major %d, name='%s'\n",
         tty_driver->major, tty_driver->name);

  ret = misc_register(&ctl_device);
  if (ret) {
    printk(KERN_ERR "vserial: [INIT] misc register failed (%d)\n", ret);
    tty_unregister_driver(tty_driver);
    tty_driver_kref_put(tty_driver);
    return ret;
  }
  printk(KERN_INFO "vserial: [INIT] control device created at /dev/%s\n",
         ctl_device.name);

  // Initialize global state
  mutex_init(&port_mutex);
  port_array = NULL;
  current_num_pairs = 0;

  return 0;
}

static void __exit vserial_exit(void) {
  int i;

  printk(KERN_INFO "vserial: [CLEANUP] Starting module cleanup\n");

  misc_deregister(&ctl_device);
  printk(KERN_INFO "vserial: [CLEANUP] Control device unregistered\n");

  mutex_lock(&port_mutex);
  if (port_array) {
    for (i = 0; i < current_num_pairs; i++) {
      struct vserial_port *port = port_array[i];
      printk(KERN_INFO "vserial: [CLEANUP] Removing port '%s' (A=%d, B=%d)\n",
             port->name, port->indexA, port->indexB);
      tty_port_unregister_device(&port->portA, tty_driver, port->indexA);
      tty_port_unregister_device(&port->portB, tty_driver, port->indexB);
      tty_port_destroy(&port->portA);
      tty_port_destroy(&port->portB);
      kfree(port);
    }
    kfree(port_array);
    port_array = NULL;
    current_num_pairs = 0;
  }
  mutex_unlock(&port_mutex);

  if (tty_driver) {
    printk(KERN_INFO "vserial: [CLEANUP] Unregistering TTY driver (major=%d)\n",
           tty_driver->major);
    tty_unregister_driver(tty_driver);
    tty_driver_kref_put(tty_driver);
    tty_driver = NULL;
  }

  printk(KERN_INFO "vserial: [CLEANUP] Module cleanup completed\n");
}

module_init(vserial_init);
module_exit(vserial_exit);
MODULE_DESCRIPTION("A virtual serial port driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("zz z");
