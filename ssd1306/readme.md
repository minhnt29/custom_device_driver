# SSD1306 Driver - Giải Thích Chi Tiết Từ A-Z

## 📚 Mục Lục
1. [Kiến trúc tổng quan](#overview)
2. [Cách kernel load driver](#loading)
3. [Tạo character device /dev/ssd1306](#chardev)
4. [I2C subsystem](#i2c)
5. [File operations](#fops)
6. [Từng function chi tiết](#functions)

---

## 1. Kiến Trúc Tổng Quan {#overview}

### Luồng hoạt động khi boot
```
[Boot] → [Kernel khởi động] → [Device Tree parsing] 
    ↓
[Phát hiện I2C device: ssd1306@0x3c]
    ↓
[Gọi ssd1306_probe()] ← Driver đã register
    ↓
[Khởi tạo hardware + Tạo /dev/ssd1306]
    ↓
[Sẵn sàng cho userspace sử dụng]
```

### Layers trong driver
```
┌─────────────────────────────────────┐
│  Userspace Application              │
│  (test_ssd1306, Python, Shell)      │
└──────────────┬──────────────────────┘
               │ open(), write(), read()
               ↓
┌─────────────────────────────────────┐
│  Character Device Interface         │
│  /dev/ssd1306                        │
└──────────────┬──────────────────────┘
               │ file_operations
               ↓
┌─────────────────────────────────────┐
│  SSD1306 Driver (ssd1306.c)         │
│  - Buffer management                │
│  - Command/Data handling            │
└──────────────┬──────────────────────┘
               │ i2c_master_send()
               ↓
┌─────────────────────────────────────┐
│  I2C Subsystem                      │
│  (Kernel I2C framework)             │
└──────────────┬──────────────────────┘
               │ Hardware access
               ↓
┌─────────────────────────────────────┐
│  I2C Controller Driver              │
│  (BCM2835 I2C, etc)                 │
└──────────────┬──────────────────────┘
               │ Physical I2C bus
               ↓
         [SSD1306 Hardware]
```

---

## 2. Cách Kernel Load Driver {#loading}

### 2.1. Device Tree Role

**Device Tree (.dts):**
```dts
&i2c1 {
    status = "okay";
    
    ssd1306: oled@3c {
        compatible = "solomon,ssd1306";  // ← Key quan trọng!
        reg = <0x3c>;                     // ← I2C address
        status = "okay";
    };
};
```

**Trong driver có matching table:**
```c
static const struct of_device_id ssd1306_of_match[] = {
    { .compatible = "solomon,ssd1306" },  // ← Phải match với DTS
    { }
};
MODULE_DEVICE_TABLE(of, ssd1306_of_match);
```

### 2.2. Quá trình Matching

```
Kernel boot
    ↓
Parse device tree → Tìm thấy node "ssd1306@3c"
    ↓
Đọc property "compatible" = "solomon,ssd1306"
    ↓
Tìm driver có of_device_id match
    ↓
Tìm thấy ssd1306_driver với matching "solomon,ssd1306"
    ↓
Gọi ssd1306_probe()
```

### 2.3. Driver Registration

**Macro quan trọng:**
```c
module_i2c_driver(ssd1306_driver);

// Macro này expand thành:
static int __init ssd1306_init(void)
{
    return i2c_add_driver(&ssd1306_driver);
}
module_init(ssd1306_init);

static void __exit ssd1306_exit(void)
{
    i2c_del_driver(&ssd1306_driver);
}
module_exit(ssd1306_exit);
```

**Giải thích:**
- `module_init()`: Hàm được gọi khi module load (insmod hoặc boot time)
- `module_exit()`: Hàm được gọi khi module unload (rmmod)
- `i2c_add_driver()`: Register driver với I2C subsystem
- `i2c_del_driver()`: Unregister driver

---

## 3. Tạo Character Device /dev/ssd1306 {#chardev}

### 3.1. Character Device là gì?

**Character Device:**
- Là file đặc biệt trong /dev/
- Cho phép userspace tương tác với driver qua file operations
- Mỗi char device có **major** và **minor** number

**Ví dụ:**
```bash
$ ls -l /dev/ssd1306
crw-rw---- 1 root root 240, 0 Nov 10 10:00 /dev/ssd1306
 ↑                    ↑    ↑
 |                    |    └─ Minor number
 |                    └─ Major number  
 └─ c = character device
```

### 3.2. Các bước tạo Character Device

**Trong ssd1306_probe():**

```c
/* Bước 1: Xin cấp phát device number */
ret = alloc_chrdev_region(&ssd1306_device->dev_num, 0, 1, DRIVER_NAME);
// Input:
//   - &dev_num: Con trỏ để nhận device number
//   - 0: Minor number đầu tiên (base minor)
//   - 1: Số lượng device (chỉ cần 1)
//   - DRIVER_NAME: Tên xuất hiện trong /proc/devices
// Output:
//   - dev_num chứa major:minor (ví dụ: 240:0)

/* Bước 2: Khởi tạo cdev structure */
cdev_init(&ssd1306_device->cdev, &ssd1306_fops);
// Liên kết cdev với file_operations
// ssd1306_fops chứa các hàm: open, read, write, release

/* Bước 3: Thêm cdev vào kernel */
ret = cdev_add(&ssd1306_device->cdev, ssd1306_device->dev_num, 1);
// Kernel giờ biết device này tồn tại
// Nhưng chưa có file /dev/ssd1306!

/* Bước 4: Tạo device class */
ssd1306_device->class = class_create(THIS_MODULE, DRIVER_CLASS);
// Tạo class trong /sys/class/ssd1306Class/
// Dùng cho udev tự động tạo device node

/* Bước 5: Tạo device node */
ssd1306_device->device = device_create(
    ssd1306_device->class,           // Class vừa tạo
    NULL,                             // Parent device (NULL = no parent)
    ssd1306_device->dev_num,          // Major:Minor number
    NULL,                             // Driver data
    DRIVER_NAME                       // Tên device = "ssd1306"
);
// → Udev tự động tạo /dev/ssd1306!
```

### 3.3. Udev và Device Node Creation

**Udev workflow:**
```
kernel: device_create() được gọi
    ↓
kernel: Gửi uevent đến userspace
    ↓
udev daemon: Nhận uevent
    ↓
udev: Đọc rules từ /etc/udev/rules.d/
    ↓
udev: Tạo /dev/ssd1306 với permissions từ rules
    ↓
Done! File /dev/ssd1306 sẵn sàng
```

**Xem thông tin:**
```bash
# Major:Minor number
cat /proc/devices | grep ssd1306

# Sysfs info
ls -la /sys/class/ssd1306Class/ssd1306/

# Udev info
udevadm info /dev/ssd1306
```

---

## 4. I2C Subsystem {#i2c}

### 4.1. I2C Client Structure

```c
struct i2c_client {
    unsigned short addr;        // I2C address (0x3c)
    struct i2c_adapter *adapter; // I2C bus controller
    struct device dev;          // Device structure
    int irq;                    // IRQ number (if any)
    // ... more fields
};
```

**Trong driver:**
```c
static int ssd1306_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    // client được kernel tự động tạo và truyền vào
    // client->addr = 0x3c (từ device tree: reg = <0x3c>)
    // client->adapter = I2C bus controller (i2c-1, i2c-2, etc)
    
    ssd1306_device->client = client; // Lưu lại để dùng sau
}
```

### 4.2. I2C Communication

**i2c_master_send() hoạt động như thế nào:**

```c
int i2c_master_send(const struct i2c_client *client,
                    const char *buf, int count)
```

**Bên trong kernel (simplified):**
```c
i2c_master_send(client, buf, count)
    ↓
i2c_transfer(client->adapter, msg)  // msg chứa addr + data
    ↓
adapter->algo->master_xfer()  // Gọi hardware-specific function
    ↓
[BCM2835 I2C driver (cho Raspberry Pi)]
    ↓
bcm2835_i2c_xfer()  // Viết vào I2C hardware registers
    ↓
    while (not done) {
        write_register(BSC_FIFO, data);  // Write to FIFO
        wait_for_interrupt();
    }
    ↓
[Hardware gửi data qua SDA/SCL pins]
    ↓
[SSD1306 nhận data]
```

**Ví dụ cụ thể:**
```c
// Trong driver
u8 buf[2] = {0x00, 0xAF};  // Control byte + Display ON command
i2c_master_send(client, buf, 2);

// Kernel thực hiện:
// 1. Lock I2C bus
// 2. Generate START condition
// 3. Send slave address: 0x3C + Write bit
// 4. Wait for ACK
// 5. Send 0x00
// 6. Wait for ACK
// 7. Send 0xAF
// 8. Wait for ACK
// 9. Generate STOP condition
// 10. Unlock I2C bus
```

---

## 5. File Operations {#fops}

### 5.1. File Operations Structure

```c
static const struct file_operations ssd1306_fops = {
    .owner   = THIS_MODULE,
    .open    = ssd1306_open,
    .release = ssd1306_release,
    .read    = ssd1306_read,
    .write   = ssd1306_write,
};
```

### 5.2. Luồng hoạt động khi userspace gọi

**Userspace code:**
```c
int fd = open("/dev/ssd1306", O_RDWR);
// → Kernel gọi ssd1306_open()

write(fd, buffer, 1024);
// → Kernel gọi ssd1306_write()

read(fd, buffer, 1024);
// → Kernel gọi ssd1306_read()

close(fd);
// → Kernel gọi ssd1306_release()
```

### 5.3. Chi tiết từng operation

#### open()
```c
static int ssd1306_open(struct inode *inode, struct file *file)
{
    file->private_data = ssd1306_device;
    return 0;
}
```

**Giải thích:**
- `inode`: Chứa thông tin về file trong filesystem (major/minor, permissions)
- `file`: Chứa thông tin về file đã mở (position, flags, private_data)
- `private_data`: Pointer tùy ý để lưu driver-specific data
- Return 0 = success, negative = error code

**Tại sao cần private_data?**
```c
// Trong open(): Lưu device structure
file->private_data = ssd1306_device;

// Trong write(): Lấy lại device structure
struct ssd1306_dev *dev = file->private_data;
// Giờ có thể dùng dev->client, dev->buffer, etc.
```

#### write()
```c
static ssize_t ssd1306_write(struct file *file, const char __user *buf, 
                              size_t count, loff_t *ppos)
{
    struct ssd1306_dev *dev = file->private_data;
    size_t to_write;
    
    // Kiểm tra offset
    if (*ppos >= SSD1306_BUFFER_SIZE)
        return 0;  // EOF
    
    // Tính số bytes cần write
    to_write = min(count, (size_t)(SSD1306_BUFFER_SIZE - *ppos));
    
    mutex_lock(&dev->lock);
    
    // Copy data từ userspace vào kernel buffer
    if (copy_from_user(dev->buffer + *ppos, buf, to_write)) {
        mutex_unlock(&dev->lock);
        return -EFAULT;  // Bad address
    }
    
    // Update display hardware
    ssd1306_write_cmd(dev->client, SSD1306_COLUMN_ADDR);
    ssd1306_write_cmd(dev->client, 0);
    ssd1306_write_cmd(dev->client, SSD1306_WIDTH - 1);
    
    ssd1306_write_data(dev->client, dev->buffer, SSD1306_BUFFER_SIZE);
    
    mutex_unlock(&dev->lock);
    
    *ppos += to_write;  // Update file position
    return to_write;     // Return bytes written
}
```

**Chi tiết quan trọng:**

1. **`__user` pointer:**
```c
const char __user *buf
```
- `__user` = annotation cho sparse checker
- Báo rằng `buf` trỏ đến userspace memory
- KHÔNG được dereference trực tiếp!
- Phải dùng `copy_from_user()` hoặc `copy_to_user()`

2. **`copy_from_user()`:**
```c
unsigned long copy_from_user(void *to, const void __user *from, unsigned long n)
```
- Copy dữ liệu từ userspace → kernelspace
- Kiểm tra address hợp lệ
- Handle page faults
- Return: số bytes KHÔNG copy được (0 = success)

3. **`mutex_lock()` tại sao?**
```c
mutex_lock(&dev->lock);
// Critical section: modify dev->buffer
mutex_unlock(&dev->lock);
```
- Protect shared resource (buffer)
- Tránh race condition khi nhiều process cùng write
- Ví dụ race condition:
```
Process A: Read buffer[0]     Process B: Read buffer[0]
Process A: Modify = 5                  |
Process A: Write buffer[0] = 5         |
         |                   Process B: Modify = 10
         |                   Process B: Write buffer[0] = 10
Result: Data của A bị mất!
```

4. **`loff_t *ppos` (file position):**
```c
// First write
write(fd, "ABC", 3);  // ppos: 0 → 3

// Second write
write(fd, "DEF", 3);  // ppos: 3 → 6

// Result in buffer: "ABCDEF..."
```

#### read()
```c
static ssize_t ssd1306_read(struct file *file, char __user *buf,
                             size_t count, loff_t *ppos)
{
    struct ssd1306_dev *dev = file->private_data;
    size_t to_read;
    
    if (*ppos >= SSD1306_BUFFER_SIZE)
        return 0;  // EOF
    
    to_read = min(count, (size_t)(SSD1306_BUFFER_SIZE - *ppos));
    
    mutex_lock(&dev->lock);
    
    // Copy data từ kernel buffer → userspace
    if (copy_to_user(buf, dev->buffer + *ppos, to_read)) {
        mutex_unlock(&dev->lock);
        return -EFAULT;
    }
    
    mutex_unlock(&dev->lock);
    
    *ppos += to_read;
    return to_read;
}
```

**copy_to_user():**
```c
unsigned long copy_to_user(void __user *to, const void *from, unsigned long n)
```
- Copy từ kernelspace → userspace
- Ngược lại với copy_from_user()

---

## 6. Chi Tiết Từng Function {#functions}

### 6.1. ssd1306_probe() - Khởi tạo driver

```c
static int ssd1306_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int ret;
    
    dev_info(&client->dev, "SSD1306 probe started\n");
```

**`dev_info()` là gì?**
```c
dev_info(&client->dev, "message %d", value);

// Tương đương với:
printk(KERN_INFO "ssd1306: message %d", value);

// Xuất hiện trong:
dmesg | grep ssd1306
```

**Các log level khác:**
```c
dev_err()   // Error messages
dev_warn()  // Warnings
dev_info()  // Information
dev_dbg()   // Debug (chỉ hiện khi enable DEBUG)
```

**Allocate memory:**
```c
    ssd1306_device = kzalloc(sizeof(struct ssd1306_dev), GFP_KERNEL);
    if (!ssd1306_device)
        return -ENOMEM;
```

**`kzalloc()` vs `malloc()`:**

| Userspace         | Kernelspace          |
|-------------------|----------------------|
| malloc()          | kmalloc()            |
| calloc()          | kzalloc()            |
| free()            | kfree()              |
| Can sleep         | Depends on flags     |

**GFP flags:**
```c
GFP_KERNEL  // Có thể sleep, dùng trong process context
GFP_ATOMIC  // Không sleep, dùng trong interrupt context
GFP_DMA     // Memory cho DMA
```

**Allocate buffer:**
```c
    ssd1306_device->buffer = kzalloc(SSD1306_BUFFER_SIZE, GFP_KERNEL);
    if (!ssd1306_device->buffer) {
        kfree(ssd1306_device);
        return -ENOMEM;
    }
```

**Tại sao cần buffer trong kernel?**
```
Userspace write() → copy_from_user() → Kernel buffer → I2C transfer → Hardware

Buffer này:
- Store frame buffer (1024 bytes = 128x64 pixels / 8)
- Avoid multiple userspace copies
- Allow partial updates
```

**Initialize mutex:**
```c
    mutex_init(&ssd1306_device->lock);
```

**Mutex structure:**
```c
struct mutex {
    atomic_long_t owner;
    spinlock_t wait_lock;
    struct list_head wait_list;
};

// Usage:
mutex_lock(&lock);    // Acquire, sleep if locked
// ... critical section ...
mutex_unlock(&lock);  // Release
```

**Tạo character device (đã giải thích ở section 3):**
```c
    ret = alloc_chrdev_region(&ssd1306_device->dev_num, 0, 1, DRIVER_NAME);
    cdev_init(&ssd1306_device->cdev, &ssd1306_fops);
    ret = cdev_add(&ssd1306_device->cdev, ssd1306_device->dev_num, 1);
    ssd1306_device->class = class_create(THIS_MODULE, DRIVER_CLASS);
    ssd1306_device->device = device_create(...);
```

**Save client pointer:**
```c
    i2c_set_clientdata(client, ssd1306_device);
```

**`i2c_set_clientdata()` làm gì?**
```c
// Set:
i2c_set_clientdata(client, ssd1306_device);

// Get (trong các hàm khác):
struct ssd1306_dev *dev = i2c_get_clientdata(client);

// Implementation:
static inline void i2c_set_clientdata(struct i2c_client *client, void *data)
{
    dev_set_drvdata(&client->dev, data);  // Store in device structure
}
```

**Error handling với goto:**
```c
    return 0;

err_device:
    class_destroy(ssd1306_device->class);
err_class:
    cdev_del(&ssd1306_device->cdev);
err_cdev:
    unregister_chrdev_region(ssd1306_device->dev_num, 1);
err_init:
    kfree(ssd1306_device->buffer);
    kfree(ssd1306_device);
    return ret;
}
```

**Tại sao dùng goto?**
- Clean error handling
- Tránh code duplication
- Đảm bảo cleanup đúng thứ tự (reverse của init)

### 6.2. ssd1306_remove() - Cleanup

```c
static int ssd1306_remove(struct i2c_client *client)
{
    struct ssd1306_dev *dev = i2c_get_clientdata(client);
    
    // Cleanup theo thứ tự ngược lại với probe
    device_destroy(dev->class, dev->dev_num);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->dev_num, 1);
    
    kfree(dev->buffer);
    kfree(dev);
    
    return 0;
}
```

### 6.3. ssd1306_init_display() - Initialize Hardware

```c
static int ssd1306_init_display(struct i2c_client *client)
{
    int ret;
    
    // Turn off display
    ret = ssd1306_write_cmd(client, SSD1306_DISPLAY_OFF);
    if (ret) return ret;
```

**Tại sao tắt display trước?**
- Safe initialization
- Avoid flickering
- Some commands chỉ work khi display OFF

**Set display clock:**
```c
    ret = ssd1306_write_cmd(client, SSD1306_SET_DISPLAY_CLOCK);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x80);
    if (ret) return ret;
```

**Command format:**
```
Command byte: 0xD5 (Set Display Clock Divide)
Data byte:    0x80 (Default value)
    [7:4] = Oscillator Frequency
    [3:0] = Divide Ratio
    0x80 = 0b10000000
         = Freq: 8, Divide: 0 → Default frequency
```

**Set multiplex ratio:**
```c
    ret = ssd1306_write_cmd(client, SSD1306_SET_MULTIPLEX);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x3F);
    if (ret) return ret;
```

**Multiplex ratio = 0x3F (63):**
- Display có 64 rows (0-63)
- Multiplex 64 rows → Full display height

**Charge pump:**
```c
    ret = ssd1306_write_cmd(client, SSD1306_CHARGE_PUMP);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x14);
    if (ret) return ret;
```

**Charge pump là gì?**
- OLED cần voltage cao (~7-15V) để phát sáng
- Charge pump tạo high voltage từ VCC (3.3V/5V)
- 0x14 = Enable charge pump (required cho display hoạt động)

**Memory mode:**
```c
    ret = ssd1306_write_cmd(client, SSD1306_MEMORY_MODE);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x00);
    if (ret) return ret;
```

**Memory addressing modes:**
```
0x00 = Horizontal Addressing Mode
0x01 = Vertical Addressing Mode
0x02 = Page Addressing Mode

Horizontal (0x00):
Write auto-increment từ trái → phải, top → bottom
Dễ dùng nhất cho frame buffer
```

**Segment remap & COM scan:**
```c
    ret = ssd1306_write_cmd(client, SSD1306_SEG_REMAP);      // 0xA1
    ret = ssd1306_write_cmd(client, SSD1306_COM_SCAN_DEC);   // 0xC8
```

**Remap là gì?**
```
SEG_REMAP (0xA1):
- Map column address 0 → SEG127
- Mirror display horizontally

COM_SCAN_DEC (0xC8):
- Scan từ COM63 → COM0
- Mirror display vertically

Kết hợp: Display đúng chiều!
```

**Set contrast:**
```c
    ret = ssd1306_write_cmd(client, SSD1306_SET_CONTRAST);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0xCF);
    if (ret) return ret;
```

**Contrast range:**
- 0x00 = Darkest
- 0xFF = Brightest
- 0xCF = Bright enough, not burn OLED

**Turn on display:**
```c
    ret = ssd1306_write_cmd(client, SSD1306_DISPLAY_ON);
    if (ret) return ret;
    
    dev_info(&client->dev, "SSD1306 initialized successfully\n");
    return 0;
}
```

---

## 7. Kernel vs Userspace Programming

### Differences Table

| Aspect              | Userspace               | Kernelspace            |
|---------------------|-------------------------|------------------------|
| Memory allocation   | malloc()/free()         | kmalloc()/kfree()      |
| Sleep               | Always allowed          | Depends on context     |
| Printf              | printf()                | printk()/dev_info()    |
| Access hardware     | Cannot                  | Can                    |
| Page fault          | OS handles it           | Kernel panic!          |
| Floating point      | Allowed                 | Not recommended        |
| Userspace memory    | Direct access           | copy_from/to_user()    |
| Error codes         | -1 + errno              | Negative error codes   |

### Context trong Kernel

```c
// Process context (có thể sleep)
- Driver probe/remove
- File operations (open, read, write)
- Sysfs operations
→ Có thể dùng: mutex, sleep, schedule, GFP_KERNEL

// Interrupt context (KHÔNG được sleep)
- IRQ handlers
- Softirqs
- Tasklets
→ Chỉ dùng: spinlock, GFP_ATOMIC, atomic operations
```

---

## 8. Testing & Debugging

### 8.1. Load module
```bash
# Load
insmod ssd1306.ko

# Check loaded
lsmod | grep ssd1306

# Kernel messages
dmesg | tail -20

# Module info
modinfo ssd1306.ko
```

### 8.2. Debug techniques

**Add debug prints:**
```c
#define DEBUG 1

static int ssd1306_write(struct file *file, const char __user *buf, 
                         size_t count, loff_t *ppos)
{
    dev_dbg(&dev->client->dev, "write: count=%zu, ppos=%lld\n", 
            count, *ppos);
    
    print_hex_dump(KERN_DEBUG, "Data: ", DUMP_PREFIX_OFFSET,
                   16, 1, dev->buffer, 32, true);
    // ...
}
```

**Dynamic debug:**
```bash
# Enable debug messages
echo 'file ssd1306.c +p' > /sys/kernel/debug/dynamic_debug/control

# Disable
echo 'file ssd1306.c -p' > /sys/kernel/debug/dynamic_debug/control
```

**Kernel debugger:**
```bash
# KGDB over serial
CONFIG_KGDB=y
CONFIG_KGDB_SERIAL_CONSOLE=y

# Set breakpoint
(gdb) break ssd1306_probe
(gdb) continue
```

### 8.3. Common issues

**Device not created:**
```bash
# Check if probe was called
dmesg | grep "probe started"

# Check device tree
cat /sys/firmware/devicetree/base/soc/i2c*/ssd1306*/compatible

# Check I2C address
i2cdetect -y 1
```

**I2C communication fails:**
```bash
# Check I2C bus
ls /dev/i2c-*

# Test I2C manually
i2cset -y 1 0x3c 0x00 0xAF i
```

**Module won't load:**
```bash
# Check kernel version mismatch
modinfo ssd1306.ko | grep vermagic
uname -r

# Check dependencies
modprobe -v ssd1306
```

---

## 9. Summary Workflow

```
Boot
 ↓
Device Tree parsed → Found "solomon,ssd1306" at 0x3c
 ↓
Kernel match driver có of_device_id tương ứng
 ↓
ssd1306_probe() được gọi
 ↓
  1. kmalloc() device structure
  2. kmalloc() frame buffer (1024 bytes)
  3. mutex_init()
  4. ssd1306_init_display() → Initialize hardware qua I2C
  5. alloc_chrdev_region() → Xin major:minor number
  6. cdev_init() + cdev_add() → Register character device
  7. class_create() → Tạo class trong /sys/class/
  8. device_create() → Trigger udev tạo /dev/ssd1306
 ↓
Driver ready! /dev/ssd1306 có thể dùng
 ↓
Userspace application: open("/dev/ssd1306", O_RDWR)
 ↓
Kernel: ssd1306_open() được gọi
 ↓
Userspace: write(fd, buffer, 1024)
 ↓
Kernel: ssd1306_write() được gọi
  1. copy_from_user() → Copy data từ userspace vào kernel buffer
  2. mutex_lock() → Lock để tránh race condition
  3. Update hardware qua I2C (gửi buffer tới SSD1306)
  4. mutex_unlock()
  5. Return số bytes đã write
 ↓
Display updated! Pixels hiển thị trên màn hình
 ↓
Userspace: close(fd)
 ↓
Kernel: ssd1306_release() được gọi
 ↓
rmmod ssd1306
 ↓
ssd1306_remove() được gọi
  1. device_destroy() → Xóa /dev/ssd1306
  2. class_destroy()
  3. cdev_del()
  4. unregister_chrdev_region()
  5. kfree() buffer và device structure
 ↓
Driver unloaded
```

---

## 10. Các Khái Niệm Quan Trọng Cần Nhớ

### 10.1. Memory Management

```c
/* Kernel memory allocation */
void *kmalloc(size_t size, gfp_t flags);
void *kzalloc(size_t size, gfp_t flags);  // Zero-initialized
void kfree(const void *ptr);

/* Ví dụ */
struct my_data *data = kzalloc(sizeof(*data), GFP_KERNEL);
if (!data)
    return -ENOMEM;  // Out of memory

// Sử dụng data...

kfree(data);  // Nhớ free!
```

**Lưu ý quan trọng:**
- Luôn check NULL sau khi allocate
- Luôn free memory trong error path
- Sử dụng goto để cleanup dễ dàng

### 10.2. Locking Mechanisms

```c
/* Mutex - Có thể sleep */
#include <linux/mutex.h>

struct mutex lock;
mutex_init(&lock);

mutex_lock(&lock);      // Sleep nếu đã locked
// Critical section
mutex_unlock(&lock);

/* Spinlock - Không được sleep */
#include <linux/spinlock.h>

spinlock_t lock;
spin_lock_init(&lock);

spin_lock(&lock);       // Busy wait nếu đã locked
// Critical section (FAST, no sleep!)
spin_unlock(&lock);

/* Khi nào dùng cái nào? */
- Mutex: Process context, có thể sleep
- Spinlock: Interrupt context, critical section ngắn
```

### 10.3. Error Codes

```c
/* Kernel error codes (negative) */
-ENOMEM     // Out of memory
-EFAULT     // Bad address
-EINVAL     // Invalid argument
-EBUSY      // Device busy
-EIO        // I/O error
-ENODEV     // No such device
-EAGAIN     // Try again
-ETIMEDOUT  // Timeout

/* Return conventions */
Success:  return 0;
Error:    return -ERRNO;

/* Check error */
ret = some_function();
if (ret < 0) {
    // Error occurred
    return ret;
}
```

### 10.4. Copy Between User/Kernel Space

```c
/* Userspace → Kernel */
unsigned long copy_from_user(void *to, 
                            const void __user *from, 
                            unsigned long n);

/* Kernel → Userspace */
unsigned long copy_to_user(void __user *to, 
                          const void *from, 
                          unsigned long n);

/* Return value */
0         = Success
non-zero  = Số bytes không copy được

/* Ví dụ sử dụng */
if (copy_from_user(kernel_buf, user_buf, count)) {
    return -EFAULT;  // Bad address
}
```

**Tại sao cần các hàm này?**
1. **Security**: Kiểm tra userspace pointer hợp lệ
2. **Protection**: Kernel space và user space có page table riêng
3. **Page fault handling**: Có thể trigger page fault an toàn

---

## 11. Struct Quan Trọng Trong Driver

### 11.1. struct ssd1306_dev

```c
struct ssd1306_dev {
    struct i2c_client *client;    // I2C device
    struct cdev cdev;             // Character device
    dev_t dev_num;                // Major:Minor number
    struct class *class;          // Device class
    struct device *device;        // Device object
    u8 *buffer;                   // Frame buffer (1024 bytes)
    struct mutex lock;            // Mutex cho sync
};
```

**Mục đích từng field:**
- `client`: Để communicate với hardware qua I2C
- `cdev`: Character device registration
- `dev_num`: Device number (ví dụ: 240:0)
- `class`: Sysfs class (/sys/class/ssd1306Class/)
- `device`: Device node (/dev/ssd1306)
- `buffer`: Store pixel data trước khi gửi đến hardware
- `lock`: Synchronization giữa các process

### 11.2. struct i2c_driver

```c
static struct i2c_driver ssd1306_driver = {
    .driver = {
        .name = DRIVER_NAME,               // "ssd1306"
        .of_match_table = ssd1306_of_match, // Device tree matching
    },
    .probe = ssd1306_probe,      // Called when device found
    .remove = ssd1306_remove,    // Called when device removed
    .id_table = ssd1306_id,      // I2C device ID table
};
```

### 11.3. struct file_operations

```c
static const struct file_operations ssd1306_fops = {
    .owner   = THIS_MODULE,
    .open    = ssd1306_open,
    .release = ssd1306_release,
    .read    = ssd1306_read,
    .write   = ssd1306_write,
};
```

**Các operations khác (không dùng trong driver này):**
```c
.llseek    // lseek() system call
.poll      // poll() / select() / epoll()
.unlocked_ioctl  // ioctl() for control commands
.mmap      // mmap() for memory mapping
.flush     // Called on close() before release
```

---

## 12. Advanced Topics

### 12.1. Thêm IOCTL Support

**Tại sao cần IOCTL?**
- Control device settings (brightness, contrast, orientation)
- Get device status
- Không phù hợp với read/write interface

**Thêm vào driver:**

```c
/* Define IOCTL commands */
#define SSD1306_IOC_MAGIC  's'
#define SSD1306_IOC_SET_CONTRAST    _IOW(SSD1306_IOC_MAGIC, 1, u8)
#define SSD1306_IOC_GET_CONTRAST    _IOR(SSD1306_IOC_MAGIC, 2, u8)
#define SSD1306_IOC_CLEAR_DISPLAY   _IO(SSD1306_IOC_MAGIC, 3)
#define SSD1306_IOC_INVERT_DISPLAY  _IOW(SSD1306_IOC_MAGIC, 4, int)

/* IOCTL handler */
static long ssd1306_ioctl(struct file *file, unsigned int cmd, 
                          unsigned long arg)
{
    struct ssd1306_dev *dev = file->private_data;
    u8 contrast;
    int invert;
    
    switch (cmd) {
    case SSD1306_IOC_SET_CONTRAST:
        if (copy_from_user(&contrast, (u8 __user *)arg, sizeof(contrast)))
            return -EFAULT;
        
        mutex_lock(&dev->lock);
        ssd1306_write_cmd(dev->client, SSD1306_SET_CONTRAST);
        ssd1306_write_cmd(dev->client, contrast);
        mutex_unlock(&dev->lock);
        break;
        
    case SSD1306_IOC_GET_CONTRAST:
        // Read from device or cached value
        contrast = dev->cached_contrast;
        if (copy_to_user((u8 __user *)arg, &contrast, sizeof(contrast)))
            return -EFAULT;
        break;
        
    case SSD1306_IOC_CLEAR_DISPLAY:
        mutex_lock(&dev->lock);
        ssd1306_clear_display(dev);
        mutex_unlock(&dev->lock);
        break;
        
    case SSD1306_IOC_INVERT_DISPLAY:
        if (copy_from_user(&invert, (int __user *)arg, sizeof(invert)))
            return -EFAULT;
        
        mutex_lock(&dev->lock);
        if (invert)
            ssd1306_write_cmd(dev->client, 0xA7);  // Invert
        else
            ssd1306_write_cmd(dev->client, 0xA6);  // Normal
        mutex_unlock(&dev->lock);
        break;
        
    default:
        return -ENOTTY;  // Invalid ioctl
    }
    
    return 0;
}

/* Thêm vào fops */
static const struct file_operations ssd1306_fops = {
    .owner   = THIS_MODULE,
    .open    = ssd1306_open,
    .release = ssd1306_release,
    .read    = ssd1306_read,
    .write   = ssd1306_write,
    .unlocked_ioctl = ssd1306_ioctl,  // ← Thêm dòng này
};
```

**Userspace usage:**

```c
#include <sys/ioctl.h>
#include "ssd1306_ioctl.h"  // Header chứa defines

int fd = open("/dev/ssd1306", O_RDWR);

// Set contrast
u8 contrast = 200;
ioctl(fd, SSD1306_IOC_SET_CONTRAST, &contrast);

// Get contrast
u8 current_contrast;
ioctl(fd, SSD1306_IOC_GET_CONTRAST, &current_contrast);

// Clear display
ioctl(fd, SSD1306_IOC_CLEAR_DISPLAY);

// Invert display
int invert = 1;
ioctl(fd, SSD1306_IOC_INVERT_DISPLAY, &invert);

close(fd);
```

### 12.2. Sysfs Attributes

**Tạo sysfs files để control từ shell:**

```c
/* Sysfs show function */
static ssize_t contrast_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    struct ssd1306_dev *ssd_dev = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", ssd_dev->cached_contrast);
}

/* Sysfs store function */
static ssize_t contrast_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct ssd1306_dev *ssd_dev = dev_get_drvdata(dev);
    u8 contrast;
    int ret;
    
    ret = kstrtou8(buf, 10, &contrast);
    if (ret)
        return ret;
    
    mutex_lock(&ssd_dev->lock);
    ssd1306_write_cmd(ssd_dev->client, SSD1306_SET_CONTRAST);
    ssd1306_write_cmd(ssd_dev->client, contrast);
    ssd_dev->cached_contrast = contrast;
    mutex_unlock(&ssd_dev->lock);
    
    return count;
}

/* Define attribute */
static DEVICE_ATTR_RW(contrast);

/* Attribute group */
static struct attribute *ssd1306_attrs[] = {
    &dev_attr_contrast.attr,
    NULL,
};

static const struct attribute_group ssd1306_attr_group = {
    .attrs = ssd1306_attrs,
};

/* Register trong probe */
static int ssd1306_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    // ... existing code ...
    
    /* Create sysfs files */
    ret = sysfs_create_group(&ssd1306_device->device->kobj, 
                            &ssd1306_attr_group);
    if (ret) {
        dev_err(&client->dev, "Failed to create sysfs group\n");
        goto err_sysfs;
    }
    
    return 0;
    
err_sysfs:
    device_destroy(ssd1306_device->class, ssd1306_device->dev_num);
    // ... error cleanup ...
}

/* Remove trong remove */
static int ssd1306_remove(struct i2c_client *client)
{
    struct ssd1306_dev *dev = i2c_get_clientdata(client);
    
    sysfs_remove_group(&dev->device->kobj, &ssd1306_attr_group);
    
    // ... existing cleanup ...
}
```

**Sử dụng sysfs từ shell:**

```bash
# Read contrast
cat /sys/class/ssd1306Class/ssd1306/contrast

# Set contrast
echo 150 > /sys/class/ssd1306Class/ssd1306/contrast

# List all attributes
ls -la /sys/class/ssd1306Class/ssd1306/
```

### 12.3. Power Management

**Thêm suspend/resume support:**

```c
#ifdef CONFIG_PM_SLEEP
static int ssd1306_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ssd1306_dev *ssd_dev = i2c_get_clientdata(client);
    
    dev_info(dev, "Suspending...\n");
    
    /* Turn off display */
    mutex_lock(&ssd_dev->lock);
    ssd1306_write_cmd(client, SSD1306_DISPLAY_OFF);
    mutex_unlock(&ssd_dev->lock);
    
    return 0;
}

static int ssd1306_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ssd1306_dev *ssd_dev = i2c_get_clientdata(client);
    
    dev_info(dev, "Resuming...\n");
    
    /* Turn on display */
    mutex_lock(&ssd_dev->lock);
    ssd1306_write_cmd(client, SSD1306_DISPLAY_ON);
    
    /* Restore frame buffer */
    ssd1306_write_data(client, ssd_dev->buffer, SSD1306_BUFFER_SIZE);
    mutex_unlock(&ssd_dev->lock);
    
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ssd1306_pm_ops, ssd1306_suspend, ssd1306_resume);

/* Thêm vào i2c_driver */
static struct i2c_driver ssd1306_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = ssd1306_of_match,
        .pm = &ssd1306_pm_ops,  // ← Thêm dòng này
    },
    .probe = ssd1306_probe,
    .remove = ssd1306_remove,
    .id_table = ssd1306_id,
};
```

---

## 13. Debugging Checklist

### Module Load Issues

```bash
# 1. Check kernel ring buffer
dmesg | tail -50

# 2. Check module loaded
lsmod | grep ssd1306

# 3. Check module info
modinfo ssd1306.ko

# 4. Check kernel version match
modinfo ssd1306.ko | grep vermagic
uname -r

# 5. Load with verbose
insmod ssd1306.ko
```

### Device Not Created

```bash
# 1. Check if probe called
dmesg | grep "probe started"

# 2. Check device tree
ls /sys/firmware/devicetree/base/soc/i2c*/

# 3. Check I2C device detected
i2cdetect -y 1

# 4. Check udev
udevadm monitor  # Then load module

# 5. Manual device node (test)
mknod /dev/ssd1306 c 240 0
```

### I2C Communication Fails

```bash
# 1. Check I2C bus enabled
ls /dev/i2c-*

# 2. Scan for devices
i2cdetect -y 1

# 3. Test read/write
i2cget -y 1 0x3c 0x00
i2cset -y 1 0x3c 0x00 0xAF i

# 4. Check kernel I2C messages
dmesg | grep i2c

# 5. Enable I2C debug
echo 'file drivers/i2c/* +p' > /sys/kernel/debug/dynamic_debug/control
```

### Display Not Working

```bash
# 1. Verify hardware connections
# - VCC → 3.3V
# - GND → GND
# - SDA → GPIO 2 (Pin 3)
# - SCL → GPIO 3 (Pin 5)

# 2. Test with simple write
echo "test" > /dev/ssd1306

# 3. Check driver messages
dmesg | grep ssd1306

# 4. Use test application
./test_ssd1306

# 5. Check display initialization
# Display should light up after probe
```

---

## 14. Complete Example Flow

### Scenario: Write "HELLO" to display

**1. Userspace code:**
```c
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

int main() {
    int fd;
    char buffer[1024] = {0};
    
    // Tạo pattern cho text "HELLO"
    // (simplified - thực tế cần font rendering)
    for (int i = 0; i < 100; i++)
        buffer[i] = 0xFF;
    
    fd = open("/dev/ssd1306", O_RDWR);
    write(fd, buffer, 1024);
    close(fd);
    
    return 0;
}
```

**2. Kernel thực hiện:**

```
open("/dev/ssd1306", O_RDWR)
    ↓
VFS layer: Tìm file /dev/ssd1306
    ↓
VFS: Check permissions
    ↓
VFS: Lookup inode → major=240, minor=0
    ↓
VFS: Find registered cdev với (240,0)
    ↓
VFS: Call cdev->ops->open (ssd1306_open)
    ↓
ssd1306_open():
    file->private_data = ssd1306_device
    return 0
    ↓
Return fd=3 to userspace

write(fd=3, buffer, 1024)
    ↓
VFS: fd=3 → file structure
    ↓
VFS: file->f_op->write (ssd1306_write)
    ↓
ssd1306_write():
    1. Get dev from file->private_data
    2. Check ppos < BUFFER_SIZE
    3. Calculate to_write
    4. mutex_lock(&dev->lock)
    5. copy_from_user(dev->buffer, user_buffer, 1024)
       └→ CPU copy data từ userspace → kernelspace
    6. Setup SSD1306 addressing:
       - Write cmd 0x21 (Column address)
       - Write cmd 0x00 (start col)
       - Write cmd 0x7F (end col)
       - Write cmd 0x22 (Page address)
       - Write cmd 0x00 (start page)
       - Write cmd 0x07 (end page)
    7. ssd1306_write_data(client, buffer, 1024):
       a. Allocate: buf = kmalloc(1025)
       b. buf[0] = 0x40 (data control byte)
       c. memcpy(buf+1, buffer, 1024)
       d. i2c_master_send(client, buf, 1025)
          └→ I2C subsystem
             └→ I2C controller driver (bcm2835-i2c)
                └→ Hardware registers
                   └→ Physical I2C bus (SDA/SCL)
                      └→ SSD1306 chip receives data
                         └→ Display updates!
       e. kfree(buf)
    8. mutex_unlock(&dev->lock)
    9. *ppos += 1024
    10. return 1024
    ↓
Return 1024 to userspace

close(fd=3)
    ↓
VFS: file->f_op->release (ssd1306_release)
    ↓
ssd1306_release():
    return 0
    ↓
VFS: Free file structure
```

---

## 15. Tổng Kết

### Key Takeaways

1. **Device Tree** → Kernel matching → **probe()** được gọi
2. **probe()** tạo character device → **udev** tạo /dev/ssd1306
3. **Userspace open()** → Kernel gọi driver's **open()**
4. **Userspace write()** → **copy_from_user()** → I2C transfer → Hardware update
5. **I2C subsystem** abstraction → Driver không cần biết hardware details
6. **Mutex** bảo vệ shared resources
7. **Error handling** với goto labels
8. **Memory management** với kmalloc/kfree
9. **Logging** với dev_info/dev_err

### Next Steps

Để học sâu hơn về kernel driver development:

1. **Read Linux Device Drivers book** (LDD3)
2. **Study kernel source code**: `drivers/video/fbdev/` 
3. **Practice**: Viết drivers cho sensors khác (BME280, MPU6050)
4. **Learn DMA**: Để optimize large data transfers
5. **Framebuffer driver**: Tích hợp SSD1306 với fb subsystem
6. **DRM driver**: Modern graphics driver framework

### Useful Commands Reference

```bash
# Module
insmod ssd1306.ko
rmmod ssd1306
lsmod | grep ssd1306
modinfo ssd1306.ko

# Device
ls -l /dev/ssd1306
cat /proc/devices | grep ssd1306

# I2C
i2cdetect -y 1
i2cdump -y 1 0x3c
i2cget -y 1 0x3c 0x00
i2cset -y 1 0x3c 0x00 0xAF i

# Kernel messages
dmesg | grep ssd1306
dmesg -w  # Watch mode

# Sysfs
ls /sys/class/ssd1306Class/
cat /sys/class/ssd1306Class/ssd1306/uevent

# Device tree
cat /sys/firmware/devicetree/base/soc/i2c*/ssd1306*/compatible
dtc -I fs /sys/firmware/devicetree/base > current.dts

# Debug
echo 8 > /proc/sys/kernel/printk  # Enable all kernel messages
echo 'file ssd1306.c +p' > /sys/kernel/debug/dynamic_debug/control
```