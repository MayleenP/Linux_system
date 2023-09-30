//看门狗定时器Linux驱动系统

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>

static unsigned long *wrr;
static unsigned long *wcs;
static unsigned long *wor;
static unsigned long *wcvh;
static unsigned long *wcvl;
static unsigned long *WDT0;
static unsigned long *WDT1;
static unsigned long sys_cnt;
static unsigned long hresetn;
static unsigned int val;
static unsigned int val2;

// 设置复位hresetn的GPIO控制
static void set_hresetn_low(void)
{
    // 设GPIO为低位（0）
    // 写GPIO寄存器或控制
}

// 结束复位hresetn
static void release_hresetn(void)
{
    // 设GPIO为高位（1）
}

// 写WOR计数值
static void write_wor(unsigned long value)
{
    *wor = value;
}

static void enable_wdt(void)
{
    // 设WCS低位，驱动看门狗
    *wcs |= (1 << 0);
}

static void refresh_wdt(void)
{
    // 写WRR，更新看门狗
    *wrr = 0x0000; 
}

// 看门狗超时
static int WDT_timeout(void)
{
    // 检查sys_cnt是否大于WCV
    unsigned long current_wcv = ((*wcvh) << 32) | (*wcvl);
    if (sys_cnt > current_wcv) {
        
        printk(KERN_ALERT "WDT Timeout!\n");
        
        return 1; // 超时，返1
    }
    return 0; // 没超时，返0

    set_hresetn_low();
}

//启动看门狗
static int wdt_start(void)
{
    WDT0 = ioremap(0x2800A000, sizeof(unsigned long));
    if (!WDT0) 
    {
        printk(KERN_ERR "Failed to map WDT0 memory\n");
        return -ENOMEM;
    }

    wcs = WDT0 + 0x1000;
    wor = WDT0 + 0X1008;
    wcvh = WDT0 + 0x1014;
    wcvl = WDT0 + 0x1010;

    // 初始化
    sys_cnt = 0;

    // 复位启动
    set_hresetn_low();
    
    write_wor(0x3000000); // 超时值

    enable_wdt();



    refresh_wdt();

    // 复位结束
    release_hresetn();

    return 0;
}

static void wdt_exit(void)
{
    // 关闭看门狗
    *wcs &= ~(1 << 0); 

    // 清除
    if (WDT0) {
        iounmap(WDT0);
    }
    
    printk(KERN_ALERT "Exit WDT\n");
}

static int wdt_init(void)
{
    int ret = wdt_start();
    if (ret != 0) {
        printk(KERN_ERR "WDT initialization failed\n");
        return ret;
    }



    return 0;
}

module_init(wdt_init);
module_exit(wdt_exit);
MODULE_LICENSE("GPL");
