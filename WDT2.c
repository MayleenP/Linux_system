//一个无人值守的Linux应用系统

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>

int main() {
    int wdt_fd;
    int timeout = 10; // 10秒超时

    // 启动看门狗驱动
    wdt_fd = open("/dev/watchdog/WDT1", O_WRONLY);
    if (wdt_fd == -1) {
        perror("Unable to open watchdog device");
        exit(EXIT_FAILURE);
    }

    // 设置看门狗超时
    if (ioctl(wdt_fd, WDT_timeout, &timeout) != 0) {
        perror("Unable to set watchdog timeout");
        close(wdt_fd);
        exit(EXIT_FAILURE);
    }

    printf("Watchdog timer started with a timeout of %d seconds.\n", timeout);

    while (1) {
        // 喂狗
        if (write(wdt_fd, "\0", 1) != 1) {
            perror("Unable to write to watchdog device");
            close(wdt_fd);
            exit(EXIT_FAILURE);
        }

        printf("Watchdog fed.\n");

        
        sleep(timeout / 2);
    }

    // 关闭看门狗驱动
    close(wdt_fd);

    return 0;
}
