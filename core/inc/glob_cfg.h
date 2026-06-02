#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

// 窄带通信
#define DEV_DBB "/dev/ttyS2"

// 定位
#define DEV_GNSS "/dev/ttyS3"

#define DEV_LoRa "/dev/ttyS5"

#define DEV_4G "/dev/ttyS4"

// 高有效，供电
#define DEV_GNSS_DBB_IO_POWER (4)
// 低电平有效，复位
#define DEV_GNSS_DBB_IO_NRESET (5)
// 窄带通信发射指示， LVTTL 3.3V
#define DEV_DBB_TX_FLAG (6)
// 高有效，窄带通信睡眠
#define DEV_DBB_A2B_SLEEP (10)
// 高有效，窄带通信唤醒
#define DEV_DBB_A2B_WAKEUP (11)

#define DEV_DBB_LED "user-led3"

// 高有效，复位
#define DEV_LoRa_IO_RESET (56)
// 开机时，将改引脚至为输出，高电平
// 正常工作时，该引脚低电平表示忙状态，高电平表示空闲状态。
#define DEV_LoRa_IO_AUX (57)

#define DEV_LoRa_LED "user-led4"

// 高有效，供电
#define DEV_4G_IO_POWER (8)
// 高有效，复位
#define DEV_4G_IO_RESET (9)

#define DEV_4G_LED "user-led2"

// 控制USB 2选1；0：3506_USB0到typec0，1：DBB_USB到typec0
#define DEV_USB_IO_SW0 (110)
// 控制USB 2选1；0：typec1到EC_USB，1：3506_USB1到EC_USB
#define DEV_USB_IO_SW1 (109)

#ifdef __cplusplus
}
#endif
