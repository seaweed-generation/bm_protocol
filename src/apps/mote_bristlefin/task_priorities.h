//
// Define all task priorities here for ease of access/comparison
// Trying to keep them sorted by priority here
//

#define PCA9535_IRQ_TASK_PRIORITY 20

#define DEFAULT_BOOT_TASK_PRIORITY  16

#define ADIN_SPI_TASK_PRIORITY          15
#define ADIN_GPIO_TASK_PRIORITY         15
#define ADIN_SERVICE_TASK_PRIORITY      14
#define BM_DFU_EVENT_TASK_PRIORITY      11
#define BM_L2_TX_TASK_PRIORITY          7

#define GPIO_ISR_TASK_PRIORITY 6

#define BCMP_TASK_PRIORITY	5
#define STRESS_TASK_PRIORITY 5

#define MIDDLEWARE_NET_TASK_PRIORITY 4
#define BRIDGE_POWER_TASK_PRIORITY  4

#define SENSOR_SAMPLER_TASK_PRIORITY 3
#define USB_TASK_PRIORITY 3

#define SERIAL_TX_TASK_PRIORITY 2
#define CONSOLE_RX_TASK_PRIORITY 2

#define CLI_TASK_PRIORITY 1
#define DEFAULT_TASK_PRIORITY 1

#define IWDG_TASK_PRIORITY 0
