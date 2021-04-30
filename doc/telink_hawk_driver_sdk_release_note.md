### Version：V1.2.1
* SDK version : telink_hawk_driver_sdk v1.2.1.
* This version sdk support hawk.

### Refactoring

* **uart**
  * add restrictions for DMA receive length in UART-DMA mode.
* **flash**
  * In order to reduce the size of ram_code, the code structure of flash has been re-adjusted.

### Note

* **flash**
  * Before calling the FLASH function, please check the power supply voltage of the chip. Only if the detected voltage is greater than the safe voltage value, the FLASH function can be called. Taking into account the factors such as power supply fluctuations, the safe voltage value needs to be greater than the minimum chip operating voltage. For the specific value, please make a reasonable setting according to the specific application and hardware circuit.
  * Risk description: When the chip power supply voltage is relatively low, due to the unstable power supply, there may be a risk of error in the operation of the flash (especially for the write and erase operations. If an abnormality occurs, the firmware and user data may be rewritten, resulting in the final Product failure).
  * If you use the write protection & UID function, you need to update the program(because the chip has a new flash model number).

<hr style="border-bottom:2.5px solid rgb(146, 240, 161)">

### 版本

* SDK版本: telink_hawk_driver_sdk v1.2.1。
* 此版本SDK支持hawk芯片。

### Refactoring

* **uart**
  * DEMO中增加DMA模式接收数据长度的说明。
* **flash**
  * 为了减少ram_code大小，重新调整了flash的代码结构。

### Note

* **flash**
  * 在调用FLASH 函数前，请先做芯片供电电压检测，当检测电压大于安全电压值时，才可以调用FLASH 函数。考虑到电源波动等因素，安全电压值需要比最低芯片工作电压大，具体设置为多少，请根据具体应用以及硬件电路等因素进行合理的设置。
  * 风险描述：当芯片供电电压比较低时，由于电源不稳定，flash的操作可能会有出错的风险（特别是写和擦操作，如果出现异常，可能会造成固件和用户数据被改写，从而导致最终产品失效）。
  * 如果使用了写保护&UID功能，需要更新程序（因为芯片新增了flash型号）。

## V1.2.0
### Bug Fixes
* N/A

### Features
* **Flash:** Add compatible process for different flash type.
* **EMI/BQB:** add EBMI/BQB demo.

### BREAKING CHANGES
* **Flash:**:Modify some Flash API usage for compitible.
 - void flash_read_mid(unsigned char* mid) change to unsigned int flash_read_mid(void),the mid from 3byte change to 4byte.
 - The API of flash_read_status、flash_write_status not provide to external use,you need use the API in the directory of flash depend on mid(eg:flash_write_status_midxxxxxx).
 - The first argument of API int flash_read_mid_uid_with_check( unsigned int *flash_mid ,unsigned char *flash_uid),flash_mid need 4byte space.The second argument need 16byte,has two case,8byte or 16byte,if the flash only has 8byte uid,flash_uid[8:15] will be clear to zero.
 - The API of flash_lock,flash_unlock will be instead of flash_lock_midxxxxxx and flash_unlock_midxxxxxx.
 - delete the useless API。

## V1.2.0
### Bug Fixes
* N/A

### Features
* **Flash:** 为兼容不同的Flash型号做了兼容处理。
* **EMI/BQB:** 新增EMI/BQB例程。

### BREAKING CHANGES
* **Flash:**:为兼容不同的Flash型号，Flash驱动结构做了调整，修改了部分Flash接口调用方式。
 - void flash_read_mid(unsigned char* mid) 改为 unsigned int flash_read_mid(void),mid由3byte改为4byte,最高byte用于区分mid相同但是功能存在差异的flash。
 - 为兼容不同型号的Flash,flash_read_status、flash_write_status不提供给外部使用，需要使用对应接口时，需要根据mid去选择flash目录下的接口(例如：flash_write_status_midxxxxxx)。
 - 接口int flash_read_mid_uid_with_check( unsigned int *flash_mid ,unsigned char *flash_uid)的第一个参数flash_mid需要4个字节空间，第二个参数需要16byte空间，现有flash的uid有两种情况，一种16byte，一种8byte，如果是8byte，flash_uid[8:15]会被清零。
 - 接口flash_lock、flash_unlock由flash_lock_midxxxxxx和flash_unlock_midxxxxxx替代。
 - 删除不使用的接口。

---

## V1.1.0
### Bug Fixes

* **pm:** fix suspend crash issue(xtal work abnormal after wakeup)
* **pwm:** solve the problem that the driver cannot generate interrupts
 - fix the problem of incorrect length setting in pwm_ir_dma_fifi mode

### Features

* **rf:** add intenal and external cap support
 - update rffe
 - modify zigbee 2M mode setting in order to solve the problem of high packet loss rate
* **interrupt：** clear gpio interrupt source before enable to avoid a unexpected interrupt
* **uart:** update uart_clear_parity_error function
* **gpio:** update gpio_set_func function
* **flash:** modify flash_read_mid_uid_with_check function to supplement all flash
* **pm:** add 32k calibration function,update pm_get_32k_tick function
* **uart:** after wakeup from pm,you must reset the value of 'uart_TxIndex' before sending data for no dma mode
* **pwm:** update pwm pin


### BREAKING CHANGES
* the crystal may work abnormal after suspend wake up,need add manual kick api etc.Need SDK update.

## V1.1.0
### Bug Fixes

* **pm:** 解决suspend唤后概率性出现的晶振不能起振或者起振后工作不正常的问题
* **pwm:** 解决不能产生中断的问题
 - 解决pwm_ir_dma_fifo模式错误的长度设置

### Features

* **rf:** 新增内外部电容支持
 - 更新rffe
 - 修改zigee 2M设置解决丢包率高问题
* **interrupt:** 使能中断前清除gpio中断源避免终端误触发
* **uart:** 更新uart_clear_parity_error接口
* **gpio:** 更新gpio_set_func接口
* **flash:** 修改flash_read_mid_uid with_check接口适应不同flash
* **pm:** 增加32k校准函数，更新获取32K_tick接口
* **uart:** 从低功耗模式唤醒后，使用非dma模式发包前需要将“uart_TxIndex”清零。
* **pwm:** 更新pwm pin

### BREAKING CHANGES
* suspend唤醒后晶体概率性工作不正常，需要加手动kick等操作，需要各个SDK更新。

---

## V1.0.0
* The first release version

## V1.0.0
* 初始release版本




标签（空格分隔）： 未分类

---

在此输入正文




