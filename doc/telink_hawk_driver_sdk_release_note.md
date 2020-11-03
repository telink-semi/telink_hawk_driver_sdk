## V1.1.0
### Bug Fixes

* **pm:** 解决suspend唤后概率性出现的晶振不能起振或者起振后工作不正常的问题
* **pm:** fix suspend crash issue(xtal work abnormal after wakeup)
* **pwm:** 解决不能产生中断的问题
* **pwm:** solve the problem that the driver cannot generate interrupts
 - 解决pwm_ir_dma_fifo模式错误的长度设置
 - fix the problem of incorrect length setting in pwm_ir_dma_fifi mode

 
### Features

* **rf:** 新增内外部电容支持
* **rf:** add intenal and external cap support
 - 更新rffe
 - update rffe
 - 修改zigee 2M设置解决丢包率高问题
 - modify zigbee 2M mode setting in order to solve the problem of high packet loss rate
* **interrupt:** 使能中断前清除gpio中断源避免终端误触发
* **interrupt：** clear gpio interrupt source before enable to avoid a unexpected interrupt
* **uart:** 更新uart_clear_parity_error接口
* **uart:** update uart_clear_parity_error function
* **gpio:** 更新gpio_set_func接口
* **gpio:** update gpio_set_func function
* **flash:** 修改flash_read_mid_uid with_check接口适应不同flash
* **flash:** modify flash_read_mid_uid_with_check function to supplement all flash
* **pm:** 增加32k校准函数，更新获取32K_tick接口
* **pm:** add 32k calibration function,update pm_get_32k_tick function
* **uart:** 从低功耗模式唤醒后，使用非dma模式发包前需要将“uart_TxIndex”清零。
* **uart:** after wakeup from pm,you must reset the value of 'uart_TxIndex' before sending data for no dma mode
* **pwm:** 更新pwm pin
* **pwm:** update pwm pin


### BREAKING CHANGES
* N/A

标签（空格分隔）： 未分类

---

在此输入正文




