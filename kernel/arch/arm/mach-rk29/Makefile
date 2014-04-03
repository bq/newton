obj-y += timer.o io.o devices.o iomux.o clock.o rk29-pl330.o dma.o ddr.o sram.o memcpy_dma.o reset.o
obj-y += tests.o memtester.o
obj-y += early_printk.o
ifndef CONFIG_DEBUG_LL
obj-y += ../kernel/debug.o
endif
obj-$(CONFIG_RK29_LAST_LOG) += last_log.o
obj-$(CONFIG_USB_GADGET) += usb_detect.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o
obj-$(CONFIG_DDR_FREQ) += ddrfreq.o
obj-$(CONFIG_RK29_I2C_INSRAM) += i2c_sram.o
obj-y += spi_sram.o
obj-$(CONFIG_RK29_PWM_INSRAM) += pwm_sram.o
obj-$(CONFIG_MACH_A10BG3) += board-rk29-A10BG3.o board-A10BG3-key.o board-rk29sdk-rfkill.o board-rk29sdk-power.o
obj-$(CONFIG_MACH_A10BG3DT) += board-rk29-A10BG3.o board-A10BG3-key.o board-rk29sdk-rfkill.o board-rk29sdk-power.o
obj-$(CONFIG_MACH_AN7DG3) += board-rk29-AN7DG3.o board-AN7DG3-key.o board-rk29sdk-rfkill.o board-rk29sdk-power.o
obj-$(CONFIG_MACH_AN7DG3ST) += board-rk29-AN7DG3.o board-AN7DG3-key.o board-rk29sdk-rfkill.o board-rk29sdk-power.o
obj-$(CONFIG_MACH_AN7DG3ST_CP) += board-rk29-AN7DG3.o board-AN7DG3-key.o board-rk29sdk-rfkill.o board-rk29sdk-power.o
obj-$(CONFIG_MACH_AN9G3) += board-rk29-AN9G3.o board-AN9G3-key.o board-rk29sdk-rfkill.o board-rk29sdk-power.o
obj-$(CONFIG_RK29_CHARGE_EARLYSUSPEND) += rk29_charge_lowpower.o
