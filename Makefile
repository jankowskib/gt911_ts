#gt915 touchpanel driver


obj-$(CONFIG_GT9XX_TOUCHPANEL_DRIVER)	+= gt9xx.o
#gt915 update file
obj-$(CONFIG_GT9XX_TOUCHPANEL_UPDATE)	+= gt9xx.o gt9xx_update.o
#debug tool
obj-$(CONFIG_GT9XX_TOUCHPANEL_DEBUG)	+= gt9xx_tool.o 
