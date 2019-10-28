# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_IIO_SN65HVS885 sn65hvs885.c)
zephyr_sources_ifdef(CONFIG_IIO_DRV8804 drv8804.c)
zephyr_sources_ifdef(CONFIG_IIO_ADC121S051 adc121s051.c)
