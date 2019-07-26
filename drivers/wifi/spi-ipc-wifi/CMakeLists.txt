# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_SPI_IPC)
  zephyr_sources_ifdef(CONFIG_WIFI_SPI_IPC spi-ipc-wifi.c)
endif()

