# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_SPI_IPC spi-ipc.c spi-ipc-util.c)
zephyr_sources_ifdef(CONFIG_SPI_IPC_MGMT spi-ipc-mgmt.c)
zephyr_sources_ifdef(CONFIG_SPI_IPC_ETHERNET spi-ipc-ethernet.c)
zephyr_sources_ifdef(CONFIG_SPI_IPC_WIFI_MGMT spi-ipc-wifi-mgmt.c)
