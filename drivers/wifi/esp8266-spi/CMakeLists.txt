if(CONFIG_WIFI_ESP8266_SPI)
  zephyr_include_directories(
    .
    )
  zephyr_sources(
    esp8266-spi.c
    )
endif()
