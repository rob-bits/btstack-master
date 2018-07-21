deps_config := \
	/home/R0b1T5/esp-idf/components/app_trace/Kconfig \
	/home/R0b1T5/esp-idf/components/aws_iot/Kconfig \
	/home/R0b1T5/esp-idf/components/bt/Kconfig \
	/home/R0b1T5/esp-idf/components/driver/Kconfig \
	/home/R0b1T5/esp-idf/components/esp32/Kconfig \
	/home/R0b1T5/esp-idf/components/esp_adc_cal/Kconfig \
	/home/R0b1T5/esp-idf/components/esp_http_client/Kconfig \
	/home/R0b1T5/esp-idf/components/ethernet/Kconfig \
	/home/R0b1T5/esp-idf/components/fatfs/Kconfig \
	/home/R0b1T5/esp-idf/components/freertos/Kconfig \
	/home/R0b1T5/esp-idf/components/heap/Kconfig \
	/home/R0b1T5/esp-idf/components/libsodium/Kconfig \
	/home/R0b1T5/esp-idf/components/log/Kconfig \
	/home/R0b1T5/esp-idf/components/lwip/Kconfig \
	/home/R0b1T5/esp-idf/components/mbedtls/Kconfig \
	/home/R0b1T5/esp-idf/components/openssl/Kconfig \
	/home/R0b1T5/esp-idf/components/pthread/Kconfig \
	/home/R0b1T5/esp-idf/components/spi_flash/Kconfig \
	/home/R0b1T5/esp-idf/components/spiffs/Kconfig \
	/home/R0b1T5/esp-idf/components/tcpip_adapter/Kconfig \
	/home/R0b1T5/esp-idf/components/vfs/Kconfig \
	/home/R0b1T5/esp-idf/components/wear_levelling/Kconfig \
	/home/R0b1T5/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/R0b1T5/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/R0b1T5/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/R0b1T5/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
