set(SOURCES main.c)
idf_component_register(SRCS main.c test.c imu.c button_handler.c
                       INCLUDE_DIRS . includes
                       REQUIRES core2forAWS esp-cryptoauthlib fft math nvs_flash)
