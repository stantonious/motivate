set(SOURCES main.c)
idf_component_register(SRCS main.c game_tab.c maze_tab.c details_tab.c button_handler.c app_wifi.c maze_client.c plot_tab.c
                       INCLUDE_DIRS . includes
                       REQUIRES core2forAWS esp-tls esp_http_client esp-cryptoauthlib fft esp-dsp nvs_flash CircularBuffer)
