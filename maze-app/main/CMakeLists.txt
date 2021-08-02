set(SOURCES main.c)
idf_component_register(SRCS main.c model.cc constants.cc tfl-example.cpp game_tab.c tilt_maze_tab.c maze_tab.c  button_handler.c app_wifi.c maze_client.c 
                       INCLUDE_DIRS . includes
                       REQUIRES mot_mqtt_client tfmicro utilities widgets core2forAWS esp-tls esp_http_client esp-cryptoauthlib fft esp-dsp nvs_flash buffers CircularBuffer)
