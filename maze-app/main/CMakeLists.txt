set(SOURCES main.c)
idf_component_register(SRCS globals.c train_tab.c main.c constants.cc game_tab.c  maze_tab.c button_handler.c app_wifi.c maze_client.c 
                       INCLUDE_DIRS . includes
                       EMBED_TXTFILES  ../certs/root_CA.crt ../certs/mot0_private.pem ../certs/mot0_public.pem ../certs/mot0_device.pem
                       REQUIRES mot_imu mot_model mot_mqtt_client tfmicro utilities widgets core2forAWS esp-tls esp_http_client esp-cryptoauthlib fft esp-dsp mot_math nvs_flash buffers CircularBuffer)
