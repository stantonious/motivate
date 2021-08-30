set(SOURCES main.c)
idf_component_register(SRCS etch_tab.c imu_task.c pred_tab.c train_tab.c mot-imu-model.cc mot-imu-tf.cpp main.c model.cc constants.cc game_tab.c tilt_maze_tab.c maze_tab.c  button_handler.c app_wifi.c maze_client.c 
                       INCLUDE_DIRS . includes
                       EMBED_TXTFILES  ../certs/root_CA.crt ../certs/mot0_private.pem ../certs/mot0_public.pem ../certs/mot0_device.pem
                       REQUIRES mot_mqtt_client tfmicro utilities widgets core2forAWS esp-tls esp_http_client esp-cryptoauthlib fft esp-dsp nvs_flash buffers CircularBuffer)
