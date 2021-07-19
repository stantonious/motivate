set(SOURCES main.c)
idf_component_register(SRCS main.c game_tab.c maze_tab.c details_tab.c button_handler.c
                       INCLUDE_DIRS . includes
                       REQUIRES core2forAWS esp-cryptoauthlib fft nvs_flash)
