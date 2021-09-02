
idf_component_register(SRCS "mot_mqtt_client.c"
                       INCLUDE_DIRS . includes
                       REQUIRES esp-aws-iot mqtt json
                       )
