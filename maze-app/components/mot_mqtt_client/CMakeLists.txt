
idf_component_register(SRCS "mot_client.c" "mot_client2.c"
                       INCLUDE_DIRS . includes
                       EMBED_TXTFILES  ../../certs/root_CA.crt ../../certs/mot0_private.pem ../../certs/mot0_public.pem ../../certs/mot0_device.pem
                       REQUIRES esp-aws-iot mqtt json
                       )
