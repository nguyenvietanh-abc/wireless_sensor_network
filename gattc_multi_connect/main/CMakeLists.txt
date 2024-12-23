idf_component_register(SRCS "gattc_multi_connect.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi mqtt nvs_flash input bt)
