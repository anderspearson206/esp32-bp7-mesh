idf_component_register(SRCS "mesh_main.c"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_timer esp_netif esp_driver_uart
                    INCLUDE_DIRS ".")
