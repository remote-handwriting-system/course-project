idf_component_register(SRCS "main.c"
                            "double-angles-reader.c"
                            "2d-pos-encoder.c"
                            "data-sender.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver)
