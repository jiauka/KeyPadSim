set(srcs
    "main.c"
    "myBle.c" "misc.c" "peer.c"
)


set(reqs
    "OwnN2K"
    "bt"
    "nvs_flash"

)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ""
    REQUIRES ${reqs}
)
