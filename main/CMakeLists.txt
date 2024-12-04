#idf_component_register(SRCS "esp32_serial_transport.c" "main.c" "crash_micro_ros.c" "crash_hello_world.c"
#                    INCLUDE_DIRS "")
idf_component_register(
#    SRCS main.c esp32_serial_transport.c 
    SRCS main.c
    INCLUDE_DIRS .
)
