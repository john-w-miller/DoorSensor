idf_component_register(
    SRCS "lsm303agr_reg.c" "lsm6ds3tr-c_reg.c" "imu.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)