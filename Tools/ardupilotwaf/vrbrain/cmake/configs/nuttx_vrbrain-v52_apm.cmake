include(configs/nuttx_vrbrain-common_apm)

list(APPEND config_module_list
    drivers/boards/vrbrain-v52
    drivers/pwm_input
    lib/rc
)
