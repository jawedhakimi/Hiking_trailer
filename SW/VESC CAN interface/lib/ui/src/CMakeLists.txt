SET(SOURCES ui_comp.c
    ui_Home.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_battery_android_0_45dp_1f1f1f_fill0_wght400_grad0_opsz48_png.c
    ui_img_pin_road_2_35dp_1f1f1f_fill0_wght400_grad0_opsz40_png.c
    ui_img_home_35dp_1f1f1f_fill0_wght400_grad0_opsz40_png.c
    ui_img_info_35dp_1f1f1f_fill0_wght400_grad0_opsz40_png.c
    ui_img_settings_35dp_1f1f1f_fill0_wght400_grad0_opsz40_png.c)

add_library(ui ${SOURCES})
