# SPDX-License-Identifier: GPL-3.0-or-later

idf_component_register(SRCS "lowpower.c"
                    INCLUDE_DIRS "include"
                    REQUIRES soc ulp esp_adc)
set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_riscv_sources "riscv/main.c")
set(ulp_exp_dep_srcs "lowpower.c")

ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")
