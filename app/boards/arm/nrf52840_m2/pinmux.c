/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/sys_io.h>
#include <devicetree.h>

static int pinmux_nrf52840_m2_init(const struct device *port) {
    ARG_UNUSED(port);

    const struct device *p0 = device_get_binding("GPIO_0");
    gpio_pin_configure(p0, 28, GPIO_OUTPUT);
    gpio_pin_set(p0, 28, 1);
    return 0;
}

SYS_INIT(pinmux_nrf52840_m2_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
