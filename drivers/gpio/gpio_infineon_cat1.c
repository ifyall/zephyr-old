/* Copyright (c) 2021 Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT  infineon_mtbhal_gpio

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/gpio.h>

#include "gpio_utils.h"
#include "cyhal_gpio.h"
#include "cy_gpio.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_cat1);

typedef void (*config_func_t)(const struct device *dev);

struct gpio_cat1_config {
    /* gpio_driver_config needs to be first */
    struct gpio_driver_config   common;
    cyhal_gpio_callback_data_t *cb_data_ptr;
    GPIO_PRT_Type              *regs;
    uint8_t                     ngpios;
    uint8_t                     intr_priority;
};

struct gpio_cat1_data {
    /* gpio_driver_data needs to be first */
    struct gpio_driver_data common;
    
    /* device's owner of this data */
    const struct device *dev;

    /* callbacks list */
    sys_slist_t callbacks;
};

#define DEV_CFG(dev) \
    ((const struct gpio_cat1_config * const)(dev)->config)

#define DEV_DATA(dev) \
    ((struct gpio_cat1_data * const)(dev)->data)

/* Get port number by calculation difference from current port address and
 * GPIO base address divided by GPIO structure size. */
#define GET_PORT_NUM(dev) \
    (((uint32_t) DEV_CFG(dev)->regs - CY_GPIO_BASE) / GPIO_PRT_SECTION_SIZE)

#define GET_DEV_OBJ(n)  DEVICE_DT_GET_OR_NULL(DT_NODELABEL(n))


static int gpio_cat1_config(const struct device *dev,
                            gpio_pin_t pin, gpio_flags_t flags)
{
    uint32_t gpio_pin = CYHAL_GET_GPIO(GET_PORT_NUM(dev), pin);
    cyhal_gpio_drive_mode_t gpio_mode;
    cyhal_gpio_direction_t gpio_dir;
    bool pin_val; 

    switch (flags & (GPIO_INPUT | GPIO_OUTPUT)) {
    case GPIO_INPUT:
        gpio_dir = CYHAL_GPIO_DIR_INPUT;

        if ((flags & GPIO_PULL_UP) && (flags & GPIO_PULL_DOWN)) 
        {
            gpio_mode = CYHAL_GPIO_DRIVE_PULLUPDOWN;
            pin_val   = 0;
        } 
        else if (flags & GPIO_PULL_UP) 
        {
            gpio_mode = CYHAL_GPIO_DRIVE_PULLUP;
            pin_val   = 1;
        } 
        else if (flags & GPIO_PULL_DOWN) 
        {
            gpio_mode = CYHAL_GPIO_DRIVE_PULLDOWN;
            pin_val   = 0;
        } 
        else 
        {
            gpio_mode = CYHAL_GPIO_DRIVE_ANALOG;
            pin_val   = 0;
        }
        break;

    case GPIO_OUTPUT:
        gpio_dir = CYHAL_GPIO_DIR_OUTPUT;
        if (flags & GPIO_SINGLE_ENDED) 
        {
            if (flags & GPIO_LINE_OPEN_DRAIN)
            {
                gpio_mode = CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW;
                pin_val = 1;
            }
            else
            {
                gpio_mode = CYHAL_GPIO_DRIVE_OPENDRAINDRIVESHIGH;
                pin_val = 0;
            }
        } 
        else 
        {
            gpio_mode = CYHAL_GPIO_DRIVE_STRONG;
            pin_val = (flags & GPIO_OUTPUT_INIT_HIGH) ? 1 : 0;
        }
        break;

    case GPIO_DISCONNECTED:
        cyhal_gpio_free(gpio_pin);
        return 0;

    default:
        return -ENOTSUP;
    }

    (void) cyhal_gpio_init(gpio_pin, gpio_dir, gpio_mode, pin_val);
    return 0;
}


static int gpio_cat1_port_get_raw(const struct device *dev,
                                  uint32_t *value)
{
    GPIO_PRT_Type * const base = DEV_CFG(dev)->regs;
    *value = GPIO_PRT_IN(base);

    return 0;
}


static int gpio_cat1_port_set_masked_raw(const struct device *dev,
                                         uint32_t mask, uint32_t value)
{
    GPIO_PRT_Type * const base = DEV_CFG(dev)->regs;
    GPIO_PRT_OUT(base) = (GPIO_PRT_IN(base) & ~mask) | (mask & value);

    return 0;
}

static int gpio_cat1_port_set_bits_raw(const struct device *dev,
                                       uint32_t mask)
{
    GPIO_PRT_Type * const base = DEV_CFG(dev)->regs;
    GPIO_PRT_OUT_SET(base) = mask;

    return 0;
}


static int gpio_cat1_port_clear_bits_raw(const struct device *dev,
                                         uint32_t mask)
{
    GPIO_PRT_Type * const base = DEV_CFG(dev)->regs;
    GPIO_PRT_OUT_CLR(base) = mask;

    return 0;
}


static int gpio_cat1_port_toggle_bits(const struct device *dev,
                                      uint32_t mask)
{
    GPIO_PRT_Type * const base = DEV_CFG(dev)->regs;
    GPIO_PRT_OUT_INV(base) = mask;

    return 0;
}


static uint32_t gpio_cat1_get_pending_int(const struct device *dev)
{
    const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
    GPIO_PRT_Type * const base = cfg->regs;

    return GPIO_PRT_INTR_MASKED(base);
}


static void gpio_event_callback(void *callback_arg, cyhal_gpio_event_t event)
{
    ARG_UNUSED(event);
    uint32_t port_num = CYHAL_GET_PORT((uint32_t) callback_arg);
    uint32_t pin_num = CYHAL_GET_PIN((uint32_t) callback_arg);

    static const struct device *const port_dev_obj[IOSS_GPIO_GPIO_PORT_NR] =
    {
        GET_DEV_OBJ(gpio_prt0),  GET_DEV_OBJ(gpio_prt1),  GET_DEV_OBJ(gpio_prt2), 
        GET_DEV_OBJ(gpio_prt3),  GET_DEV_OBJ(gpio_prt4),  GET_DEV_OBJ(gpio_prt5), 
        GET_DEV_OBJ(gpio_prt6),  GET_DEV_OBJ(gpio_prt7),  GET_DEV_OBJ(gpio_prt8),
        GET_DEV_OBJ(gpio_prt9),  GET_DEV_OBJ(gpio_prt10), GET_DEV_OBJ(gpio_prt11),
        GET_DEV_OBJ(gpio_prt12), GET_DEV_OBJ(gpio_prt13), GET_DEV_OBJ(gpio_prt14)
    };

    const struct device *dev = port_dev_obj[port_num];

    /* Goes through and fires callback from a callback list */
    gpio_fire_callbacks(&DEV_DATA(dev)->callbacks, dev, 1 << pin_num);

    /* NOTE: cyhal gpio handles cleaning of interrupts */
}


static int gpio_cat1_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
                                             enum gpio_int_mode mode, enum gpio_int_trig trig)
{
    const struct gpio_cat1_config * const cfg = DEV_CFG(dev);
    cyhal_gpio_event_t event = CYHAL_GPIO_IRQ_NONE;

    switch (trig)
    {
    case GPIO_INT_TRIG_LOW:
        event = CYHAL_GPIO_IRQ_FALL;
        break;

    case GPIO_INT_TRIG_HIGH:
        event = CYHAL_GPIO_IRQ_RISE;
        break;

    case GPIO_INT_TRIG_BOTH:
        event = CYHAL_GPIO_IRQ_BOTH;
        break;

    default:
        event = CYHAL_GPIO_IRQ_NONE;
        break;
    }    

    uint32_t gpio_pin = CYHAL_GET_GPIO(GET_PORT_NUM(dev), pin);
    cyhal_gpio_callback_data_t *cb_data_ptr = cfg->cb_data_ptr;

    /* Find index of free callback data structure */
    uint32_t index;
    for (index = 0u; i < cfg->ngpios; index++)
    {
        if ( (cb_data_ptr[index].callback == NULL) || (cb_data_ptr[index].pin == gpio_pin) )
        {
            break;
        }
    }

    if (index != cfg->ngpios)
    {    
        /* Store callback data: gpio callback and gpio device driver handle */
        cb_data_ptr[index].callback = &gpio_event_callback;
        cb_data_ptr[index].callback_arg = (void *)(gpio_pin);

        /* Register/clear a callback handler for pin events */
        cyhal_gpio_register_callback(gpio_pin, (mode == GPIO_INT_MODE_DISABLED) ?
                                                NULL : &cb_data_ptr[index]);
        
        /* Enable/disable the specified GPIO event */
        cyhal_gpio_enable_event(gpio_pin, event, cfg->intr_priority,
                                mode == GPIO_INT_MODE_DISABLED ? false : true);
        return 0;
    }
    else
    {
        return -EINVAL;
    }
}


static int gpio_cat1_manage_callback(const struct device *port,
                                     struct gpio_callback *callback,
                                     bool set)
{
    return gpio_manage_callback(&DEV_DATA(port)->callbacks, callback, set);
}
    

static const struct gpio_driver_api gpio_cat1_api = {
    .pin_configure           = gpio_cat1_config,
    .port_get_raw            = gpio_cat1_port_get_raw,
    .port_set_masked_raw     = gpio_cat1_port_set_masked_raw,
    .port_set_bits_raw       = gpio_cat1_port_set_bits_raw,
    .port_clear_bits_raw     = gpio_cat1_port_clear_bits_raw,
    .port_toggle_bits        = gpio_cat1_port_toggle_bits,
    .pin_interrupt_configure = gpio_cat1_pin_interrupt_configure,
    .manage_callback         = gpio_cat1_manage_callback,
    .get_pending_int         = gpio_cat1_get_pending_int,
};


int gpio_cat1_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

#define GPIO_CAT1_INIT(n)                                               \
                                                                        \
    cyhal_gpio_callback_data_t                                          \
        _cat1_gpio##n##_cb_data[DT_INST_PROP(n, ngpios)];               \
                                                                        \
    static const struct gpio_cat1_config _cat1_gpio##n##_config = {     \
        .common = {                                                     \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),        \
        },                                                              \
        .intr_priority = DT_INST_IRQ_BY_IDX(n, 0, priority),            \
        .cb_data_ptr   = _cat1_gpio##n##_cb_data,                       \
        .ngpios        = DT_INST_PROP(n, ngpios),                       \
        .regs          = (GPIO_PRT_Type *)DT_INST_REG_ADDR(n),          \
    };                                                                  \
                                                                        \
    static struct gpio_cat1_data _cat1_gpio##n##_data = { 0 };          \
                                                                        \
    DEVICE_DT_INST_DEFINE(n, gpio_cat1_init, NULL,                      \
                          &_cat1_gpio##n##_data,                        \
                          &_cat1_gpio##n##_config, POST_KERNEL,         \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,           \
                          &gpio_cat1_api);


DT_INST_FOREACH_STATUS_OKAY(GPIO_CAT1_INIT)
