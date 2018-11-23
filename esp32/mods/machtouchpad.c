/*
 * This file is derived from the MicroPython project, http://micropython.org/
 *
 * Copyright (c) 2018 Pat Deegan, Psychogenic Technologies INC 
 * https://psychogenic.com
 *
 * This software is licensed under the GNU GPL version 3 or any later version,
 * with permitted additional terms. For more information see the Pycom Licence
 * v1.0 document supplied with this file, or available at:
 * https://www.pycom.io/opensource/licensing
 */

/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Nick Moore
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "mpexception.h"
#include "mpirq.h"
#include "machtouchpad.h"
#include "machpin.h"
#include "modmachine.h"


/* **** Touchpad object structure and instances **** */
typedef struct _mach_touchpad_obj_t {
    mp_obj_base_t base;
    bool               is_init;
    gpio_num_t         gpio_id;
    touch_pad_t        touchpad_id;
    mp_obj_t           callback;
} machtouchpad_obj_t;




#define NUM_TOUCHPADS_AVAIL 10
STATIC machtouchpad_obj_t touchpad_obj[NUM_TOUCHPADS_AVAIL] = {
    // type                is_init gpio id      touchpad id     callback
    {{&mach_touchpad_type}, false, GPIO_NUM_4,  TOUCH_PAD_NUM0, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_0,  TOUCH_PAD_NUM1, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_2,  TOUCH_PAD_NUM2, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_15, TOUCH_PAD_NUM3, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_13, TOUCH_PAD_NUM4, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_12, TOUCH_PAD_NUM5, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_14, TOUCH_PAD_NUM6, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_27, TOUCH_PAD_NUM7, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_33, TOUCH_PAD_NUM8, mp_const_none},
    {{&mach_touchpad_type}, false, GPIO_NUM_32, TOUCH_PAD_NUM9, mp_const_none},
};

/* **** touch pad ISR related functions **** */
STATIC void tp_interrupt_queue_handler(void *arg) {
    // this function will be called by the interrupt thread
    machtouchpad_obj_t *tp = arg;
    if (tp->callback && tp->callback != mp_const_none) {
        mp_call_function_1(tp->callback, tp);
    }
}

STATIC IRAM_ATTR void call_tp_interrupt_handler (machtouchpad_obj_t * tp) {
    if (tp->callback) {
        // pass it to the queue
        mp_irq_queue_interrupt(tp_interrupt_queue_handler, tp);
    }
}

STATIC IRAM_ATTR void touchpad_isr(void * arg) {
    machtouchpad_obj_t * tp = NULL;
    uint32_t pad_intr = touch_pad_get_status();
    touch_pad_clear_status();

    for (uint8_t i=0; i<NUM_TOUCHPADS_AVAIL; i++) {
            tp = &(touchpad_obj[i]);
	    if (tp && tp->is_init && ((pad_intr >> tp->touchpad_id) & 0x01) ) {
		call_tp_interrupt_handler(tp);
            }
    }
}




/* **** system-level touch-pad functions **** */

STATIC void touchpad_disableunused_isr() {
    // only leave interrupts enabled if we have anyone with a trigger callback 
    // still watching
    for (uint8_t i=0; i<NUM_TOUCHPADS_AVAIL; i++) {
        if (touchpad_obj[i].is_init && touchpad_obj[i].callback != mp_const_none) {
            return;
        }
    }
    touch_pad_intr_disable();
}

STATIC void touchpad_enable_isrifrequired() {
    // turn on the interrupts if anyone has a callback set
    //
    for (uint8_t i=0; i<NUM_TOUCHPADS_AVAIL; i++) {
        if (touchpad_obj[i].is_init && touchpad_obj[i].callback != mp_const_none) {
            touch_pad_intr_enable();
            return;
        }
    }
}

static uint8_t initialized = 0;
STATIC void lowlevel_touchpad_init() {
    if (initialized) {
        return;
    }
    // first init, do all the setup
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_filter_start(10);
    touch_pad_isr_register(touchpad_isr, NULL);
    touchpad_enable_isrifrequired();
    initialized = 1;
}


STATIC void lowlevel_touchpad_deinit() {
    // called on every object deinit, so we check if anyone is 
    // left to use this
    if (!initialized) {
        // hm, this is a little weird... how did we even get here?
	return;
    } 

    for (uint8_t i=0; i<NUM_TOUCHPADS_AVAIL; i++) {
        if (touchpad_obj[i].is_init) {
            // at least one user, we stay up and running
	    touchpad_disableunused_isr(); // but disable ISR unless needed
	    return;
	}
    }
    // no one left around
    initialized = 0;
    touch_pad_intr_disable();
    touch_pad_filter_stop();
    touch_pad_filter_delete();
    touch_pad_deinit();
}




STATIC const mp_arg_t touchpad_init_args[] = {
    { MP_QSTR_pin,         MP_ARG_OBJ, {.u_obj = mp_const_none} },
    { MP_QSTR_trigger,      MP_ARG_KW_ONLY  |  MP_ARG_OBJ, {.u_obj = mp_const_none} },
};

STATIC mp_obj_t mtp_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw,
        const mp_obj_t *all_args) {

    gpio_num_t pin_id = 0;
    mp_map_t kw_args;
    //parse args
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    mp_arg_val_t args[MP_ARRAY_SIZE(touchpad_init_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(args), touchpad_init_args, args);

    pin_obj_t *_pin = (pin_obj_t *)pin_find(all_args[0]);
    if (_pin && _pin != mp_const_none) {
        pin_id = _pin->pin_number;
    } else {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_resource_not_avaliable));
        return mp_const_none;
    }

    machtouchpad_obj_t *self = NULL;
    for (int i = 0; i < MP_ARRAY_SIZE(touchpad_obj); i++) {
        if (pin_id == touchpad_obj[i].gpio_id) { self = &touchpad_obj[i]; break; }
    }
    if (!self) nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "invalid pin for touchpad"));

    lowlevel_touchpad_init();
    self->is_init = true;
    // leave untouched, so you can fetch repeatedly without changing
    // config? self->callback = mp_const_none;
    esp_err_t err = touch_pad_config(self->touchpad_id, 0);

    if (err != ESP_OK) {
	    mp_raise_ValueError("Touch pad error");
	    return mp_const_none;
    }
    if (args[1].u_obj) {
        self->callback = args[1].u_obj;
    }

    touchpad_enable_isrifrequired();
    return MP_OBJ_FROM_PTR(self);
}


/* mtp_filter_period -- class method, set period for all. */
STATIC mp_obj_t mtp_filter_period(mp_obj_t period_in) {
    uint32_t period = mp_obj_get_int(period_in);
    lowlevel_touchpad_init();
    esp_err_t err = touch_pad_set_filter_period(period);
    if (err == ESP_OK) return mp_const_none;
    mp_raise_ValueError("Error setting period");
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mtp_filter_period_obj, mtp_filter_period);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(mtp_filter_period_static_obj, &mtp_filter_period_obj);


/* *** instance methods *** */

STATIC mp_obj_t mtp_id(mp_obj_t self_in) {
    machtouchpad_obj_t *self = self_in;
    return MP_OBJ_NEW_SMALL_INT(self->touchpad_id);
}
MP_DEFINE_CONST_FUN_OBJ_1(mtp_id_obj, mtp_id);

STATIC mp_obj_t mtp_config(mp_obj_t self_in, mp_obj_t value_in) {
    machtouchpad_obj_t *self = self_in;
    uint16_t value = mp_obj_get_int(value_in);
    if (! self->is_init ) {
        mp_raise_ValueError("pad de-init");
	return mp_const_none;
    }
    esp_err_t err = touch_pad_config(self->touchpad_id, value);
    if (err == ESP_OK) return mp_const_none;
    mp_raise_ValueError("Touch pad error");
}
MP_DEFINE_CONST_FUN_OBJ_2(mtp_config_obj, mtp_config);

STATIC mp_obj_t mtp_threshold(mp_obj_t self_in, mp_obj_t value_in) {
    machtouchpad_obj_t *self = self_in;
    uint16_t value = mp_obj_get_int(value_in);
    if (! self->is_init ) {
        mp_raise_ValueError("pad de-init");
	return mp_const_none;
    }
    esp_err_t err = touch_pad_set_thresh(self->touchpad_id, value);
    if (err == ESP_OK) return mp_const_none;
    mp_raise_ValueError("Touch pad error");
}
MP_DEFINE_CONST_FUN_OBJ_2(mtp_threshold_obj, mtp_threshold);

STATIC mp_obj_t mtp_read(mp_obj_t self_in) {
    machtouchpad_obj_t *self = self_in;
    uint16_t value;
    esp_err_t err = touch_pad_read(self->touchpad_id, &value);
    if (err == ESP_OK) return MP_OBJ_NEW_SMALL_INT(value);
    mp_raise_ValueError("Touch pad error");
}
MP_DEFINE_CONST_FUN_OBJ_1(mtp_read_obj, mtp_read);


STATIC mp_obj_t mtp_deinit(mp_obj_t self_in) {
    machtouchpad_obj_t *self = self_in;
    if (! self->is_init) {
        mp_raise_ValueError("already de-init");
    }
    self->is_init = false;
    self->callback = mp_const_none;

    // lowlevel_touchpad_deinit: will turn everything 
    // off if no other objects are still alive.
    lowlevel_touchpad_deinit();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mtp_deinit_obj, mtp_deinit);


STATIC void mtp_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind ) {
    machtouchpad_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "TouchPad(T%u)", self->touchpad_id);
}

STATIC const mp_rom_map_elem_t mtp_locals_dict_table[] = {
    // class methods
    { MP_ROM_QSTR(MP_QSTR_filter_period),     MP_ROM_PTR(&mtp_filter_period_static_obj) },

    // instance methods
    { MP_ROM_QSTR(MP_QSTR_id),                MP_ROM_PTR(&mtp_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_config),            MP_ROM_PTR(&mtp_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),              MP_ROM_PTR(&mtp_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_threshold),         MP_ROM_PTR(&mtp_threshold_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),            MP_ROM_PTR(&mtp_deinit_obj) },
};

STATIC MP_DEFINE_CONST_DICT(mtp_locals_dict, mtp_locals_dict_table);

const mp_obj_type_t mach_touchpad_type = {
    { &mp_type_type },
    .name = MP_QSTR_TouchPad,
    .make_new = mtp_make_new,
    .locals_dict = (mp_obj_t)&mtp_locals_dict,
    .print = mtp_print,
};
