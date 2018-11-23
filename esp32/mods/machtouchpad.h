/*
 * machtouchpad
 * Provides Touchpad support for ESP32, through the machine.TouchPad module.
 *
 * Copyright (c) 2018 Pat Deegan, Psychogenic Technologies INC 
 * https://psychogenic.com
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 *
 *
 * Sample usage on the uPython side:
 * >>> def hullo(tp):
 * ...     print("Hullooo: %s" % str(tp))
 * ...
 * >>> t6 = machine.TouchPad('P23', trigger=hullo)
 * >>> t8 = machine.TouchPad('P20', trigger=hullo)
 * >>> t6.read()
 * 1149
 * >>> t8.read()
 * 1034
 * >>> machine.TouchPad.filter_period(200)
 * >>> t6.threshold(600)
 * >>> t8.threshold(500)
 * >>> Hullooo: TouchPad(T8)
 * Hullooo: TouchPad(T8)
 * Hullooo: TouchPad(T6)
 * Hullooo: TouchPad(T8)
 * etc 
 */


#ifndef MACHTOUCHPAD_H_
#define MACHTOUCHPAD_H_

extern const mp_obj_type_t mach_touchpad_type;
typedef struct _mach_touchpad_obj_t mach_touchpad_obj_t;

#endif  // MACHTOUCHPAD_H_
