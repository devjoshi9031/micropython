/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Damien P. George
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

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "py/objarray.h"
#include "extmod/modmachine.h"

#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#define DEFAULT_I2C_FREQ (400000)
#define DEFAULT_I2C_TIMEOUT (50000)

#ifdef MICROPY_HW_I2C_NO_DEFAULT_PINS

// With no default I2C, need to require the pin args.
#define MICROPY_HW_I2C0_SCL (0)
#define MICROPY_HW_I2C0_SDA (0)
#define MICROPY_HW_I2C1_SCL (0)
#define MICROPY_HW_I2C1_SDA (0)
#define MICROPY_I2C_PINS_ARG_OPTS MP_ARG_REQUIRED

#else

// Most boards do not require pin args.
#define MICROPY_I2C_PINS_ARG_OPTS 0

#ifndef MICROPY_HW_I2C0_SCL
#if PICO_DEFAULT_I2C == 0
#define MICROPY_HW_I2C0_SCL (PICO_DEFAULT_I2C_SCL_PIN)
#define MICROPY_HW_I2C0_SDA (PICO_DEFAULT_I2C_SDA_PIN)
#else
#define MICROPY_HW_I2C0_SCL (9)
#define MICROPY_HW_I2C0_SDA (8)
#endif
#endif

#ifndef MICROPY_HW_I2C1_SCL
#if PICO_DEFAULT_I2C == 1
#define MICROPY_HW_I2C1_SCL (PICO_DEFAULT_I2C_SCL_PIN)
#define MICROPY_HW_I2C1_SDA (PICO_DEFAULT_I2C_SDA_PIN)
#else
#define MICROPY_HW_I2C1_SCL (7)
#define MICROPY_HW_I2C1_SDA (6)
#endif
#endif
#endif

// SDA/SCL on even/odd pins, I2C0/I2C1 on even/odd pairs of pins.
#define IS_VALID_SCL(i2c, pin) (((pin) & 1) == 1 && (((pin) & 2) >> 1) == (i2c))
#define IS_VALID_SDA(i2c, pin) (((pin) & 1) == 0 && (((pin) & 2) >> 1) == (i2c))
#define MICROPY_I2C_DEFAULT_SLAVE_ADDR  (0x0B)
#define MICROPY_I2C_MAX_SLAVE_ADDR      (0x7F)

typedef struct _slave_i2c_obj_t {
    uint8_t     mode;   /*!< slave_mode of type slave_mode_t */
    bool     reg_addr_written;   /*!< flag to store if the register index is written or not. */
    uint8_t     addr;   /*!< slave addr */
    uint        mem_buf_size;
    uint8_t*    mem_buf;
    uint16_t    reg_addr;  /*!< var. to store the register index that master wants to write/read to/from.*/
}slave_i2c_obj_t;

typedef struct _machine_i2c_obj_t {
    mp_obj_base_t base;
    i2c_inst_t *const i2c_inst;
    uint8_t i2c_id;
    uint8_t scl;
    uint8_t sda;
    uint32_t freq;
    uint32_t timeout;
    uint8_t i2c_mode;
    slave_i2c_obj_t slave_cxt;
} machine_i2c_obj_t;

typedef enum
{
    MASTER,
    SLAVE,
} i2c_mode_t;

typedef enum
{
    SLAVE_MODE_88,
    SLAVE_MODE_816,
    SLAVE_MODE_1616,
    SLAVE_MODE_INVALID,
} slave_mode_t;

static machine_i2c_obj_t machine_i2c_obj[] = {
    {{&machine_i2c_type}, i2c0, 0, MICROPY_HW_I2C0_SCL, MICROPY_HW_I2C0_SDA, 0},
    {{&machine_i2c_type}, i2c1, 1, MICROPY_HW_I2C1_SCL, MICROPY_HW_I2C1_SDA, 0},
};

void machine_i2c_slave_handler(i2c_inst_t * i2c, i2c_slave_event_t event)
{
    uint i2c_id = i2c_hw_index(i2c);
    machine_i2c_obj_t* self = &machine_i2c_obj[i2c_id];
    switch(event)
    {
        /**
         * @brief I2C_SLAVE_REQUEST will be passed in when the MASTER request for data. 
         */
        case I2C_SLAVE_REQUEST:
            if(self->slave_cxt.reg_addr >= self->slave_cxt.mem_buf_size)
            {
                /** Send last byte if the register index request is out of bound of the reg map.
                 * TODO: Need to check if we can NACK this request as it is not valid.
                */
                self->slave_cxt.reg_addr = self->slave_cxt.mem_buf_size-1;
            }
            else
            {
                i2c_write_byte_raw(i2c,self->slave_cxt.mem_buf[self->slave_cxt.reg_addr]);
                self->slave_cxt.reg_addr++;
            }
            break;

        /**
         * @brief Everytime this callback function is called, we will come to this case as the first bytes will be the register index master wants to read or write.
         */
        case I2C_SLAVE_RECEIVE:
            if(!self->slave_cxt.reg_addr_written)
            {
                /** If it is the first time we are entering this case, the first 1 or 2 bytes we will always treat it as register index. */
                uint8_t reg_addr = i2c_read_byte_raw(i2c);
                self->slave_cxt.reg_addr = reg_addr;
                self->slave_cxt.reg_addr_written = true;
            }
            else
            {
                self->slave_cxt.mem_buf[self->slave_cxt.reg_addr] = i2c_read_byte_raw(i2c);
                self->slave_cxt.reg_addr++;
            }
            break;

        case I2C_SLAVE_FINISH:
            self->slave_cxt.reg_addr_written=false;
            break;
    }
}

static void machine_i2c_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_i2c_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "I2C(%u, freq=%u, scl=%u, sda=%u, timeout=%u)",
        self->i2c_id, self->freq, self->scl, self->sda, self->timeout);
}

mp_obj_t machine_i2c_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_id, ARG_freq, ARG_mode, ARG_scl, ARG_sda, ARG_timeout,  ARG_slave_mode, ARG_slave_addr, ARG_slave_mem_buf };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_freq, MP_ARG_INT, {.u_int = DEFAULT_I2C_FREQ} },
        { MP_QSTR_mode, MP_ARG_INT, {.u_int = MASTER} },
        { MP_QSTR_scl, MICROPY_I2C_PINS_ARG_OPTS | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_sda, MICROPY_I2C_PINS_ARG_OPTS | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_I2C_TIMEOUT} },
        { MP_QSTR_slave_mode, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = SLAVE_MODE_88} },
        { MP_QSTR_slave_addr, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = MICROPY_I2C_DEFAULT_SLAVE_ADDR} },
        { MP_QSTR_slave_mem_buf, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Get I2C bus.
    int i2c_id = mp_obj_get_int(args[ARG_id].u_obj);
    if (i2c_id < 0 || i2c_id >= MP_ARRAY_SIZE(machine_i2c_obj)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("I2C(%d) doesn't exist"), i2c_id);
    }

    // Get static peripheral object.
    machine_i2c_obj_t *self = (machine_i2c_obj_t *)&machine_i2c_obj[i2c_id];

    // Set SCL/SDA pins if configured.
    if (args[ARG_scl].u_obj != mp_const_none) {
        int scl = mp_hal_get_pin_obj(args[ARG_scl].u_obj);
        if (!IS_VALID_SCL(self->i2c_id, scl)) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad SCL pin"));
        }
        self->scl = scl;
    }
    if (args[ARG_sda].u_obj != mp_const_none) {
        int sda = mp_hal_get_pin_obj(args[ARG_sda].u_obj);
        if (!IS_VALID_SDA(self->i2c_id, sda)) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad SDA pin"));
        }
        self->sda = sda;
    }

    if(args[ARG_mode].u_int == SLAVE)
    {
        self->i2c_mode = args[ARG_mode].u_int;
        if(args[ARG_slave_mode].u_int >= SLAVE_MODE_INVALID)
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid Slave mode. Acceptable values are [I2C.SLAVE_MODE_88, I2C.SLAVE_MODE_816, I2C.SLAVE_MODE_1616].\n"));
        
        if(args[ARG_slave_addr].u_int > MICROPY_I2C_MAX_SLAVE_ADDR)
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid Slave Addr. Slave Addr has be between [0 - 127].\n"));
        
        if(args[ARG_slave_mem_buf].u_obj == MP_OBJ_NULL)
            mp_raise_ValueError(MP_ERROR_TEXT("argument slave_mem_buf not provided. Please provide a buffer to use as register map for I2C slave.\n"));

        self->slave_cxt.mode = args[ARG_slave_mode].u_int;
        self->slave_cxt.addr = args[ARG_slave_addr].u_int;
        mp_obj_array_t *bytearray = (mp_obj_array_t*) MP_OBJ_TO_PTR(args[ARG_slave_mem_buf].u_obj);
        self->slave_cxt.mem_buf = (uint8_t*)(bytearray->items);
        self->slave_cxt.mem_buf_size = (uint)(bytearray->len);
    }

    // Initialise the I2C peripheral if any arguments given, or it was not initialised previously.
    if (n_args > 1 || n_kw > 0 || self->freq == 0) {
        self->freq = args[ARG_freq].u_int;
        if(self->i2c_mode == SLAVE)
            i2c_slave_init(self->i2c_inst, self->slave_cxt.addr, &machine_i2c_slave_handler);
        else
            i2c_init(self->i2c_inst, self->freq);
        self->freq = i2c_set_baudrate(self->i2c_inst, self->freq);
        self->timeout = args[ARG_timeout].u_int;
        gpio_set_function(self->scl, GPIO_FUNC_I2C);
        gpio_set_function(self->sda, GPIO_FUNC_I2C);
        gpio_set_pulls(self->scl, true, 0);
        gpio_set_pulls(self->sda, true, 0);
    }

    return MP_OBJ_FROM_PTR(self);
}

static int machine_i2c_transfer_single(mp_obj_base_t *self_in, uint16_t addr, size_t len, uint8_t *buf, unsigned int flags) {
    machine_i2c_obj_t *self = (machine_i2c_obj_t *)self_in;
    int ret;
    bool nostop = !(flags & MP_MACHINE_I2C_FLAG_STOP);
    if (flags & MP_MACHINE_I2C_FLAG_READ) {
        ret = i2c_read_timeout_us(self->i2c_inst, addr, buf, len, nostop, self->timeout);
    } else {
        if (len == 0) {
            // Workaround issue with hardware I2C not accepting zero-length writes.
            mp_machine_soft_i2c_obj_t soft_i2c = {
                .base = { &mp_machine_soft_i2c_type },
                .us_delay = 500000 / self->freq + 1,
                .us_timeout = self->timeout,
                .scl = self->scl,
                .sda = self->sda,
            };
            mp_machine_i2c_buf_t bufs = {
                .len = len,
                .buf = buf,
            };
            mp_hal_pin_open_drain(self->scl);
            mp_hal_pin_open_drain(self->sda);
            ret = mp_machine_soft_i2c_transfer(&soft_i2c.base, addr, 1, &bufs, flags);
            gpio_set_function(self->scl, GPIO_FUNC_I2C);
            gpio_set_function(self->sda, GPIO_FUNC_I2C);
            return ret;
        } else {
            ret = i2c_write_timeout_us(self->i2c_inst, addr, buf, len, nostop, self->timeout);
        }
    }
    if (ret < 0) {
        if (ret == PICO_ERROR_TIMEOUT) {
            return -MP_ETIMEDOUT;
        } else {
            return -MP_EIO;
        }
    } else {
        return ret;
    }
}

static const mp_machine_i2c_p_t machine_i2c_p = {
    .transfer = mp_machine_i2c_transfer_adaptor,
    .transfer_single = machine_i2c_transfer_single,
};

MP_DEFINE_CONST_OBJ_TYPE(
    machine_i2c_type,
    MP_QSTR_I2C,
    MP_TYPE_FLAG_NONE,
    make_new, machine_i2c_make_new,
    print, machine_i2c_print,
    protocol, &machine_i2c_p,
    locals_dict, &mp_machine_i2c_locals_dict
    );
