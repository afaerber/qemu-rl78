/*
 * 78K0 / RL78 CPU
 *
 * Copyright (c) 2011-2013 Andreas Färber
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "cpu.h"
#include "qemu-common.h"
#include "error.h"
#ifndef CONFIG_USER_ONLY
#include "hw/loader.h" /* for rom_ptr() */
#endif


/* CPUClass::reset() */
static void rl78_cpu_reset(CPUState *cs)
{
    RL78CPU *cpu = RL78_CPU(cs);
    RL78CPUClass *rcc = RL78_CPU_GET_CLASS(cs);
    uint8_t *rom;
    uint16_t reset_vector;

    rcc->parent_reset(cs);

    tlb_flush(&cpu->env, 1);

    memset(&cpu->env, 0, offsetof(CPU78K0State, breakpoints));

    rom = rom_ptr(0x00000);
    if (rom == NULL) {
        reset_vector = 0;
    } else {
        reset_vector = lduw_p(rom);
    }
    cpu->env.pc = reset_vector;
}

static void rl78_cpu_set_pc(CPUState *cs, vaddr value)
{
    RL78CPU *cpu = RL78_CPU(cs);

    cpu->env.pc = value;
}

/* ObjectClass::realize() */
static void rl78_cpu_realize(DeviceState *dev, Error **errp)
{
    RL78CPUClass *rcc = RL78_CPU_GET_CLASS(dev);

    cpu_reset(CPU(dev));

    rcc->parent_realize(dev, errp);
}

RL78CPU *cpu_rl78_init(const char *cpu_model)
{
    RL78CPU *cpu;
    Error *err = NULL;

    cpu = RL78_CPU(object_new(TYPE_RL78_CPU));

    object_property_set_bool(OBJECT(cpu), true, "realized", &err);
    if (err != NULL) {
        fprintf(stderr, "%s\n", error_get_pretty(err));
        error_free(err);
        object_unref(OBJECT(cpu));
        return NULL;
    }
    return cpu;
}

void cpu_78k0_list(FILE *f, fprintf_function cpu_fprintf)
{
}

static void rl78_cpu_initfn(Object *obj)
{
    RL78CPU *cpu = RL78_CPU(obj);
    static bool tcg_initialized = false;

    cpu_exec_init(&cpu->env);

    if (tcg_enabled() && !tcg_initialized) {
        tcg_initialized = true;
        cpu_rl78_translate_init();
    }
}

static void rl78_cpu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    RL78CPUClass *rcc = RL78_CPU_CLASS(oc);

    rcc->parent_reset = cc->reset;
    cc->reset = rl78_cpu_reset;

    rcc->parent_realize = dc->realize;
    dc->realize = rl78_cpu_realize;

    cc->set_pc = rl78_cpu_set_pc;
    cc->get_phys_page_debug = rl78_cpu_get_phys_page_debug;
}

static const TypeInfo rl78_cpu_type_info = {
    .name = TYPE_RL78_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(RL78CPU),
    .instance_init = rl78_cpu_initfn,
    .abstract = false,
    .class_size = sizeof(RL78CPUClass),
    .class_init = rl78_cpu_class_init,
};

static void rl78_cpu_register_types(void)
{
    type_register_static(&rl78_cpu_type_info);
}

type_init(rl78_cpu_register_types)
