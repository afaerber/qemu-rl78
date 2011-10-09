/*
 * 78K0 / RL78 helpers
 *
 * Copyright (c) 2011 Andreas Färber
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cpu.h"
#include "gdbstub.h"
#include "helper.h"
#include "qemu-common.h"


void cpu_reset(CPUState *env)
{
}

CPUState *cpu_78k0_init(const char *cpu_model)
{
    CPUState *env;

    env = g_malloc0(sizeof(CPUState));
    cpu_exec_init(env);

    env->cpu_model_str = cpu_model;
    cpu_reset(env);
    qemu_init_vcpu(env);
    return env;
}

void cpu_78k0_list(FILE *f, fprintf_function cpu_fprintf)
{
}

void cpu_78k0_close(CPUState *env)
{
    g_free(env);
}

#if !defined(CONFIG_USER_ONLY)

/* Handle a CPU exception.  */
void do_interrupt(CPUState *env)
{
    env->interrupt_request |= CPU_INTERRUPT_EXITTB;
}

target_phys_addr_t cpu_get_phys_page_debug(CPUState *env, target_ulong addr)
{
    uint32_t phys_addr;

    phys_addr = addr;

    return phys_addr;
}

#endif

