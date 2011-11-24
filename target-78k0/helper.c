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
#ifndef CONFIG_USER_ONLY
#include "hw/loader.h" /* for rom_ptr() */
#endif


void cpu_reset(CPUState *env)
{
    uint8_t *rom;
    uint16_t reset_vector;

    tlb_flush(env, 1);

    memset(env, 0, offsetof(CPUState, breakpoints));

    rom = rom_ptr(0x00000);
    if (rom == NULL) {
        reset_vector = 0;
    } else {
        reset_vector = lduw_p(rom);
    }
    env->pc = reset_vector;
}

CPUState *cpu_78k0_init(const char *cpu_model)
{
    static bool tcg_initialized = false;
    CPUState *env;

    env = g_malloc0(sizeof(CPUState));
    cpu_exec_init(env);
    if (!tcg_initialized) {
        tcg_initialized = true;
        cpu_78k0_translate_init();
    }

    env->cpu_model_str = cpu_model;
    cpu_reset(env);
    qemu_init_vcpu(env);
    printf("end init\n");
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

int cpu_78k0_handle_mmu_fault(CPUState *env, target_ulong address, int rw,
                              int mmu_idx)
{
    int prot;

    address &= TARGET_PAGE_MASK;
    prot = PAGE_BITS;
    tlb_set_page(env, address, address, prot, mmu_idx, TARGET_PAGE_SIZE);

    return 0;
}

/* Handle a CPU exception.  */
void do_interrupt(CPUState *env)
{
    env->interrupt_request |= CPU_INTERRUPT_EXITTB;
}

target_phys_addr_t cpu_get_phys_page_debug(CPUState *env, target_ulong addr)
{
    target_phys_addr_t phys_addr = addr;

    return phys_addr & TARGET_PAGE_MASK;
}

#endif

