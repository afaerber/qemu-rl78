/*
 * 78K0 / RL78 virtual CPU header
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
#ifndef QEMU_CPU_78K0_H
#define QEMU_CPU_78K0_H

#define TARGET_LONG_BITS 32

#define CPUState struct CPU78K0State

#include "config.h"
#include "qemu-common.h"
#include "cpu-defs.h"

#include "softfloat.h"

#define ELF_MACHINE EM_RL78

#define NB_MMU_MODES 1

typedef struct CPU78K0State {
    target_ulong pc; /* 20/16 bits */
    uint32_t psw; /* 8 bits */
    uint32_t sp; /* 16 bits */

    CPU_COMMON

} CPU78K0State;

CPU78K0State *cpu_78k0_init(const char *cpu_model);
void cpu_78k0_translate_init(void);
int cpu_78k0_exec(CPU78K0State *s);
void cpu_78k0_close(CPU78K0State *s);
void do_interrupt(CPU78K0State *);

void cpu_78k0_list(FILE *f, fprintf_function cpu_fprintf);

int cpu_78k0_handle_mmu_fault(CPU78K0State *env, target_ulong address, int rw,
                              int mmu_idx);


#define TARGET_PAGE_BITS 12

#if defined(TARGET_RL78)
#define TARGET_PHYS_ADDR_SPACE_BITS 20
#define TARGET_VIRT_ADDR_SPACE_BITS 20
#else
#define TARGET_PHYS_ADDR_SPACE_BITS 16
#define TARGET_VIRT_ADDR_SPACE_BITS 16
#endif

#define cpu_init cpu_78k0_init
#define cpu_exec cpu_78k0_exec
#define cpu_list cpu_78k0_list
#define cpu_handle_mmu_fault cpu_78k0_handle_mmu_fault

static inline int cpu_mmu_index(CPUState *env)
{
    return 0;
}

#include "cpu-all.h"

static inline target_ulong cpu_get_pc(CPUState *env)
{
    return env->pc;
}

static inline void cpu_get_tb_cpu_state(CPUState *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = 0;
}

static inline bool cpu_has_work(CPUState *env)
{
    return env->interrupt_request &
        (CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
}

#include "exec-all.h"

static inline void cpu_pc_from_tb(CPUState *env, TranslationBlock *tb)
{
    env->pc = tb->pc;
}


#endif

