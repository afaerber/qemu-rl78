/*
 * 78K0 / RL78 virtual CPU header
 *
 * Copyright (c) 2011-2012 Andreas Färber
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
#ifndef QEMU_CPU_RL78_H
#define QEMU_CPU_RL78_H

#define TARGET_LONG_BITS 32

#define CPUArchState struct CPU78K0State

#include "config.h"
#include "qemu-common.h"
#include "exec/cpu-defs.h"
#include "qom/cpu.h"
#include "error.h"

#include "fpu/softfloat.h"

#ifdef TARGET_RL78
#define TYPE_RL78_CPU "rl78-cpu"
#else
#define TYPE_RL78_CPU "78k0-cpu"
#endif

#define RL78_CPU(obj) OBJECT_CHECK(RL78CPU, (obj), TYPE_RL78_CPU)
#define RL78_CPU_CLASS(class) \
    OBJECT_CLASS_CHECK(RL78CPUClass, (class), TYPE_RL78_CPU)
#define RL78_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(RL78CPUClass, (obj), TYPE_RL78_CPU)

typedef struct RL78CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);
} RL78CPUClass;

#define ELF_MACHINE EM_RL78

#define NB_MMU_MODES 1

typedef struct CPU78K0State {
    target_ulong pc; /* 20/16 bits */
    uint32_t psw; /* 8 bits */
    uint32_t sp; /* 16 bits */

    CPU_COMMON

} CPU78K0State;

typedef struct RL78CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPU78K0State env;
} RL78CPU;

static inline RL78CPU *rl78_env_get_cpu(CPU78K0State *env)
{
    return container_of(env, RL78CPU, env);
}

#define ENV_GET_CPU(e) CPU(rl78_env_get_cpu(e))
#define ENV_OFFSET offsetof(RL78CPU, env)

RL78CPU *cpu_rl78_init(const char *cpu_model);

static inline CPU78K0State *cpu_init(const char *cpu_model)
{
    RL78CPU *cpu = cpu_rl78_init(cpu_model);
    if (cpu == NULL) {
        return NULL;
    }
    return &cpu->env;
}

void cpu_rl78_translate_init(void);

int cpu_78k0_exec(CPU78K0State *env);
void rl78_cpu_do_interrupt(CPUState *cpu);
void rl78_cpu_dump_state(CPUState *cpu, FILE *f, fprintf_function cpu_fprintf,
                         int flags);
hwaddr rl78_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);

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

#define cpu_exec cpu_78k0_exec
#define cpu_list cpu_78k0_list
#define cpu_handle_mmu_fault cpu_78k0_handle_mmu_fault

static inline int cpu_mmu_index(CPU78K0State *env)
{
    return 0;
}

#include "exec/cpu-all.h"

static inline void cpu_get_tb_cpu_state(CPU78K0State *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = 0;
}

static inline bool cpu_has_work(CPUState *cs)
{
    return cs->interrupt_request &
        (CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
}

#include "exec/exec-all.h"


#endif
