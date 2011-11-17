/*
 * 78K0 / RL78 helpers
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

#include "cpu.h"
#include "exec/gdbstub.h"
#include "helper.h"
#include "qemu-common.h"


#if !defined(CONFIG_USER_ONLY)

#include "exec/softmmu_exec.h"

#define MMUSUFFIX _mmu

#define SHIFT 0
#include "exec/softmmu_template.h"

#define SHIFT 1
#include "exec/softmmu_template.h"

#define SHIFT 2
#include "exec/softmmu_template.h"

#define SHIFT 3
#include "exec/softmmu_template.h"

void tlb_fill(CPU78K0State *env, target_ulong addr, int is_write, int mmu_idx,
              uintptr_t retaddr)
{
    int ret;

    ret = cpu_78k0_handle_mmu_fault(env, addr, is_write, mmu_idx);
    if (unlikely(ret != 0)) {
        if (retaddr != (uintptr_t)0) {
            cpu_restore_state(env, retaddr);
        }
        cpu_loop_exit(env);
    }
}

int cpu_78k0_handle_mmu_fault(CPU78K0State *env, target_ulong address, int rw,
                              int mmu_idx)
{
    int prot;

    address &= TARGET_PAGE_MASK;
    prot = PAGE_BITS;
    tlb_set_page(env, address, address, prot, mmu_idx, TARGET_PAGE_SIZE);

    return 0;
}

/* Handle a CPU exception.  */
void rl78_cpu_do_interrupt(CPUState *cs)
{
    cs->interrupt_request |= CPU_INTERRUPT_EXITTB;
}

hwaddr rl78_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    hwaddr phys_addr = addr;

    return phys_addr & TARGET_PAGE_MASK;
}

#endif
