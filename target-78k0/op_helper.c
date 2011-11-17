/*
 * 78K0 / RL78 helper routines
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
#include "cpu.h"
#include "dyngen-exec.h"
#include "helper.h"


#if !defined(CONFIG_USER_ONLY)

#include "softmmu_exec.h"

#define MMUSUFFIX _mmu

#define SHIFT 0
#include "softmmu_template.h"

#define SHIFT 1
#include "softmmu_template.h"

#define SHIFT 2
#include "softmmu_template.h"

#define SHIFT 3
#include "softmmu_template.h"

void tlb_fill(CPUState *env1, target_ulong addr, int is_write, int mmu_idx,
              void *retaddr)
{
    CPUState *saved_env;
    unsigned long pc;
    TranslationBlock *tb;
    int ret;

    saved_env = env;
    env = env1;

    ret = cpu_78k0_handle_mmu_fault(env, addr, is_write, mmu_idx);
    if (unlikely(ret != 0)) {
        if (retaddr != NULL) {
            pc = (unsigned long)retaddr;
            tb = tb_find_pc(pc);
            if (tb != NULL) {
                cpu_restore_state(tb, env, pc);
            }
        }
        cpu_loop_exit(env);
    }

    env = saved_env;
}

#endif

