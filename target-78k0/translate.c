/*
 * 78K0 / RL78 translation
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
#include "qemu-common.h"
#include "disas/disas.h"
#include "tcg-op.h"
#include "qemu/log.h"

#include "helper.h"
#define GEN_HELPER 1
#include "helper.h"


/* Generate intermediate code in gen_opc_buf and gen_opparam_buf for
   basic block @tb. If @search_pc is %true, also generate PC
   information for each intermediate instruction. */
static inline void gen_intermediate_code_internal(RL78CPU *cpu,
                                                  TranslationBlock *tb,
                                                  bool search_pc)
{
}

void gen_intermediate_code(CPU78K0State *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(rl78_env_get_cpu(env), tb, false);
}

void gen_intermediate_code_pc(CPU78K0State *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(rl78_env_get_cpu(env), tb, true);
}

void restore_state_to_opc(CPU78K0State *env, TranslationBlock *tb, int pc_pos)
{
    env->pc = tcg_ctx.gen_opc_pc[pc_pos];
}

void rl78_cpu_dump_state(CPUState *cs, FILE *f, fprintf_function cpu_fprintf,
                         int flags)
{
}
