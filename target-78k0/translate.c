/*
 * 78K0 / RL78 translation
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
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "cpu.h"
#include "disas.h"
#include "tcg-op.h"
#include "qemu-log.h"

/* global register indexes */
static TCGv_ptr cpu_env;

static TCGv env_pc;
static TCGv_i32 cpu_sp;
#ifdef TARGET_RL78
static TCGv_i32 cpu_es;
static TCGv_i32 cpu_cs;
#endif

#include "helper.h"
#define GEN_HELPER 1
#include "helper.h"

#include "gen-icount.h"

//#define RL78_DEBUG_DISAS

#ifdef RL78_DEBUG_DISAS
#  define LOG_DISAS(...) printf(__VA_ARGS__)
#else
#  define LOG_DISAS(...) do { } while (0)
#endif

#define LOG_ASM(...) \
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) { \
        qemu_log(__VA_ARGS__); \
    }

typedef struct DisasContext DisasContext;
struct DisasContext {
    uint32_t pc;
    int is_jmp;
    struct TranslationBlock *tb;
};


void cpu_78k0_translate_init(void)
{
    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    env_pc = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, pc), "pc");
    cpu_sp = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, sp), "sp");
#ifdef TARGET_RL78
    cpu_es = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, es), "es");
    cpu_cs = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, cs), "cs");
#endif

#define GEN_HELPER 2
#include "helper.h"
}

static void disas_rl78_insn(DisasContext *s)
{
    uint8_t opc, opc2;
    int ins_len = 1;

    opc = ldub_code(s->pc);

    switch (opc) {
#ifdef TARGET_RL78
    case 0x41: /* MOV ES, #byte */
        ins_len = 2;
        {
            uint8_t data = ldub_code(s->pc + 1);
            LOG_ASM("MOV ES, #%02" PRIx8 "H\n", data);
            TCGv_i32 value = tcg_const_i32(data & 0xf);
            tcg_gen_mov_i32(cpu_es, value);
            tcg_temp_free_i32(value);
        }
        break;
#endif
    case 0xCB: /* MOVW sfrp,#word */
        ins_len = 4;
        {
            uint8_t sfrp = ldub_code(s->pc + 1) & ~0x1;
            uint16_t data = lduw_code(s->pc + 2);
            LOG_ASM("MOVW %02" PRIx8 "H, #%04" PRIx16 "H\n", sfrp, data);
            TCGv addr = tcg_const_tl(0xFFF00 | sfrp);
            TCGv val = tcg_const_tl(data);
            tcg_gen_qemu_st16(val, addr, 0);
            tcg_temp_free(addr);
            tcg_temp_free(val);
        }
        break;
    case 0xCF: /* MOV !addr16,#byte */
        ins_len = 4;
        {
            uint16_t addr16 = lduw_code(s->pc + 1);
            uint8_t data = ldub_code(s->pc + 3);
            LOG_ASM("MOV !%04" PRIx16 "H, #%02" PRIx8 "\n", addr16, data);
            TCGv addr = tcg_const_tl(0xF0000 | addr16);
            TCGv value = tcg_const_tl(data);
            tcg_gen_qemu_st8(value, addr, 0);
            tcg_temp_free(addr);
            tcg_temp_free(value);
        }
        break;

    case 0x61: /* 2nd MAP */
        opc2 = ldub_code(s->pc + 1);
        ins_len++;
        switch (opc2) {
        default:
            qemu_log("unimplemented 0x%" PRIx8 " opcode 0x%" PRIx8 "\n", opc, opc2);
            // TODO
            tcg_gen_movi_tl(env_pc, s->pc);
            s->is_jmp = DISAS_UPDATE;
            break;
        }
        break;
    default:
        qemu_log("unimplemented opcode 0x%" PRIx8 "\n", opc);
        // TODO
        tcg_gen_movi_tl(env_pc, s->pc);
        s->is_jmp = DISAS_UPDATE;
        break;
    }

    s->pc += ins_len;
}

/* generate intermediate code in gen_opc_buf and gen_opparam_buf for
   basic block 'tb'. If search_pc is TRUE, also generate PC
   information for each intermediate instruction. */
static inline void gen_intermediate_code_internal(CPUState *env,
                                                  TranslationBlock *tb,
                                                  bool search_pc)
{
    DisasContext dc;
    target_ulong pc_start;
    uint16_t *gen_opc_end;
    int num_insns, max_insns;
    uint32_t next_page_start;
    CPUBreakpoint *bp;
    int j, lj = -1;

    pc_start = tb->pc;
    pc_start &= 0x0fffff;
    dc.pc = pc_start;
    dc.is_jmp = DISAS_NEXT;
    dc.tb = tb;

    gen_opc_end = gen_opc_buf + OPC_MAX_SIZE;
    next_page_start = (pc_start & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;

    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0) {
        max_insns = CF_COUNT_MASK;
    }

    gen_icount_start();

    do {
        if (unlikely(!QTAILQ_EMPTY(&env->breakpoints))) {
            QTAILQ_FOREACH(bp, &env->breakpoints, entry) {
                if (bp->pc == dc.pc) {
                    // TODO
                    break;
                }
            }
        }
        if (search_pc) {
            j = gen_opc_ptr - gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j) {
                    gen_opc_instr_start[lj++] = 0;
                }
            }
            gen_opc_pc[lj] = dc.pc;
            gen_opc_instr_start[lj] = 1;
            gen_opc_icount[lj] = num_insns;
        }

        if (num_insns + 1 == max_insns && (tb->cflags & CF_LAST_IO)) {
            gen_io_start();
        }

        if (unlikely(qemu_loglevel_mask(CPU_LOG_TB_OP))) {
            tcg_gen_debug_insn_start(dc.pc);
        }

        disas_rl78_insn(&dc);

        num_insns++;
        if (env->singlestep_enabled) {
            // TODO
        }
    } while (dc.is_jmp == DISAS_NEXT
             && gen_opc_ptr < gen_opc_end && dc.pc < next_page_start && num_insns < max_insns
             && !env->singlestep_enabled && !singlestep);

    if (tb->cflags & CF_LAST_IO) {
        gen_io_end();
    }

    if (unlikely(env->singlestep_enabled)) {
        if (dc.is_jmp == DISAS_NEXT) {
            tcg_gen_movi_tl(env_pc, dc.pc);
        }
    } else {
        if (dc.is_jmp == DISAS_NEXT) {
            tcg_gen_movi_tl(env_pc, dc.pc);
        }
    }
    if (dc.is_jmp != DISAS_TB_JUMP) {
        tcg_gen_exit_tb(0);
    }
    gen_icount_end(tb, num_insns);
    *gen_opc_ptr = INDEX_op_end;
    if (search_pc) {
        j = gen_opc_ptr - gen_opc_buf;
        lj++;
        while (lj <= j) {
            gen_opc_instr_start[lj++] = 0;
        }
    } else {
        tb->size = dc.pc - pc_start;
        tb->icount = num_insns;
    }
}

void gen_intermediate_code(CPUState *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, false);
}

void gen_intermediate_code_pc(CPUState *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, true);
}

void cpu_dump_state(CPUState *env, FILE *f, fprintf_function cpu_fprintf,
                    int flags)
{
    cpu_fprintf(f, "PC %05" PRIx32 "\n", env->pc);
    cpu_fprintf(f, "SP  %04" PRIx32 "\n", env->sp);
#ifdef TARGET_RL78
    cpu_fprintf(f, "ES    %02" PRIx32 "\n", env->es);
    cpu_fprintf(f, "CS    %02" PRIx32 "\n", env->cs);
#endif
}

void restore_state_to_opc(CPUState *env, TranslationBlock *tb, int pc_pos)
{
    env->pc = gen_opc_pc[pc_pos];
}
