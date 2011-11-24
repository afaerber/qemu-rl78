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

/* global register indexes */
static TCGv_ptr cpu_env;

static TCGv env_pc;

#include "helper.h"
#define GEN_HELPER 1
#include "helper.h"

#include "exec/gen-icount.h"

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


void cpu_rl78_translate_init(void)
{
    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    env_pc = tcg_global_mem_new(TCG_AREG0, offsetof(CPU78K0State, pc), "pc");

#define GEN_HELPER 2
#include "helper.h"
}

typedef int (*OpcodeHandler)(RL78CPU *cpu, uint8_t opcode, DisasContext *s);

/* MOV !addr16,#byte */
static int rl78_disas_mov_addr16_byte(RL78CPU *cpu, uint8_t opcode, DisasContext *s)
{
    uint16_t addr16 = cpu_lduw_code(&cpu->env, s->pc + 1);
    uint8_t data = cpu_ldub_code(&cpu->env, s->pc + 3);
    TCGv addr = tcg_const_tl(0xF0000 | addr16);
    TCGv value = tcg_const_tl(data);

    LOG_ASM("MOV !%04" PRIx16 "H, #%02" PRIx8 "\n", addr16, data);
    tcg_gen_qemu_st8(value, addr, 0);
    tcg_temp_free(addr);
    tcg_temp_free(value);
    return 4;
}

static const OpcodeHandler rl78_1st_map[256] = {
    [0xCF] = rl78_disas_mov_addr16_byte,
};

static void disas_rl78_insn(RL78CPU *cpu, DisasContext *s)
{
    uint8_t opc;
    int ins_len;

    opc = cpu_ldub_code(&cpu->env, s->pc);

    if (likely(rl78_1st_map[opc] != NULL)) {
        ins_len = rl78_1st_map[opc](cpu, opc, s);
    } else {
        qemu_log("unimplemented opcode 0x%" PRIx8 "\n", opc);
        // TODO
        tcg_gen_movi_tl(env_pc, s->pc);
        s->is_jmp = DISAS_UPDATE;
        ins_len = 1;
    }

    s->pc += ins_len;
}

/* Generate intermediate code in gen_opc_buf and gen_opparam_buf for
   basic block @tb. If @search_pc is %true, also generate PC
   information for each intermediate instruction. */
static inline void gen_intermediate_code_internal(RL78CPU *cpu,
                                                  TranslationBlock *tb,
                                                  bool search_pc)
{
    CPUState *cs = CPU(cpu);
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

    gen_opc_end = tcg_ctx.gen_opc_buf + OPC_MAX_SIZE;
    next_page_start = (pc_start & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;

    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0) {
        max_insns = CF_COUNT_MASK;
    }

    gen_tb_start();

    do {
        if (unlikely(!QTAILQ_EMPTY(&cpu->env.breakpoints))) {
            QTAILQ_FOREACH(bp, &cpu->env.breakpoints, entry) {
                if (bp->pc == dc.pc) {
                    // TODO
                    break;
                }
            }
        }
        if (search_pc) {
            j = tcg_ctx.gen_opc_ptr - tcg_ctx.gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j) {
                    tcg_ctx.gen_opc_instr_start[lj++] = 0;
                }
            }
            tcg_ctx.gen_opc_pc[lj] = dc.pc;
            tcg_ctx.gen_opc_instr_start[lj] = 1;
            tcg_ctx.gen_opc_icount[lj] = num_insns;
        }

        if (num_insns + 1 == max_insns && (tb->cflags & CF_LAST_IO)) {
            gen_io_start();
        }

        if (unlikely(qemu_loglevel_mask(CPU_LOG_TB_OP))) {
            tcg_gen_debug_insn_start(dc.pc);
        }

        disas_rl78_insn(cpu, &dc);

        num_insns++;
        if (cs->singlestep_enabled) {
            // TODO
        }
    } while (dc.is_jmp == DISAS_NEXT
             && tcg_ctx.gen_opc_ptr < gen_opc_end
             && dc.pc < next_page_start && num_insns < max_insns
             && !cs->singlestep_enabled && !singlestep);

    if (tb->cflags & CF_LAST_IO) {
        gen_io_end();
    }

    if (unlikely(cs->singlestep_enabled)) {
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
    gen_tb_end(tb, num_insns);
    *tcg_ctx.gen_opc_ptr = INDEX_op_end;
    if (search_pc) {
        j = tcg_ctx.gen_opc_ptr - tcg_ctx.gen_opc_buf;
        lj++;
        while (lj <= j) {
            tcg_ctx.gen_opc_instr_start[lj++] = 0;
        }
    } else {
        tb->size = dc.pc - pc_start;
        tb->icount = num_insns;
    }
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
    RL78CPU *cpu = RL78_CPU(cs);

    cpu_fprintf(f, "PC %05" PRIx32 "\n\n", cpu->env.pc);
}
