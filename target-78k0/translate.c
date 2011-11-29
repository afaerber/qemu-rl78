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
static TCGv_i32 cpu_psw;
static TCGv_i32 cpu_sp;
#ifdef TARGET_RL78
static TCGv_i32 cpu_es;
static TCGv_i32 cpu_cs;
#endif

static const char* gpr8_names[] = { "X", "A", "C", "B", "E", "D", "L", "H" };
static const char* gpr16_names[] = { "AX", "BC", "DE", "HL" };

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
    cpu_psw = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, psw), "psw");
    cpu_sp = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, sp), "sp");
#ifdef TARGET_RL78
    cpu_es = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, es), "es");
    cpu_cs = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, cs), "cs");
#endif

#define GEN_HELPER 2
#include "helper.h"
}

enum PSWBitShifts {
    PSW_CY_SHIFT   = 0,
    PSW_ISP0_SHIFT = 1,
    PSW_ISP1_SHIFT = 2,
    PSW_RBS0_SHIFT = 3,
    PSW_AC_SHIFT   = 4,
    PSW_RBS1_SHIFT = 5,
    PSW_Z_SHIFT    = 6,
    PSW_IE_SHIFT   = 7,
};

static void gen_get_bank(TCGv_i32 ret)
{
    TCGv_i32 rbs0, rbs1;
    rbs0 = tcg_temp_new_i32();
    tcg_gen_shri_i32(rbs0, cpu_psw, PSW_RBS0_SHIFT);
    tcg_gen_andi_i32(rbs0, rbs0, 0x1);
    rbs1 = tcg_temp_new_i32();
    tcg_gen_shri_i32(rbs1, cpu_psw, PSW_RBS1_SHIFT - 1);
    tcg_gen_andi_i32(rbs1, rbs1, 0x2);
    tcg_gen_or_i32(ret, rbs0, rbs1);
    tcg_temp_free_i32(rbs0);
    tcg_temp_free_i32(rbs1);
}

#define GPR_START 0xffee0

typedef enum {
    REG_X = 0,
    REG_A,
    REG_C,
    REG_B,
    REG_E,
    REG_D,
    REG_L,
    REG_H,
} RL78GPR8;

typedef enum {
    RP_AX = 0,
    RP_BC = 2,
    RP_DE = 4,
    RP_HL = 6,
} RL78GPR16;

static void gen_get_bank_addr(TCGv ret)
{
    TCGv tmp;
    TCGv_i32 bank = tcg_temp_new_i32();
    gen_get_bank(bank);
    tcg_gen_muli_i32(bank, bank, 8);
    tmp = tcg_temp_new();
    tcg_gen_extu_i32_tl(tmp, bank);
    tcg_temp_free_i32(bank);
    tcg_gen_movi_tl(ret, GPR_START + 3 * 8);
    tcg_gen_sub_tl(ret, ret, tmp);
    tcg_temp_free(tmp);
}

static void gen_get_gpr_addr16(TCGv ret, RL78GPR16 reg)
{
    gen_get_bank_addr(ret);
    tcg_gen_addi_tl(ret, ret, reg);
}

static void gen_gpr_ld16u(TCGv ret, RL78GPR16 reg)
{
    TCGv addr = tcg_temp_new();
    gen_get_gpr_addr16(addr, reg);
    tcg_gen_qemu_ld16u(ret, addr, 0);
    tcg_temp_free(addr);
}

static void gen_gpr_st16(RL78GPR16 reg, TCGv val)
{
    TCGv addr = tcg_temp_new();
    gen_get_gpr_addr16(addr, reg);
    tcg_gen_qemu_st16(val, addr, 0);
    tcg_temp_free(addr);
}

static void gen_movw_rp_word(RL78GPR16 regpair, uint16_t data)
{
    TCGv value;
    LOG_ASM("MOVW %s, #%04" PRIx16 "H\n", gpr16_names[regpair / 2], data);
    value = tcg_const_tl(data);
    gen_gpr_st16(regpair, value);
    tcg_temp_free(value);
}

static void gen_select_bank(unsigned int bank)
{
    TCGv_i32 tmp, val;
    LOG_ASM("SEL RB%u\n", bank);
    assert(bank <= 3);
    tmp = tcg_temp_new_i32();
    val = tcg_const_i32(bank & 0x1);
    tcg_gen_deposit_i32(tmp, cpu_psw, val, PSW_RBS0_SHIFT, 1);
    tcg_gen_movi_i32(val, (bank >> 1) & 0x1);
    tcg_gen_deposit_i32(tmp, tmp, val, PSW_RBS1_SHIFT, 1);
    tcg_gen_mov_i32(cpu_psw, tmp);
    tcg_temp_free_i32(val);
    tcg_temp_free_i32(tmp);
}

static void disas_rl78_insn(DisasContext *s)
{
    uint8_t opc, opc2;
    int ins_len = 1;

    opc = ldub_code(s->pc);

    switch (opc) {
    case 0x30: /* MOVW AX, #word */
    case 0x32: /* MOVW BC, #word */
    case 0x34: /* MOVW DE, #word */
    case 0x36: /* MOVW HL, #word */
        ins_len = 3;
        gen_movw_rp_word(opc & 0xf, lduw_code(s->pc + 1));
        break;
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
        case 0xcf: /* SEL RB0 */
        case 0xdf: /* SEL RB1 */
        case 0xef: /* SEL RB2 */
        case 0xff: /* SEL RB3 */
            gen_select_bank((opc2 >> 4) - 0xc);
            break;
        default:
            qemu_log("unimplemented 0x%" PRIx8 " opcode 0x%" PRIx8 "\n", opc, opc2);
            // TODO
            tcg_gen_movi_tl(env_pc, s->pc);
            s->is_jmp = DISAS_UPDATE;
            break;
        }
        break;
    case 0x31: /* 4th MAP */
        opc2 = ldub_code(s->pc + 1);
        ins_len++;
        switch (opc2) {
        case 0x1e: /* SHRW AX, 1 */
        case 0x2e: /* SHRW AX, 2 */
        case 0x3e: /* SHRW AX, 3 */
        case 0x4e: /* SHRW AX, 4 */
        case 0x5e: /* SHRW AX, 5 */
        case 0x6e: /* SHRW AX, 6 */
        case 0x7e: /* SHRW AX, 7 */
        case 0x8e: /* SHRW AX, 8 */
        case 0x9e: /* SHRW AX, 9 */
        case 0xae: /* SHRW AX, 10 */
        case 0xbe: /* SHRW AX, 11 */
        case 0xce: /* SHRW AX, 12 */
        case 0xde: /* SHRW AX, 13 */
        case 0xee: /* SHRW AX, 14 */
        case 0xfe: /* SHRW AX, 15 */
            {
                unsigned int cnt = opc2 >> 4;
                TCGv tmp;
                TCGv_i32 cy;
                LOG_ASM("SHRW AX, %u\n", cnt);
                tmp = tcg_temp_new();
                gen_gpr_ld16u(tmp, RP_AX);

                cy = tcg_temp_new_i32();
                tcg_gen_trunc_tl_i32(cy, tmp);
                tcg_gen_shri_i32(cy, cy, cnt - 1);
                tcg_gen_deposit_i32(cpu_psw, cpu_psw, cy, PSW_CY_SHIFT, 1);
                tcg_temp_free_i32(cy);

                tcg_gen_shri_tl(tmp, tmp, cnt);
                gen_gpr_st16(RP_AX, tmp);
                tcg_temp_free(tmp);
            }
            break;
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
    int i, j;

    cpu_fprintf(f, "PC %05" PRIx32 "\n", env->pc);
    cpu_fprintf(f, "SP  %04" PRIx32 "\n", env->sp);
    cpu_fprintf(f, "PSW   %02" PRIx32 "\n", env->psw);
#ifdef TARGET_RL78
    cpu_fprintf(f, "ES    %02" PRIx32 "\n", env->es);
    cpu_fprintf(f, "CS    %02" PRIx32 "\n", env->cs);
#endif

    cpu_fprintf(f, "\n");
    for (i = 0; i < 4; i++) {
        cpu_fprintf(f, "BANK%d           ", i);
    }
    cpu_fprintf(f, "\n");
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 4; j++) {
            target_phys_addr_t bank_addr = GPR_START + (3 - j) * 8;
            if ((i % 2) == 0) {
                cpu_fprintf(f, " %s %04" PRIx16 "  %s %02" PRIx8 "  ",
                            gpr16_names[i / 2], lduw_phys(bank_addr + i),
                            gpr8_names[i], ldub_phys(bank_addr + i));
            } else {
                cpu_fprintf(f, "          %s %02" PRIx8 "  ",
                            gpr8_names[i], ldub_phys(bank_addr + i));
            }
        }
        cpu_fprintf(f, "\n");
    }
}

void restore_state_to_opc(CPUState *env, TranslationBlock *tb, int pc_pos)
{
    env->pc = gen_opc_pc[pc_pos];
}
