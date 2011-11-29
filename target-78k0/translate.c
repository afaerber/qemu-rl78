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
    cpu_psw = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPU78K0State, psw), "psw");
    cpu_sp = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPU78K0State, sp), "sp");
#ifdef TARGET_RL78
    cpu_es = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPU78K0State, es), "es");
    cpu_cs = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPU78K0State, cs), "cs");
#endif

#define GEN_HELPER 2
#include "helper.h"
}

#define GPR_START 0xffee0

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

typedef int (*OpcodeHandler)(RL78CPU *cpu, uint8_t opcode, DisasContext *s);

/* SEL RB0  (0xcf)
 * SEL RB1  (0xdf)
 * SEL RB2  (0xef)
 * SEL RB3  (0xff) */
static int rl78_disas_sel_rb(RL78CPU *cpu, uint8_t opcode, DisasContext *s)
{
    unsigned int bank = (opcode >> 4) - 0xc;
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
    return 2;
}

static const OpcodeHandler rl78_2nd_map[256] = {
    [0xcf] = rl78_disas_sel_rb,
    [0xdf] = rl78_disas_sel_rb,
    [0xef] = rl78_disas_sel_rb,
    [0xff] = rl78_disas_sel_rb,
};

/* SHRW AX, 1   (0x1e)
 * SHRW AX, 2   (0x2e)
 * SHRW AX, 3   (0x3e)
 * SHRW AX, 4   (0x4e)
 * SHRW AX, 5   (0x5e)
 * SHRW AX, 6   (0x6e)
 * SHRW AX, 7   (0x7e)
 * SHRW AX, 8   (0x8e)
 * SHRW AX, 9   (0x9e)
 * SHRW AX, 10  (0xae)
 * SHRW AX, 11  (0xbe)
 * SHRW AX, 12  (0xce)
 * SHRW AX, 13  (0xde)
 * SHRW AX, 14  (0xee)
 * SHRW AX, 15  (0xfe) */
static int rl78_disas_shrw_ax(RL78CPU *cpu, uint8_t opcode, DisasContext *s)
{
    unsigned int cnt = opcode >> 4;
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

    return 2;
}

static const OpcodeHandler rl78_4th_map[256] = {
    [0x1e] = rl78_disas_shrw_ax,
    [0x2e] = rl78_disas_shrw_ax,
    [0x3e] = rl78_disas_shrw_ax,
    [0x4e] = rl78_disas_shrw_ax,
    [0x5e] = rl78_disas_shrw_ax,
    [0x6e] = rl78_disas_shrw_ax,
    [0x7e] = rl78_disas_shrw_ax,
    [0x8e] = rl78_disas_shrw_ax,
    [0x9e] = rl78_disas_shrw_ax,
    [0xae] = rl78_disas_shrw_ax,
    [0xbe] = rl78_disas_shrw_ax,
    [0xce] = rl78_disas_shrw_ax,
    [0xde] = rl78_disas_shrw_ax,
    [0xee] = rl78_disas_shrw_ax,
    [0xfe] = rl78_disas_shrw_ax,
};

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

/* MOVW AX,#word  (0x30)
 * MOVW BC,#word  (0x32)
 * MOVW DE,#word  (0x34)
 * MOVW HL,#word  (0x36) */
static int rl78_disas_movw_rp_word(RL78CPU *cpu, uint8_t opcode, DisasContext *s)
{
    RL78GPR16 regpair = opcode & 0xf;
    uint16_t data = cpu_lduw_code(&cpu->env, s->pc + 1);
    TCGv value;

    LOG_ASM("MOVW %s, #%04" PRIx16 "H\n", gpr16_names[regpair / 2], data);
    value = tcg_const_tl(data);
    gen_gpr_st16(regpair, value);
    tcg_temp_free(value);
    return 3;
}

/* MOVW sfrp,#word */
static int rl78_disas_movw_sfrp_word(RL78CPU *cpu, uint8_t opcode, DisasContext *s)
{
    uint8_t sfrp = cpu_ldub_code(&cpu->env, s->pc + 1) & ~0x1;
    uint16_t data = cpu_lduw_code(&cpu->env, s->pc + 2);
    TCGv addr = tcg_const_tl(0xFFF00 | sfrp);
    TCGv val = tcg_const_tl(data);

    LOG_ASM("MOVW %02" PRIx8 "H, #%04" PRIx16 "H\n", sfrp, data);
    tcg_gen_qemu_st16(val, addr, 0);
    tcg_temp_free(addr);
    tcg_temp_free(val);
    return 4;
}

#ifdef TARGET_RL78
/* MOV ES, #byte */
static int rl78_disas_mov_es_byte(RL78CPU *cpu, uint8_t opcode, DisasContext *s)
{
    uint8_t data = cpu_ldub_code(&cpu->env, s->pc + 1);
    TCGv_i32 value = tcg_const_i32(data & 0xf);

    LOG_ASM("MOV ES, #%02" PRIx8 "H\n", data);
    tcg_gen_mov_i32(cpu_es, value);
    tcg_temp_free_i32(value);
    return 2;
}
#endif

/* 2nd MAP */
static int rl78_disas_2nd_map(RL78CPU *cpu, uint8_t opc, DisasContext *s)
{
    uint8_t opc2;

    opc2 = cpu_ldub_code(&cpu->env, s->pc + 1);

    if (likely(rl78_2nd_map[opc2] != NULL)) {
        return rl78_2nd_map[opc2](cpu, opc2, s);
    } else {
        qemu_log("unimplemented 0x%" PRIx8 " opcode 0x%" PRIx8 "\n", opc, opc2);
        // TODO
        tcg_gen_movi_tl(env_pc, s->pc);
        s->is_jmp = DISAS_UPDATE;
        return 2;
    }
}

/* 4th MAP */
static int rl78_disas_4th_map(RL78CPU *cpu, uint8_t opc, DisasContext *s)
{
    uint8_t opc2;

    opc2 = cpu_ldub_code(&cpu->env, s->pc + 1);

    if (likely(rl78_4th_map[opc2] != NULL)) {
        return rl78_4th_map[opc2](cpu, opc2, s);
    } else {
        qemu_log("unimplemented 0x%" PRIx8 " opcode 0x%" PRIx8 "\n", opc, opc2);
        // TODO
        tcg_gen_movi_tl(env_pc, s->pc);
        s->is_jmp = DISAS_UPDATE;
        return 2;
    }
}

static const OpcodeHandler rl78_1st_map[256] = {
    [0x30] = rl78_disas_movw_rp_word,
    [0x31] = rl78_disas_4th_map,
    [0x32] = rl78_disas_movw_rp_word,
    [0x34] = rl78_disas_movw_rp_word,
    [0x36] = rl78_disas_movw_rp_word,
#ifdef TARGET_RL78
    [0x41] = rl78_disas_mov_es_byte,
#endif
    [0x61] = rl78_disas_2nd_map,
    [0xCB] = rl78_disas_movw_sfrp_word,
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
    int i, j;

    cpu_fprintf(f, "PC %05" PRIx32 "\n\n", cpu->env.pc);
    cpu_fprintf(f, "SP  %04" PRIx32 "\n", cpu->env.sp);
    cpu_fprintf(f, "PSW   %02" PRIx32 "\n", cpu->env.psw);
#ifdef TARGET_RL78
    cpu_fprintf(f, "ES    %02" PRIx32 "\n", cpu->env.es);
    cpu_fprintf(f, "CS    %02" PRIx32 "\n", cpu->env.cs);
#endif

    cpu_fprintf(f, "\n");
    for (i = 0; i < 4; i++) {
        cpu_fprintf(f, "BANK%d           ", i);
    }
    cpu_fprintf(f, "\n");
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 4; j++) {
            hwaddr bank_addr = GPR_START + (3 - j) * 8;
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
