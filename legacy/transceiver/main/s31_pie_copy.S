/*
 * SPDX-License-Identifier: MIT
 *
 * ESP32-S31 PIE copy primitive.  Both addresses and byte_count must be
 * 16-byte aligned.  The first PIE instruction used by a FreeRTOS task invokes
 * the port's lazy coprocessor handler, which pins/owns and saves the Q-register
 * context.  Call this routine once outside a critical section before using it
 * with interrupts masked.
 */
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#if CONFIG_IDF_TARGET_ESP32S31 && SOC_CPU_HAS_PIE

    .text
    .align  4
    .type   s31_pie_memcpy_aligned, @function
    .global s31_pie_memcpy_aligned
s31_pie_memcpy_aligned:
    /*
     * Eight Q registers and two load/store groups amortize the loop branch
     * over 256 bytes.  A matched 4 MS/s Pluto test reduced the mean protected
     * handoff from 422.8 to 409.35 samples.  A 512-byte unroll was rejected:
     * it provided little further improvement and needlessly enlarged this
     * critical primitive.
     */
    li      t0, 256
    bltu    a2, t0, 2f
1:
    esp.vld.128.ip q0, a1, 16
    esp.vld.128.ip q1, a1, 16
    esp.vld.128.ip q2, a1, 16
    esp.vld.128.ip q3, a1, 16
    esp.vld.128.ip q4, a1, 16
    esp.vld.128.ip q5, a1, 16
    esp.vld.128.ip q6, a1, 16
    esp.vld.128.ip q7, a1, 16
    esp.vst.128.ip q0, a0, 16
    esp.vst.128.ip q1, a0, 16
    esp.vst.128.ip q2, a0, 16
    esp.vst.128.ip q3, a0, 16
    esp.vst.128.ip q4, a0, 16
    esp.vst.128.ip q5, a0, 16
    esp.vst.128.ip q6, a0, 16
    esp.vst.128.ip q7, a0, 16
    esp.vld.128.ip q0, a1, 16
    esp.vld.128.ip q1, a1, 16
    esp.vld.128.ip q2, a1, 16
    esp.vld.128.ip q3, a1, 16
    esp.vld.128.ip q4, a1, 16
    esp.vld.128.ip q5, a1, 16
    esp.vld.128.ip q6, a1, 16
    esp.vld.128.ip q7, a1, 16
    esp.vst.128.ip q0, a0, 16
    esp.vst.128.ip q1, a0, 16
    esp.vst.128.ip q2, a0, 16
    esp.vst.128.ip q3, a0, 16
    esp.vst.128.ip q4, a0, 16
    esp.vst.128.ip q5, a0, 16
    esp.vst.128.ip q6, a0, 16
    esp.vst.128.ip q7, a0, 16
    addi    a2, a2, -256
    bgeu    a2, t0, 1b
2:
    beqz    a2, 4f
3:
    esp.vld.128.ip q0, a1, 16
    esp.vst.128.ip q0, a0, 16
    addi    a2, a2, -16
    bnez    a2, 3b
4:
    ret
    .size s31_pie_memcpy_aligned, .-s31_pie_memcpy_aligned

    /*
     * Pack ESP modem dump words into interleaved IQ8 bytes.
     *
     *   a0: destination (2 * sample_count bytes; may be unaligned)
     *   a1: source uint32_t words (may be unaligned)
     *   a2: sample_count (non-zero multiple of eight)
     *
     * Per word, I is bits 19:12 and Q is bits 9:2. Two 128-bit loads carry
     * eight samples. After shifting/masking, each 32-bit lane contains one
     * IQ pair in its low halfword; VUNZIP.16 compacts the even halfwords from
     * q2/q3 into q2, which is then one complete 16-byte output vector.
     */
    .align  4
    .type   s31_pie_pack_iq8, @function
    .global s31_pie_pack_iq8
s31_pie_pack_iq8:
    /* Preserve task-local PIE control state and enable misaligned vector
     * accesses for the packed frame's naturally 4/8-byte-aligned fields. */
    esp.movx.r.cfg a3
    esp.movx.r.sar a4
    ori     a5, a3, 3
    esp.movx.w.cfg a5

    li      t3, 0xff
    esp.zero.q q6
    esp.movi.32.q q6, t3, 0
    esp.movi.32.q q6, t3, 1
    esp.movi.32.q q6, t3, 2
    esp.movi.32.q q6, t3, 3
1:
    esp.vld.128.ip q0, a1, 16
    esp.vld.128.ip q1, a1, 16

    li      t4, 12
    esp.movx.w.sar t4
    esp.vsr.u32 q2, q0
    esp.vsr.u32 q3, q1
    esp.andq q2, q2, q6
    esp.andq q3, q3, q6

    li      t4, 2
    esp.movx.w.sar t4
    esp.vsr.u32 q4, q0
    esp.vsr.u32 q5, q1
    esp.andq q4, q4, q6
    esp.andq q5, q5, q6

    li      t4, 8
    esp.movx.w.sar t4
    esp.vsl.32 q4, q4
    esp.vsl.32 q5, q5
    esp.orq q2, q2, q4
    esp.orq q3, q3, q5
    esp.vunzip.16 q2, q3
    esp.vst.128.ip q2, a0, 16

    addi    a2, a2, -8
    bnez    a2, 1b

    esp.movx.w.sar a4
    esp.movx.w.cfg a3
    ret
    .size s31_pie_pack_iq8, .-s31_pie_pack_iq8

    /*
     * Copy eight completed dump words while disconnecting the modem writer
     * only for the two 128-bit loads.  The destination stores deliberately
     * happen after ownership has been returned to the writer.  a0/a1 must be
     * 16-byte aligned; a2 is HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG and a3 is the
     * writer ownership mask.  Return the measured writer-off cycle count.
     */
    .align  4
    .type   s31_pie_microgate_copy8, @function
    .global s31_pie_microgate_copy8
s31_pie_microgate_copy8:
    csrr    t0, mcycle
    sw      zero, 0(a2)
    fence   rw, rw
    esp.vld.128.ip q0, a1, 16
    esp.vld.128.ip q1, a1, 16
    sw      a3, 0(a2)
    fence   rw, rw
    csrr    t1, mcycle
    sub     t0, t1, t0
    esp.vst.128.ip q0, a0, 16
    esp.vst.128.ip q1, a0, 16
    mv      a0, t0
    ret
    .size s31_pie_microgate_copy8, .-s31_pie_microgate_copy8

    /* 32-word variant: use the complete Q register file so the writer can
     * remain connected for 32 sample periods between ownership handoffs. */
    .align  4
    .type   s31_pie_microgate_copy32, @function
    .global s31_pie_microgate_copy32
s31_pie_microgate_copy32:
    csrr    t0, mcycle
    sw      zero, 0(a2)
    fence   rw, rw
    esp.vld.128.ip q0, a1, 16
    esp.vld.128.ip q1, a1, 16
    esp.vld.128.ip q2, a1, 16
    esp.vld.128.ip q3, a1, 16
    esp.vld.128.ip q4, a1, 16
    esp.vld.128.ip q5, a1, 16
    esp.vld.128.ip q6, a1, 16
    esp.vld.128.ip q7, a1, 16
    sw      a3, 0(a2)
    fence   rw, rw
    csrr    t1, mcycle
    sub     t0, t1, t0
    esp.vst.128.ip q0, a0, 16
    esp.vst.128.ip q1, a0, 16
    esp.vst.128.ip q2, a0, 16
    esp.vst.128.ip q3, a0, 16
    esp.vst.128.ip q4, a0, 16
    esp.vst.128.ip q5, a0, 16
    esp.vst.128.ip q6, a0, 16
    esp.vst.128.ip q7, a0, 16
    mv      a0, t0
    ret
    .size s31_pie_microgate_copy32, .-s31_pie_microgate_copy32

#endif
