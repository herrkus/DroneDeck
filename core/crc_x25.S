/* crc_x25.S -- MAVLink CRC-16/MCRF4XX in hand-written x86-64 assembly.
 *
 * This is the protocol hot-path: every MAVLink frame (header + payload + the
 * per-message CRC_EXTRA seed byte) is run through this checksum on both the
 * send and receive sides, for every packet, at the full telemetry rate. That
 * makes it the one spot in the stack where dropping to assembly genuinely
 * earns its keep, so it lives in its own translation unit.
 *
 * Algorithm: CRC-16/MCRF4XX (a.k.a. the MAVLink "X25" accumulate), poly 0x1021
 * reflected, init 0xFFFF, no final xor. Reference C, per byte:
 *
 *     tmp = data ^ (crc & 0xff);
 *     tmp ^= (tmp << 4);                       // 8-bit truncated
 *     crc  = (crc >> 8) ^ (tmp << 8)
 *                       ^ (tmp << 3)
 *                       ^ (tmp >> 4);          // 16-bit truncated
 *
 * The hot loop below is fully register-resident (one byte load per iteration,
 * no tables, no branches in the body) and 4x unrolled to hide the dependency
 * chain. AT&T-free: we use Intel syntax for readability.
 *
 * System V AMD64 ABI:
 *   crc_accumulate_buffer(uint16_t crc, const uint8_t *buf, size_t len) -> uint16_t
 *   args in DI (crc), RSI (buf), RDX (len); result returned in AX.
 */

    .intel_syntax noprefix
    .text

/* --- one-byte CRC step -----------------------------------------------------
 * In:  eax = running crc (16-bit, upper bits clear), CL-addressed data in ecx
 * Out: eax = updated crc (16-bit), clobbers r8d, r9d, ecx
 * Implemented as a macro so the unrolled loop and the tail share one body.
 */
.macro CRC_STEP
    mov     r8d, eax
    xor     r8d, ecx              /* data ^ crc (only low byte matters)        */
    movzx   r8d, r8b              /* tmp = (data ^ crc) & 0xff                 */
    mov     r9d, r8d
    shl     r9d, 4
    xor     r8d, r9d
    movzx   r8d, r8b              /* tmp ^= (tmp << 4); tmp &= 0xff            */
    shr     eax, 8               /* crc >> 8                                   */
    mov     r9d, r8d
    shl     r9d, 8               /* tmp << 8                                   */
    xor     eax, r9d
    mov     r9d, r8d
    shl     r9d, 3               /* tmp << 3                                   */
    xor     eax, r9d
    mov     r9d, r8d
    shr     r9d, 4               /* tmp >> 4                                   */
    xor     eax, r9d
    movzx   eax, ax              /* crc &= 0xffff                              */
.endm

    .globl  crc_accumulate_buffer
    .type   crc_accumulate_buffer, @function
crc_accumulate_buffer:
    movzx   eax, di               /* crc -> eax (clears upper 16 bits)         */
    mov     r10, rdx              /* r10 = remaining length                    */

    /* ---- 4x unrolled main loop --------------------------------------------*/
    cmp     r10, 4
    jb      .Ltail
.Lblk4:
    movzx   ecx, byte ptr [rsi]
    CRC_STEP
    movzx   ecx, byte ptr [rsi+1]
    CRC_STEP
    movzx   ecx, byte ptr [rsi+2]
    CRC_STEP
    movzx   ecx, byte ptr [rsi+3]
    CRC_STEP
    add     rsi, 4
    sub     r10, 4
    cmp     r10, 4
    jae     .Lblk4

    /* ---- 1-byte tail ------------------------------------------------------*/
.Ltail:
    test    r10, r10
    jz      .Ldone
.Ltail1:
    movzx   ecx, byte ptr [rsi]
    CRC_STEP
    inc     rsi
    dec     r10
    jnz     .Ltail1
.Ldone:
    movzx   eax, ax
    ret
    .size   crc_accumulate_buffer, .-crc_accumulate_buffer

/* ---------------------------------------------------------------------------
 * crc_slice8 -- slicing-by-8 variant.
 *
 * The byte-wise routine above carries a hard serial dependency (each byte
 * needs the previous CRC), which the C compiler schedules just as well, so it
 * is no faster than -O3 C. Slicing-by-8 issues eight *independent* table
 * lookups per 8-byte block and only the final XOR feeds the next iteration,
 * exposing memory-level parallelism the serial form cannot. It beats the
 * compiler and -- because MAVLink frames are processed in 8-byte chunks plus a
 * short tail -- it speeds up the real per-message checksums too.
 *
 *   crc8 = T8[lo^b0] ^ T7[hi^b1] ^ T6[b2] ^ T5[b3]
 *        ^ T4[b4]    ^ T3[b5]    ^ T2[b6] ^ T1[b7]
 *
 * where lo/hi are the low/high bytes of the incoming CRC and Tk are the eight
 * 256-entry uint16 tables built in dronecore.cpp (row k-1, stride 512 bytes).
 *
 *   crc_slice8(uint16_t crc, const uint8_t *buf, size_t len, const void *tab)
 *   args in DI (crc), RSI (buf), RDX (len), RCX (tab); result in AX.
 */
    .globl  crc_slice8
    .type   crc_slice8, @function
crc_slice8:
    movzx   eax, di               /* crc */
    mov     r11, rdx              /* remaining length */

.Ls8_blk:
    cmp     r11, 8
    jb      .Ls8_tail
    /* idx0 = b0 ^ (crc & 0xff) */
    movzx   r8d, byte ptr [rsi]
    xor     r8b, al
    movzx   r8d, r8b
    /* idx1 = b1 ^ (crc >> 8) */
    mov     r10d, eax
    shr     r10d, 8
    movzx   r9d, byte ptr [rsi+1]
    xor     r9b, r10b
    movzx   r9d, r9b
    /* gather eight independent table lookups */
    movzx   eax,  word ptr [rcx + r8*2 + 3584]   /* T8[idx0] (row7) */
    movzx   r10d, word ptr [rcx + r9*2 + 3072]   /* T7[idx1] (row6) */
    xor     eax, r10d
    movzx   r8d,  byte ptr [rsi+2]
    movzx   r8d,  word ptr [rcx + r8*2 + 2560]   /* T6[b2]   (row5) */
    xor     eax, r8d
    movzx   r9d,  byte ptr [rsi+3]
    movzx   r9d,  word ptr [rcx + r9*2 + 2048]   /* T5[b3]   (row4) */
    xor     eax, r9d
    movzx   r8d,  byte ptr [rsi+4]
    movzx   r8d,  word ptr [rcx + r8*2 + 1536]   /* T4[b4]   (row3) */
    xor     eax, r8d
    movzx   r9d,  byte ptr [rsi+5]
    movzx   r9d,  word ptr [rcx + r9*2 + 1024]   /* T3[b5]   (row2) */
    xor     eax, r9d
    movzx   r8d,  byte ptr [rsi+6]
    movzx   r8d,  word ptr [rcx + r8*2 + 512]    /* T2[b6]   (row1) */
    xor     eax, r8d
    movzx   r9d,  byte ptr [rsi+7]
    movzx   r9d,  word ptr [rcx + r9*2 + 0]      /* T1[b7]   (row0) */
    xor     eax, r9d
    add     rsi, 8
    sub     r11, 8
    jmp     .Ls8_blk

    /* tail (<8 bytes): plain byte-wise; rcx is free to reuse as data scratch */
.Ls8_tail:
    test    r11, r11
    jz      .Ls8_done
.Ls8_t1:
    movzx   ecx, byte ptr [rsi]
    CRC_STEP
    inc     rsi
    dec     r11
    jnz     .Ls8_t1
.Ls8_done:
    movzx   eax, ax
    ret
    .size   crc_slice8, .-crc_slice8

/* Mark the stack non-executable (no trampolines here). */
    .section .note.GNU-stack,"",@progbits
