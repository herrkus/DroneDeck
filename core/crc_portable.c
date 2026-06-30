/* crc_portable.c -- portable C fallback for crc_x25.S.
 *
 * Built instead of the x86-64 assembly on non-x86-64 targets (e.g. ARM:
 * Apple Silicon, Raspberry Pi) so the optimized C++ core still compiles
 * everywhere. Same two symbols, same results -- only the hand-tuned asm is
 * traded for compiler-optimized C. (On x86-64, build.sh uses the asm instead.)
 */
#include <stdint.h>
#include <stddef.h>

/* g++ compiles a .c file as C++ and would mangle these names; force C linkage
 * so the extern "C" declarations in dronecore.cpp resolve either way. */
#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc_accumulate_buffer(uint16_t crc, const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        uint8_t tmp = (uint8_t)(buf[i] ^ (uint8_t)(crc & 0xff));
        tmp ^= (uint8_t)(tmp << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8)
                                    ^ ((uint16_t)tmp << 3)
                                    ^ ((uint16_t)tmp >> 4));
    }
    return crc;
}

uint16_t crc_slice8(uint16_t crc, const uint8_t *buf, size_t len, const void *tab) {
    const uint16_t (*T)[256] = (const uint16_t (*)[256])tab;
    size_t i = 0;
    for (; i + 8 <= len; i += 8) {
        uint8_t lo = (uint8_t)(crc & 0xff), hi = (uint8_t)(crc >> 8);
        crc = (uint16_t)(T[7][lo ^ buf[i]] ^ T[6][hi ^ buf[i + 1]]
                       ^ T[5][buf[i + 2]] ^ T[4][buf[i + 3]]
                       ^ T[3][buf[i + 4]] ^ T[2][buf[i + 5]]
                       ^ T[1][buf[i + 6]] ^ T[0][buf[i + 7]]);
    }
    for (; i < len; ++i) {
        uint8_t tmp = (uint8_t)(buf[i] ^ (uint8_t)(crc & 0xff));
        tmp ^= (uint8_t)(tmp << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8)
                                    ^ ((uint16_t)tmp << 3)
                                    ^ ((uint16_t)tmp >> 4));
    }
    return crc;
}

#ifdef __cplusplus
}  /* extern "C" */
#endif
