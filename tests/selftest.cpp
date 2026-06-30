// selftest.cpp -- native correctness + speed checks for the DroneDeck core.
// Built and run by ../build.sh. Exits non-zero on any failure.

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <vector>

extern "C" {
    uint16_t crc_accumulate_buffer(uint16_t, const uint8_t*, size_t);
    uint16_t mav_crc(const uint8_t*, size_t);
    uint16_t mav_crc_ref(const uint8_t*, size_t);
    uint16_t mav_crc_extra(const uint8_t*, size_t, uint8_t);
    void*    mav_new();
    void     mav_free(void*);
    int      mav_feed(void*, const uint8_t*, size_t);
    int      mav_encode_heartbeat(uint8_t, uint8_t, uint8_t, uint8_t*, int);
    int      mav_encode_command_long(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
                                     uint16_t, const float*, uint8_t*, int);
}

struct Decoded { uint32_t msgid; uint8_t sysid, compid, seq, nfields; double f[24]; };
extern "C" int mav_pop(void*, Decoded*);

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { std::printf("  FAIL: "); \
    std::printf(__VA_ARGS__); std::printf("\n"); ++failures; } } while (0)

int main() {
    std::printf("DroneDeck native self-test\n");

    // 1) Known CRC-16/MCRF4XX check value over ASCII "123456789" is 0x6F91.
    {
        const uint8_t s[] = "123456789";
        uint16_t c = mav_crc(s, 9);
        std::printf("[crc] MCRF4XX(\"123456789\") = 0x%04X (expect 0x6F91)\n", c);
        CHECK(c == 0x6F91, "known CRC vector mismatch");
    }

    // 2) Assembly CRC must agree with the portable C reference, all lengths.
    {
        std::vector<uint8_t> buf(4096);
        for (size_t i = 0; i < buf.size(); ++i) buf[i] = uint8_t((i * 131 + 7) & 0xff);
        bool agree = true;
        for (size_t len = 0; len <= buf.size(); ++len)
            if (mav_crc(buf.data(), len) != mav_crc_ref(buf.data(), len)) { agree = false; break; }
        std::printf("[crc] asm vs C reference over lengths 0..4096: %s\n", agree ? "match" : "MISMATCH");
        CHECK(agree, "asm/C CRC disagree");
    }

    // 3) Parser round-trip: encode a HEARTBEAT, feed it, pop it back.
    {
        uint8_t frame[64];
        int n = mav_encode_heartbeat(1, 1, 0, frame, sizeof(frame));
        CHECK(n == 17, "heartbeat frame len = %d (expect 17)", n);
        void* p = mav_new();
        int avail = mav_feed(p, frame, n);
        CHECK(avail == 1, "expected 1 decoded msg, got %d", avail);
        Decoded d;
        CHECK(mav_pop(p, &d) == 1, "pop failed");
        CHECK(d.msgid == 0, "msgid = %u (expect 0)", d.msgid);
        CHECK(d.nfields == 6, "heartbeat nfields = %u (expect 6)", d.nfields);
        CHECK(int(d.f[0]) == 6, "heartbeat type = %d (expect 6 = GCS)", int(d.f[0]));
        mav_free(p);
    }

    // 4) Resync: junk + partial-looking bytes before a valid frame still decodes.
    {
        uint8_t frame[64];
        int n = mav_encode_heartbeat(7, 1, 42, frame, sizeof(frame));
        std::vector<uint8_t> stream = {0x00, 0xFE, 0x13, 0xFD, 0x01, 0xFF};
        stream.insert(stream.end(), frame, frame + n);
        void* p = mav_new();
        int avail = mav_feed(p, stream.data(), stream.size());
        CHECK(avail == 1, "resync: expected 1 msg, got %d", avail);
        Decoded d;
        mav_pop(p, &d);
        CHECK(d.sysid == 7 && d.seq == 42, "resync: sysid/seq = %u/%u", d.sysid, d.seq);
        mav_free(p);
    }

    // 5) Split feed: a frame delivered in two halves must still decode once whole.
    {
        uint8_t frame[64];
        int n = mav_encode_heartbeat(2, 1, 1, frame, sizeof(frame));
        void* p = mav_new();
        CHECK(mav_feed(p, frame, 5) == 0, "partial feed should yield 0");
        CHECK(mav_feed(p, frame + 5, n - 5) == 1, "completing feed should yield 1");
        mav_free(p);
    }

    // 6) COMMAND_LONG encode -> decode parity.
    {
        float params[7] = {1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        uint8_t frame[64];
        int n = mav_encode_command_long(255, 0, 0, 1, 1, 400 /*MAV_CMD_COMPONENT_ARM_DISARM*/,
                                        params, frame, sizeof(frame));
        CHECK(n == 41, "command_long frame len = %d (expect 41)", n);
        void* p = mav_new();
        mav_feed(p, frame, n);
        Decoded d;
        CHECK(mav_pop(p, &d) == 1 && d.msgid == 76, "command_long decode");
        CHECK(int(d.f[0]) == 400, "command = %d (expect 400)", int(d.f[0]));
        CHECK(d.f[1] == 1.0, "param1 = %f (expect 1 = arm)", d.f[1]);
        mav_free(p);
    }

    // 7) Throughput: assembly CRC vs portable C over a large buffer.
    {
        std::vector<uint8_t> big(1u << 22); // 4 MiB
        for (size_t i = 0; i < big.size(); ++i) big[i] = uint8_t(i * 2654435761u >> 24);
        const int iters = 64;
        auto bench = [&](uint16_t (*fn)(const uint8_t*, size_t)) {
            volatile uint16_t sink = 0;
            auto t0 = std::chrono::steady_clock::now();
            for (int k = 0; k < iters; ++k) sink ^= fn(big.data(), big.size());
            auto t1 = std::chrono::steady_clock::now();
            double s = std::chrono::duration<double>(t1 - t0).count();
            (void)sink;
            return (double(big.size()) * iters / (1024.0 * 1024.0)) / s; // MiB/s
        };
        double asm_mbs = bench(mav_crc);
        double c_mbs   = bench(mav_crc_ref);
        std::printf("[bench] asm CRC  : %8.1f MiB/s\n", asm_mbs);
        std::printf("[bench] C   CRC  : %8.1f MiB/s\n", c_mbs);
        std::printf("[bench] speedup  : %.2fx\n", asm_mbs / c_mbs);
    }

    std::printf(failures ? "\nSELF-TEST FAILED (%d)\n" : "\nSELF-TEST PASSED\n", failures);
    return failures ? 1 : 0;
}
