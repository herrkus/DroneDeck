// dronecore.cpp -- native MAVLink engine for DroneDeck.
//
// Layering of the three languages in this project:
//   * x86-64 assembly (crc_x25.S) -- the per-byte CRC hot path.
//   * C++ (this file)              -- streaming frame parser, message decoders,
//                                     encoders, and a flat C ABI.
//   * Python (../app, ../sim)      -- GUI, links, and the test simulator, which
//                                     load this engine through ctypes.
//
// The C++ side owns everything that runs at telemetry rate: resync, CRC
// validation (delegated to the asm routine), and decoding wire bytes into a
// flat numeric struct that crosses the ctypes boundary with zero parsing on
// the Python side.

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include <deque>

// ---- assembly hot path (crc_x25.S) -----------------------------------------
// CRC-16/MCRF4XX accumulate over a buffer, seeded with `crc`.
extern "C" uint16_t crc_accumulate_buffer(uint16_t crc, const uint8_t* buf, size_t len);
// Slicing-by-8 variant; `tab` points at the eight 256-entry tables below.
extern "C" uint16_t crc_slice8(uint16_t crc, const uint8_t* buf, size_t len, const void* tab);

namespace {

// Slicing-by-8 tables, built once at load time from the single-byte MCRF4XX
// step. Row k holds the table the asm calls T(k+1). crc_slice8 reads these.
uint16_t CRC_T[8][256];
[[maybe_unused]] const bool crc_tables_ready = [] {
    for (int i = 0; i < 256; ++i) {
        uint8_t tmp = uint8_t(i);
        tmp ^= uint8_t(tmp << 4);
        CRC_T[0][i] = uint16_t((uint16_t(tmp) << 8) ^ (uint16_t(tmp) << 3) ^ (uint16_t(tmp) >> 4));
    }
    for (int k = 1; k < 8; ++k)
        for (int i = 0; i < 256; ++i) {
            uint16_t p = CRC_T[k - 1][i];
            CRC_T[k][i] = uint16_t((p >> 8) ^ CRC_T[0][p & 0xff]);
        }
    return true;
}();

// Single entry point used everywhere below.
inline uint16_t crc_buf(uint16_t crc, const uint8_t* buf, size_t len) {
    return crc_slice8(crc, buf, len, CRC_T);
}

// ---- little-endian readers from a (zero-padded) payload buffer --------------
inline uint16_t rd_u16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
inline int16_t  rd_i16(const uint8_t* p) { return int16_t(rd_u16(p)); }
inline uint32_t rd_u32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline int32_t  rd_i32(const uint8_t* p) { return int32_t(rd_u32(p)); }
inline uint64_t rd_u64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= uint64_t(p[i]) << (8 * i);
    return v;
}
inline float    rd_f32(const uint8_t* p) {
    float f; uint32_t u = rd_u32(p); std::memcpy(&f, &u, 4); return f;
}

// ---- message catalogue ------------------------------------------------------
// `extra` is the MAVLink CRC_EXTRA seed; `len` is the full (untruncated)
// payload length, used to zero-pad MAVLink v2 truncated payloads before decode.
struct MsgInfo { uint32_t id; uint8_t extra; uint8_t len; };

constexpr MsgInfo MSGS[] = {
    {0,   50,  9},   // HEARTBEAT
    {1,  124, 31},   // SYS_STATUS
    {24,  24, 30},   // GPS_RAW_INT
    {30,  39, 28},   // ATTITUDE
    {33, 104, 28},   // GLOBAL_POSITION_INT
    {65, 118, 42},   // RC_CHANNELS
    {69, 243, 11},   // MANUAL_CONTROL
    {74,  20, 20},   // VFR_HUD
    {75, 158, 35},   // COMMAND_INT
    {76, 152, 33},   // COMMAND_LONG
    {77, 143,  3},   // COMMAND_ACK
    {86,   5, 53},   // SET_POSITION_TARGET_GLOBAL_INT
    {253, 83, 51},   // STATUSTEXT
    {20, 214, 20},   // PARAM_REQUEST_READ
    {21, 159,  2},   // PARAM_REQUEST_LIST
    {22, 220, 25},   // PARAM_VALUE
    {23, 168, 23},   // PARAM_SET
    {42,  28,  2},   // MISSION_CURRENT
    {43, 132,  3},   // MISSION_REQUEST_LIST (+mission_type)
    {44, 221,  5},   // MISSION_COUNT (+mission_type)
    {45, 232,  3},   // MISSION_CLEAR_ALL (+mission_type)
    {46,  11,  2},   // MISSION_ITEM_REACHED
    {47, 153,  4},   // MISSION_ACK (+mission_type)
    {51, 196,  5},   // MISSION_REQUEST_INT (+mission_type)
    {73,  38, 38},   // MISSION_ITEM_INT (+mission_type)
    {117, 128,  6},  // LOG_REQUEST_LIST
    {118,  56, 14},  // LOG_ENTRY
    {119, 116, 12},  // LOG_REQUEST_DATA
    {120, 134, 97},  // LOG_DATA
    {122, 203,  2},  // LOG_REQUEST_END
    {109, 185,  9},  // RADIO_STATUS
    {141,  47, 32},  // ALTITUDE
    {147, 154, 36},  // BATTERY_STATUS
    {241,  90, 32},  // VIBRATION
    {193,  71, 22},  // EKF_STATUS_REPORT
    {230, 163, 42},  // ESTIMATOR_STATUS
    {231, 105, 40},  // WIND_COV
    {265,  26, 20},  // MOUNT_ORIENTATION (yaw_absolute is an extension: excluded from CRC)
    {246, 184, 38},  // ADSB_VEHICLE
    {126, 194, 79},  // SERIAL_CONTROL (PX4 nsh shell passthrough)
};

const MsgInfo* find_info(uint32_t id) {
    for (const auto& m : MSGS) if (m.id == id) return &m;
    return nullptr;
}

} // namespace

// ---- flat decoded message (mirrored by ctypes in app/core.py) ---------------
// Field order per message id is documented here and MUST stay in lockstep with
// FIELDS in app/core.py. Values are raw wire units (scaling happens in Python).
struct Decoded {
    uint32_t msgid;     // 0
    uint8_t  sysid;     // 4
    uint8_t  compid;    // 5
    uint8_t  seq;       // 6
    uint8_t  nfields;   // 7
    double   f[24];     // 8
    char     text[96];  // 200: STATUSTEXT/param_id (NUL-terminated) or LOG_DATA's
                        //      90-byte blob (length given by the `count` field)
};

namespace {

void decode(uint32_t msgid, const uint8_t* pl, Decoded& d) {
    d.nfields = 0;
    auto push = [&](double v) { if (d.nfields < 24) d.f[d.nfields++] = v; };
    switch (msgid) {
    case 0:  // HEARTBEAT: type, autopilot, base_mode, custom_mode, system_status, mavlink_version
        push(pl[4]); push(pl[5]); push(pl[6]); push(rd_u32(pl + 0)); push(pl[7]); push(pl[8]);
        break;
    case 1:  // SYS_STATUS: voltage_battery(mV), current_battery(cA), battery_remaining(%), load(0.1%),
             //            then sensors present/enabled/health bitmasks (uint32 each)
        push(rd_u16(pl + 14)); push(rd_i16(pl + 16)); push(int8_t(pl[30])); push(rd_u16(pl + 12));
        push(rd_u32(pl + 0)); push(rd_u32(pl + 4)); push(rd_u32(pl + 8));
        break;
    case 24: // GPS_RAW_INT: fix_type, sats, lat(1e7), lon(1e7), alt(mm), eph, vel(cm/s), cog(cdeg)
        push(pl[28]); push(pl[29]); push(rd_i32(pl + 8)); push(rd_i32(pl + 12));
        push(rd_i32(pl + 16)); push(rd_u16(pl + 20)); push(rd_u16(pl + 24)); push(rd_u16(pl + 26));
        break;
    case 30: // ATTITUDE: roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, time_boot_ms
        push(rd_f32(pl + 4)); push(rd_f32(pl + 8)); push(rd_f32(pl + 12));
        push(rd_f32(pl + 16)); push(rd_f32(pl + 20)); push(rd_f32(pl + 24)); push(rd_u32(pl + 0));
        break;
    case 33: // GLOBAL_POSITION_INT: lat, lon, alt(mm), rel_alt(mm), vx, vy, vz, hdg(cdeg), time_boot_ms
        push(rd_i32(pl + 4)); push(rd_i32(pl + 8)); push(rd_i32(pl + 12)); push(rd_i32(pl + 16));
        push(rd_i16(pl + 20)); push(rd_i16(pl + 22)); push(rd_i16(pl + 24)); push(rd_u16(pl + 26));
        push(rd_u32(pl + 0));
        break;
    case 65: // RC_CHANNELS: time_boot_ms, chan1..18_raw, chancount, rssi
        push(rd_u32(pl + 0));
        for (int i = 0; i < 18; ++i) push(rd_u16(pl + 4 + 2 * i));
        push(pl[40]); push(pl[41]);
        break;
    case 109: // RADIO_STATUS: rxerrors, fixed, rssi, remrssi, txbuf, noise, remnoise
        push(rd_u16(pl + 0)); push(rd_u16(pl + 2));
        push(pl[4]); push(pl[5]); push(pl[6]); push(pl[7]); push(pl[8]);
        break;
    case 141: // ALTITUDE: time_usec, monotonic, amsl, local, relative, terrain, bottom_clearance
        push(rd_u64(pl + 0));
        push(rd_f32(pl + 8)); push(rd_f32(pl + 12)); push(rd_f32(pl + 16));
        push(rd_f32(pl + 20)); push(rd_f32(pl + 24)); push(rd_f32(pl + 28));
        break;
    case 147: // BATTERY_STATUS: current_consumed, energy_consumed, temperature, voltages[10],
              //                 current_battery, id, battery_function, type, battery_remaining
        push(rd_i32(pl + 0)); push(rd_i32(pl + 4)); push(rd_i16(pl + 8));
        for (int i = 0; i < 10; ++i) push(rd_u16(pl + 10 + 2 * i));
        push(rd_i16(pl + 30)); push(pl[32]); push(pl[33]); push(pl[34]); push(int8_t(pl[35]));
        break;
    case 241: // VIBRATION: time_usec, vibration_x/y/z, clipping_0/1/2
        push(rd_u64(pl + 0));
        push(rd_f32(pl + 8)); push(rd_f32(pl + 12)); push(rd_f32(pl + 16));
        push(rd_u32(pl + 20)); push(rd_u32(pl + 24)); push(rd_u32(pl + 28));
        break;
    case 193: // EKF_STATUS_REPORT: velocity_var, pos_horiz_var, pos_vert_var, compass_var, terrain_var, flags
        push(rd_f32(pl + 0)); push(rd_f32(pl + 4)); push(rd_f32(pl + 8));
        push(rd_f32(pl + 12)); push(rd_f32(pl + 16)); push(rd_u16(pl + 20));
        break;
    case 230: // ESTIMATOR_STATUS: time_usec, vel/pos_h/pos_v/mag/hagl/tas ratios, pos_h/v accuracy, flags
        push(rd_u64(pl + 0));
        push(rd_f32(pl + 8)); push(rd_f32(pl + 12)); push(rd_f32(pl + 16));
        push(rd_f32(pl + 20)); push(rd_f32(pl + 24)); push(rd_f32(pl + 28));
        push(rd_f32(pl + 32)); push(rd_f32(pl + 36)); push(rd_u16(pl + 40));
        break;
    case 231: // WIND_COV: time_usec, wind_x/y/z, var_horiz/vert, wind_alt, horiz/vert accuracy
        push(rd_u64(pl + 0));
        push(rd_f32(pl + 8)); push(rd_f32(pl + 12)); push(rd_f32(pl + 16));
        push(rd_f32(pl + 20)); push(rd_f32(pl + 24)); push(rd_f32(pl + 28));
        push(rd_f32(pl + 32)); push(rd_f32(pl + 36));
        break;
    case 265: // MOUNT_ORIENTATION: time_boot_ms, roll, pitch, yaw, yaw_absolute (deg)
        push(rd_u32(pl + 0));
        push(rd_f32(pl + 4)); push(rd_f32(pl + 8)); push(rd_f32(pl + 12));
        push(rd_f32(pl + 16));
        break;
    case 75: // COMMAND_INT: param1-4, x, y, z, command, target_sys, target_comp, frame, current, autocont
        push(rd_f32(pl + 0)); push(rd_f32(pl + 4)); push(rd_f32(pl + 8)); push(rd_f32(pl + 12));
        push(rd_i32(pl + 16)); push(rd_i32(pl + 20)); push(rd_f32(pl + 24));
        push(rd_u16(pl + 28)); push(pl[30]); push(pl[31]); push(pl[32]); push(pl[33]); push(pl[34]);
        break;
    case 69: // MANUAL_CONTROL: x, y, z, r, buttons, target
        push(rd_i16(pl + 0)); push(rd_i16(pl + 2)); push(rd_i16(pl + 4)); push(rd_i16(pl + 6));
        push(rd_u16(pl + 8)); push(pl[10]);
        break;
    case 74: // VFR_HUD: airspeed, groundspeed, alt, climb, heading(deg), throttle(%)
        push(rd_f32(pl + 0)); push(rd_f32(pl + 4)); push(rd_f32(pl + 8)); push(rd_f32(pl + 12));
        push(rd_i16(pl + 16)); push(rd_u16(pl + 18));
        break;
    case 76: // COMMAND_LONG: command, param1..7
        push(rd_u16(pl + 28)); push(rd_f32(pl + 0)); push(rd_f32(pl + 4)); push(rd_f32(pl + 8));
        push(rd_f32(pl + 12)); push(rd_f32(pl + 16)); push(rd_f32(pl + 20)); push(rd_f32(pl + 24));
        break;
    case 77: // COMMAND_ACK: command, result
        push(rd_u16(pl + 0)); push(pl[2]);
        break;
    case 86: // SET_POSITION_TARGET_GLOBAL_INT: lat_int, lon_int, alt, type_mask
        push(rd_i32(pl + 4)); push(rd_i32(pl + 8)); push(rd_f32(pl + 12)); push(rd_u16(pl + 48));
        break;
    case 253: // STATUSTEXT: severity + text[50]
        push(pl[0]);
        std::memcpy(d.text, pl + 1, 50);
        d.text[50] = '\0';
        break;
    case 20: // PARAM_REQUEST_READ: param_index (+ param_id in text)
        push(rd_i16(pl + 0));
        std::memcpy(d.text, pl + 4, 16); d.text[16] = '\0';
        break;
    case 21: // PARAM_REQUEST_LIST: target_system, target_component
        push(pl[0]); push(pl[1]);
        break;
    case 22: // PARAM_VALUE: param_value, param_count, param_index, param_type (+ param_id in text)
        push(rd_f32(pl + 0)); push(rd_u16(pl + 4)); push(rd_u16(pl + 6)); push(pl[24]);
        std::memcpy(d.text, pl + 8, 16); d.text[16] = '\0';
        break;
    case 23: // PARAM_SET: param_value, param_type (+ param_id in text)
        push(rd_f32(pl + 0)); push(pl[22]);
        std::memcpy(d.text, pl + 6, 16); d.text[16] = '\0';
        break;
    case 42: // MISSION_CURRENT: seq
        push(rd_u16(pl + 0));
        break;
    case 43: // MISSION_REQUEST_LIST: target_system, target_component, mission_type
        push(pl[0]); push(pl[1]); push(pl[2]);
        break;
    case 44: // MISSION_COUNT: count, target_system, target_component, mission_type
        push(rd_u16(pl + 0)); push(pl[2]); push(pl[3]); push(pl[4]);
        break;
    case 45: // MISSION_CLEAR_ALL: target_system, target_component, mission_type
        push(pl[0]); push(pl[1]); push(pl[2]);
        break;
    case 46: // MISSION_ITEM_REACHED: seq
        push(rd_u16(pl + 0));
        break;
    case 47: // MISSION_ACK: target_system, target_component, type, mission_type
        push(pl[0]); push(pl[1]); push(pl[2]); push(pl[3]);
        break;
    case 51: // MISSION_REQUEST_INT: seq, target_system, target_component, mission_type
        push(rd_u16(pl + 0)); push(pl[2]); push(pl[3]); push(pl[4]);
        break;
    case 73: // MISSION_ITEM_INT: seq, frame, command, current, autocontinue, param1..4, x, y, z, mission_type
        push(rd_u16(pl + 28)); push(pl[34]); push(rd_u16(pl + 30)); push(pl[35]); push(pl[36]);
        push(rd_f32(pl + 0)); push(rd_f32(pl + 4)); push(rd_f32(pl + 8)); push(rd_f32(pl + 12));
        push(rd_i32(pl + 16)); push(rd_i32(pl + 20)); push(rd_f32(pl + 24)); push(pl[37]);
        break;
    case 117: // LOG_REQUEST_LIST: start, end, target_system, target_component
        push(rd_u16(pl + 0)); push(rd_u16(pl + 2)); push(pl[4]); push(pl[5]);
        break;
    case 118: // LOG_ENTRY: time_utc, size, id, num_logs, last_log_num
        push(rd_u32(pl + 0)); push(rd_u32(pl + 4)); push(rd_u16(pl + 8));
        push(rd_u16(pl + 10)); push(rd_u16(pl + 12));
        break;
    case 119: // LOG_REQUEST_DATA: ofs, count, id, target_system, target_component
        push(rd_u32(pl + 0)); push(rd_u32(pl + 4)); push(rd_u16(pl + 8)); push(pl[10]); push(pl[11]);
        break;
    case 120: // LOG_DATA: ofs, id, count + 90-byte blob into text[]
        push(rd_u32(pl + 0)); push(rd_u16(pl + 4)); push(pl[6]);
        std::memcpy(d.text, pl + 7, 90);
        break;
    case 122: // LOG_REQUEST_END: target_system, target_component
        push(pl[0]); push(pl[1]);
        break;
    case 126: // SERIAL_CONTROL: baudrate, timeout, device, flags, count + data[70] into text[]
        push(rd_u32(pl + 0)); push(rd_u16(pl + 4)); push(pl[6]); push(pl[7]); push(pl[8]);
        std::memcpy(d.text, pl + 9, 70);
        break;
    case 246: // ADSB_VEHICLE: ICAO, lat, lon, altitude, heading, hor_vel, ver_vel,
              //               flags, squawk, altitude_type, emitter_type, tslc (+callsign)
        push(rd_u32(pl + 0)); push(rd_i32(pl + 4)); push(rd_i32(pl + 8)); push(rd_i32(pl + 12));
        push(rd_u16(pl + 16)); push(rd_u16(pl + 18)); push(rd_i16(pl + 20)); push(rd_u16(pl + 22));
        push(rd_u16(pl + 24)); push(pl[26]); push(pl[36]); push(pl[37]);
        std::memcpy(d.text, pl + 27, 9); d.text[9] = '\0';
        break;
    default: break;
    }
}

// ---- streaming parser -------------------------------------------------------
// CRC-gated resync: a frame is only consumed once its CRC validates, so the
// parser recovers cleanly from corruption or a false start-of-frame byte.
struct Parser {
    std::vector<uint8_t> buf;
    std::deque<Decoded>  q;
    unsigned long ok = 0, drop = 0;

    void feed(const uint8_t* data, size_t len) {
        buf.insert(buf.end(), data, data + len);
        parse();
        // Bound memory if we are swimming in junk with no valid frame.
        if (buf.size() > 8192) buf.erase(buf.begin(), buf.end() - 2048);
    }

    void parse() {
        const size_t n = buf.size();
        size_t pos = 0;
        // Earliest byte of a candidate frame we could not yet complete. A genuine
        // split frame must be preserved for the next feed; but a *false* start
        // byte with a bogus length must not block real frames sitting behind it,
        // so we keep scanning past it and only fall back to `first_incomplete`
        // if no valid frame turns up later in the buffer.
        long first_incomplete = -1;
        auto mark = [&](size_t p) { if (first_incomplete < 0) first_incomplete = long(p); };

        while (pos < n) {
            if (buf[pos] != 0xFE && buf[pos] != 0xFD) { ++pos; continue; }

            bool v2 = (buf[pos] == 0xFD);
            size_t hdr = v2 ? 10 : 6;

            if (pos + 1 >= n) { mark(pos); ++pos; continue; }   // need length byte
            uint8_t payload = buf[pos + 1];

            size_t sig = 0;
            if (v2) {
                if (pos + 2 >= n) { mark(pos); ++pos; continue; } // need incompat_flags
                if (buf[pos + 2] & 0x01) sig = 13;                // signed-frame trailer
            }

            size_t total = hdr + payload + 2 + sig;
            if (pos + total > n) { mark(pos); ++pos; continue; }  // frame not all here yet

            uint32_t msgid = v2
                ? (uint32_t(buf[pos + 7]) | (uint32_t(buf[pos + 8]) << 8) | (uint32_t(buf[pos + 9]) << 16))
                : buf[pos + 5];

            const MsgInfo* info = find_info(msgid);
            if (!info) {
                // Unknown msgid: no CRC seed to validate it, but the header gave us
                // the exact frame length. If a valid start byte sits right where this
                // frame ends (or it runs to the buffer edge), the framing is
                // self-consistent -- skip the whole message rather than byte-scanning
                // its payload (which used to spawn phantom frames counted as drops).
                if (pos + total >= n || buf[pos + total] == 0xFE || buf[pos + total] == 0xFD)
                    pos += total;
                else
                    ++pos;
                continue;
            }

            // CRC over [len .. end-of-payload] then the per-message extra seed.
            uint16_t crc = crc_buf(0xFFFF, &buf[pos + 1], (hdr - 1) + payload);
            crc = crc_buf(crc, &info->extra, 1);
            size_t crc_off = pos + hdr + payload;
            uint16_t got = uint16_t(buf[crc_off] | (buf[crc_off + 1] << 8));
            if (crc != got) { ++drop; ++pos; continue; }  // bad CRC: resync by 1

            Decoded d{};
            d.msgid = msgid;
            if (v2) { d.seq = buf[pos + 4]; d.sysid = buf[pos + 5]; d.compid = buf[pos + 6]; }
            else    { d.seq = buf[pos + 2]; d.sysid = buf[pos + 3]; d.compid = buf[pos + 4]; }

            uint8_t pad[256];
            std::memset(pad, 0, sizeof(pad));
            size_t copyn = payload < info->len ? payload : info->len;
            std::memcpy(pad, &buf[pos + hdr], copyn);
            decode(msgid, pad, d);

            q.push_back(d);
            ++ok;
            pos += total;
            first_incomplete = -1;   // bytes before a valid frame are settled junk
        }

        // Drop everything we are sure about; keep only a trailing potential partial.
        size_t keep_from = (first_incomplete >= 0) ? size_t(first_incomplete) : pos;
        if (keep_from > 0) buf.erase(buf.begin(), buf.begin() + keep_from);
    }
};

// ---- frame builder ----------------------------------------------------------
int build_v1(uint8_t* out, int cap, uint8_t seq, uint8_t sysid, uint8_t compid,
             uint32_t msgid, const uint8_t* payload, uint8_t plen, uint8_t extra) {
    int total = 8 + plen;
    if (cap < total) return -1;
    out[0] = 0xFE; out[1] = plen; out[2] = seq; out[3] = sysid; out[4] = compid;
    out[5] = uint8_t(msgid);
    std::memcpy(out + 6, payload, plen);
    uint16_t crc = crc_buf(0xFFFF, out + 1, 5 + plen);
    crc = crc_buf(crc, &extra, 1);
    out[6 + plen] = uint8_t(crc & 0xff);
    out[7 + plen] = uint8_t((crc >> 8) & 0xff);
    return total;
}

} // namespace

// ============================================================================
//  extern "C" ABI -- the surface ctypes binds to (app/core.py).
// ============================================================================
extern "C" {

void* mav_new()             { return new Parser(); }
void  mav_free(void* p)     { delete static_cast<Parser*>(p); }

// Returns the number of decoded messages waiting after ingesting `len` bytes.
int mav_feed(void* p, const uint8_t* data, size_t len) {
    auto* P = static_cast<Parser*>(p);
    P->feed(data, len);
    return int(P->q.size());
}

// Pops one decoded message; returns 1 if `out` was filled, else 0.
int mav_pop(void* p, Decoded* out) {
    auto* P = static_cast<Parser*>(p);
    if (P->q.empty()) return 0;
    *out = P->q.front();
    P->q.pop_front();
    return 1;
}

unsigned long mav_count_ok(void* p)   { return static_cast<Parser*>(p)->ok; }
unsigned long mav_count_drop(void* p) { return static_cast<Parser*>(p)->drop; }

// CRC primitives (used by tests and the simulator's Python encoder).
uint16_t mav_crc(const uint8_t* data, size_t len) {
    return crc_buf(0xFFFF, data, len);
}
uint16_t mav_crc_extra(const uint8_t* data, size_t len, uint8_t extra) {
    uint16_t c = crc_buf(0xFFFF, data, len);
    return crc_buf(c, &extra, 1);
}

// GCS-side encoders.
int mav_encode_heartbeat(uint8_t sysid, uint8_t compid, uint8_t seq, uint8_t* out, int cap) {
    uint8_t pl[9];
    std::memset(pl, 0, sizeof(pl));
    // custom_mode = 0 (pl[0..3]); type=MAV_TYPE_GCS(6); autopilot=INVALID(8);
    // base_mode=0; system_status=MAV_STATE_ACTIVE(4); mavlink_version=3.
    pl[4] = 6; pl[5] = 8; pl[6] = 0; pl[7] = 4; pl[8] = 3;
    return build_v1(out, cap, seq, sysid, compid, 0, pl, 9, 50);
}

int mav_encode_command_long(uint8_t sysid, uint8_t compid, uint8_t seq,
                            uint8_t tgt_sys, uint8_t tgt_comp, uint16_t command,
                            const float* params7, uint8_t* out, int cap) {
    uint8_t pl[33];
    std::memset(pl, 0, sizeof(pl));
    for (int i = 0; i < 7; ++i) std::memcpy(pl + i * 4, &params7[i], 4);
    pl[28] = uint8_t(command & 0xff);
    pl[29] = uint8_t((command >> 8) & 0xff);
    pl[30] = tgt_sys;
    pl[31] = tgt_comp;
    pl[32] = 0; // confirmation
    return build_v1(out, cap, seq, sysid, compid, 76, pl, 33, 152);
}

int mav_encode_command_int(uint8_t sysid, uint8_t compid, uint8_t seq,
                           uint8_t tgt_sys, uint8_t tgt_comp, uint8_t frame,
                           uint16_t command, const float* params4,
                           int32_t x, int32_t y, float z, uint8_t* out, int cap) {
    uint8_t pl[35];
    std::memset(pl, 0, sizeof(pl));
    for (int i = 0; i < 4; ++i) std::memcpy(pl + i * 4, &params4[i], 4);
    std::memcpy(pl + 16, &x, 4);
    std::memcpy(pl + 20, &y, 4);
    std::memcpy(pl + 24, &z, 4);
    pl[28] = uint8_t(command & 0xff);
    pl[29] = uint8_t((command >> 8) & 0xff);
    pl[30] = tgt_sys;
    pl[31] = tgt_comp;
    pl[32] = frame;
    pl[33] = 0; // current
    pl[34] = 0; // autocontinue
    return build_v1(out, cap, seq, sysid, compid, 75, pl, 35, 158);
}

// Reference CRC in portable C++ for the build-time asm self-test.
uint16_t mav_crc_ref(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        uint8_t tmp = uint8_t(data[i] ^ uint8_t(crc & 0xff));
        tmp ^= uint8_t(tmp << 4);
        crc = uint16_t((crc >> 8) ^ (uint16_t(tmp) << 8) ^ (uint16_t(tmp) << 3) ^ (uint16_t(tmp) >> 4));
    }
    return crc;
}

} // extern "C"
