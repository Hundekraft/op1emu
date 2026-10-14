#include "memmodel.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
// SDRAM and bus timing, in SCLK cycles (R246: SDGCTL 0xC49111C9, SDBCTL 0x0025; docs/firmware/clocks-memory.md)
constexpr uint64_t kRatio = 5;       // CCLK 400 MHz / SCLK 80 MHz
constexpr uint64_t kCL = 2, kRCD = 2, kRP = 2, kWR = 2;
constexpr uint64_t kOverhead = 3;    // core -> EBIU request and return, assumed
constexpr uint64_t kLineBeats = 16;  // 32-byte line on the 16-bit bus
constexpr uint64_t kWordBeats = 4;   // the critical 64-bit word arrives first
constexpr uint64_t kWriteSlack = 8 * kRatio;   // write buffer: the core stalls only behind a longer backlog

std::vector<uint32_t> ParsePcs(const char* s)
{
    std::vector<uint32_t> out;
    for (const char* q = s; q && *q;) {
        char* end = nullptr;
        out.push_back((uint32_t)strtoul(q, &end, 16));
        if (end == q) break;
        q = *end == '+' ? end + 1 : end;
        if (*end != '+') break;
    }
    return out;
}
}  // namespace

int MemModel::Cache::Set(uint32_t line) const
{
    // line = addr >> 5; data cache: sub-bank bits 13:12 and line bits 10:5 (256 sets); instruction cache:
    // sub-bank bits 13:12 and line bits 9:5 (128 sets)
    return sets == 256 ? (int)((line & 63) | (((line >> 7) & 3) << 6)) : (int)((line & 31) | (((line >> 7) & 3) << 5));
}

void MemModel::Cache::Init(int s, int w)
{
    sets = s;
    ways = w;
    tag.assign((size_t)s * w, 0);
    dirty.assign((size_t)s * w, 0);
    used.assign((size_t)s * w, 0);
}

void MemModel::Cache::Flush()
{
    std::fill(tag.begin(), tag.end(), 0);
    std::fill(dirty.begin(), dirty.end(), 0);
}

MemModel::MemModel(const std::string& path) : path_(path)
{
    for (int cold = 0; cold < 2; ++cold)
        for (int walloc = 0; walloc < 2; ++walloc) {
            Variant v;
            v.cold = cold;
            v.walloc = walloc;
            v.d.Init(256, 2);
            v.i.Init(128, 4);
            variants_.push_back(std::move(v));
        }
    if (const char* s = getenv("OP1EMU_MEMMODEL_L1")) {   // LO-HI[+LO-HI...]: count as L1 (what-if layouts)
        for (const char* q = s; *q;) {
            char* end = nullptr;
            const uint32_t lo = (uint32_t)strtoul(q, &end, 16);
            if (*end != '-') break;
            const uint32_t hi = (uint32_t)strtoul(end + 1, &end, 16);
            l1_.emplace_back(lo, hi);
            if (*end != '+') break;
            q = end + 1;
        }
    }
    flushPcs_ = ParsePcs(getenv("OP1EMU_MEMMODEL_FLUSH"));
    if (const char* s = getenv("OP1EMU_MEMMODEL_REMAP")) {   // LO-HI:TO[+...]: model [LO, HI) as if it sat at TO
        for (const char* q = s; *q;) {
            char* end = nullptr;
            Remap r;
            r.lo = (uint32_t)strtoul(q, &end, 16);
            if (*end != '-') break;
            r.hi = (uint32_t)strtoul(end + 1, &end, 16);
            if (*end != ':') break;
            r.to = (uint32_t)strtoul(end + 1, &end, 16);
            remap_.push_back(r);
            if (*end != '+') break;
            q = end + 1;
        }
    }
}

MemModel::~MemModel()
{
    FILE* f = fopen(path_.c_str(), "w");
    if (!f) return;
    fprintf(f, "# op1emu memory model: per block start pc and variant; stall in CCLK cycles\n");
    fprintf(f, "# variant: warm|cold - noalloc|walloc\n# L1 what-if ranges:");
    for (const auto& r : l1_) fprintf(f, " %08x-%08x", r.first, r.second);
    fprintf(f, "\n# remap:");
    for (const auto& r : remap_) fprintf(f, " %08x-%08x:%08x", r.lo, r.hi, r.to);
    fprintf(f, "\n# pc variant insns imiss rmiss wmiss victims stall\n");
    for (const auto& v : variants_) {
        const char* name = v.cold ? (v.walloc ? "cold-walloc" : "cold-noalloc") : (v.walloc ? "warm-walloc" : "warm-noalloc");
        std::vector<std::pair<uint32_t, Counters>> rows(v.perPc.begin(), v.perPc.end());
        std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [pc, n] : rows)
            fprintf(f, "%08x %s %llu %llu %llu %llu %llu %llu\n", pc, name, (unsigned long long)n.insns,
                    (unsigned long long)n.imiss, (unsigned long long)n.rmiss, (unsigned long long)n.wmiss,
                    (unsigned long long)n.victims, (unsigned long long)n.stall);
    }
    fclose(f);
}

MemModel::Region MemModel::Classify(uint32_t addr) const
{
    if (addr >= 0x08000000u) return kFree;   // L1, MMRs, boot ROM, async banks: not modelled as SDRAM
    for (const auto& r : l1_)
        if (addr >= r.first && addr < r.second) return kFree;
    return addr < 0x02000000u ? kCachedWB : kCachedWT;
}

void MemModel::BeginBlock(uint32_t pc, uint64_t guestCycles)
{
    blockPc_ = pc;
    blockCycles_ = guestCycles;
    index_ = 0;
    for (auto& v : variants_) v.cur = &v.perPc[pc];
    if (std::find(flushPcs_.begin(), flushPcs_.end(), pc) != flushPcs_.end())
        for (auto& v : variants_)
            if (v.cold) {
                v.d.Flush();
                v.i.Flush();
            }
}

// One SDRAM transfer: returns when the first data word arrives (read) or when the write is on the bus.
uint64_t MemModel::Bus(Variant& v, uint32_t addr, uint64_t start, unsigned beats, bool write)
{
    const uint64_t t = std::max(start, v.busFree);
    const unsigned bank = (addr >> 24) & 3;     // 64 MB, 10-bit columns: internal bank = bits 25:24
    const uint32_t row = addr >> 11;            // 2 KB rows
    uint64_t lead = kOverhead;
    if (v.openRow[bank] != row) {
        lead += (v.openRow[bank] == ~0u ? 0 : kRP) + kRCD;
        v.openRow[bank] = row;
    }
    if (!write) lead += kCL;
    v.busFree = t + (lead + beats + (write ? kWR : 0)) * kRatio;
    return t + (lead + (write ? 0 : std::min<uint64_t>(beats, kWordBeats))) * kRatio;
}

void MemModel::Fill(Variant& v, Cache& c, uint32_t addr, bool dirty, Counters& n)
{
    const uint32_t line = addr >> 5;
    const int set = c.Set(line);
    int victim = 0;
    for (int w = 1; w < c.ways; ++w)
        if (c.used[set * c.ways + w] < c.used[set * c.ways + victim]) victim = w;
    const size_t e = (size_t)set * c.ways + victim;
    const uint64_t now = Now(v);
    const uint64_t ready = Bus(v, addr, now, kLineBeats, false);
    if (ready > now) {
        v.stall += ready - now;
        n.stall += ready - now;
    }
    if (c.tag[e] && c.dirty[e]) {               // copy-back through the victim buffer, after the fill
        ++n.victims;
        Bus(v, (c.tag[e] - 1) << 5, Now(v), kLineBeats, true);
    }
    c.tag[e] = line + 1;
    c.dirty[e] = dirty;
    c.used[e] = ++c.stamp;
}

void MemModel::Access(uint32_t addr, uint32_t len, int kind)
{
    if (kind == 2) ++index_;
    for (const auto& r : remap_)
        if (addr >= r.lo && addr < r.hi) { addr = addr - r.lo + r.to; break; }
    const Region region = Classify(addr);
    for (auto& v : variants_) {
        Counters& n = *v.cur;
        if (kind == 2) ++n.insns;
        if (region == kFree) continue;
        Cache& c = kind == 2 ? v.i : v.d;
        const uint32_t line = addr >> 5;
        const int set = c.Set(line);
        int hit = -1;
        for (int w = 0; w < c.ways; ++w)
            if (c.tag[set * c.ways + w] == line + 1) hit = w;
        if (hit >= 0) {
            const size_t e = (size_t)set * c.ways + hit;
            c.used[e] = ++c.stamp;
            if (kind == 1) {
                if (region == kCachedWB) c.dirty[e] = 1;
                else {                           // write-through: also to SDRAM through the write buffer
                    const uint64_t now = Now(v), on = Bus(v, addr, now, len > 2 ? 2 : 1, true);
                    if (on > now + kWriteSlack) { v.stall += on - now - kWriteSlack; n.stall += on - now - kWriteSlack; }
                }
            }
            continue;
        }
        if (kind == 2) { ++n.imiss; Fill(v, c, addr, false, n); continue; }
        if (kind == 0) { ++n.rmiss; Fill(v, c, addr, false, n); continue; }
        ++n.wmiss;
        if (v.walloc && region == kCachedWB) { Fill(v, c, addr, true, n); continue; }
        const uint64_t now = Now(v), on = Bus(v, addr, now, len > 2 ? 2 : 1, true);
        if (on > now + kWriteSlack) { v.stall += on - now - kWriteSlack; n.stall += on - now - kWriteSlack; }
    }
}
