#include "trace.h"
#include "utils/log.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
#include <dirent.h>

namespace {
enum Kind : uint8_t { kOther = 0, kJumpIndirect = 1, kCallIndirect = 2, kCallDirect = 3, kRts = 4, kRtx = 5, kIdle = 6 };
const char* KindName(uint8_t k) {
    switch (k) {
        case kJumpIndirect: return "jump-indirect";
        case kCallIndirect: return "call-indirect";
        case kCallDirect: return "call";
        case kRts: return "rts";
        case kRtx: return "rti";
        case kIdle: return "idle";
        default: return "other";
    }
}
}  // namespace

Tracer* Tracer::Create(Disasm disasm) {
    const char* dir = std::getenv("OP1EMU_TRACE_DIR");
    if (!dir || !*dir) return nullptr;
    return new Tracer(dir, std::move(disasm));
}

Tracer::Tracer(const std::string& dir, Disasm disasm) : dir_(dir), disasm_(std::move(disasm)) {
    mkdir(dir_.c_str(), 0755);
    if (DIR* d = opendir(dir_.c_str())) {   // a watchdog reset makes a new machine: keep the earlier phases
        while (dirent* e = readdir(d))
            if (e->d_name[0] >= '0' && e->d_name[0] <= '9' && e->d_name[1] >= '0' && e->d_name[1] <= '9' && e->d_name[2] == '_')
                firstIndex_ = std::max(firstIndex_, (e->d_name[0] - '0') * 10 + (e->d_name[1] - '0') + 1);
        closedir(d);
    }
    phaseIndex_ = firstIndex_;
    if (const char* c = std::getenv("OP1EMU_TRACE_ORDERED")) orderedCap_ = std::strtoull(c, nullptr, 10);
    if (const char* c = std::getenv("OP1EMU_TRACE_PER_SITE")) orderedPerSite_ = (uint32_t)std::strtoul(c, nullptr, 10);
    ordered_.reserve(std::min<size_t>(orderedCap_, 1 << 20));
    phaseName_ = "boot-rom";
    LogInfo("trace: writing aggregated trace to %s", dir_.c_str());
}

void Tracer::RequestPhase(const std::string& name) {
    std::lock_guard<std::mutex> lock(reqMu_);
    reqName_ = name;
    reqPending_.store(true, std::memory_order_release);
}

void Tracer::Poll(uint32_t pc) {
    if (pc == 0xFFA00000 && ffaHits_ < 2) NewPhase("ffa00000-" + std::to_string(++ffaHits_));
    if (reqPending_.load(std::memory_order_acquire)) {
        std::string name;
        {
            std::lock_guard<std::mutex> lock(reqMu_);
            name = reqName_;
            reqPending_.store(false, std::memory_order_relaxed);
        }
        NewPhase(name);
    }
}

void Tracer::NewPhase(const std::string& name) {
    WritePhase();
    blocks_.clear(); insnPc_.clear(); transfers_.clear(); transferKind_.clear(); mmr_.clear(); irq_.clear();
    ++phaseIndex_;
    phaseName_ = name;
    phaseStartCycles_ = curCycles_;
    phaseBlocks_ = 0;
    LogInfo("trace: phase %d '%s' at guest cycle %llu", phaseIndex_, name.c_str(), (unsigned long long)curCycles_);
}

uint8_t Tracer::Classify(const std::string& raw) {
    std::string t;
    for (char c : raw) t += (char)std::tolower((unsigned char)c);
    while (!t.empty() && std::isspace((unsigned char)t[0])) t.erase(0, 1);
    if (t.rfind("jump (", 0) == 0) return kJumpIndirect;
    if (t.rfind("call (", 0) == 0) return kCallIndirect;
    if (t.rfind("call ", 0) == 0) return kCallDirect;
    if (t == "rts") return kRts;
    if (t == "rti" || t == "rtx" || t == "rtn" || t == "rte") return kRtx;
    if (t.rfind("idle", 0) == 0) return kIdle;
    return kOther;
}

uint32_t Tracer::InsnPc(uint32_t start, uint32_t packet) {
    if (packet <= 1) return start;
    const uint64_t key = (uint64_t)start << 32 | packet;
    auto it = insnPc_.find(key);
    if (it != insnPc_.end()) return it->second;
    uint32_t pc = start;
    for (uint32_t i = 1; i < packet; ++i) {
        const uint32_t next = disasm_(pc).second;
        if (next <= pc) break;
        pc = next;
    }
    insnPc_.emplace(key, pc);
    return pc;
}

void Tracer::EndBlock(uint32_t packets, uint32_t nextPc) {
    ++phaseBlocks_;
    if (!pendingSites_.empty() || orderedResolved_ < ordered_.size()) ResolvePending();
    if (!packets) return;
    const uint64_t key = (uint64_t)curStart_ << 32 | packets;
    auto [it, inserted] = blocks_.try_emplace(key);
    BlockStat& b = it->second;
    if (inserted) {
        b.lastPc = InsnPc(curStart_, packets);
        auto [text, next] = disasm_(b.lastPc);
        b.end = next;
        b.kind = Classify(text);
    }
    ++b.count;
    if (b.kind == kJumpIndirect || b.kind == kCallIndirect || b.kind == kCallDirect) {
        const uint64_t t = (uint64_t)b.lastPc << 32 | nextPc;
        ++transfers_[t];
        transferKind_[t] = b.kind;
    }
}

void Tracer::ResolvePending() {
    for (const auto& p : pendingSites_) {
        const uint32_t pc = InsnPc(p.start, p.packet);
        MmrStat& m = mmr_[p.mmrKey];
        if (std::find(m.sites.begin(), m.sites.end(), pc) == m.sites.end()) {
            if (m.sites.size() < 64) m.sites.push_back(pc); else m.sitesOverflow = true;
        }
    }
    pendingSites_.clear();
    for (; orderedResolved_ < ordered_.size(); ++orderedResolved_) {
        Ordered& o = ordered_[orderedResolved_];
        o.insnPc = InsnPc(o.start, o.packet);
    }
}

void Tracer::CheckIpend(uint32_t ipend, uint32_t pc) {
    const uint32_t fresh = ipend & ~lastIpend_ & ~(1u << 4);   // bit 4 = global interrupt disable, not an IVG
    lastIpend_ = ipend;
    if (!fresh) return;
    const unsigned ivg = (unsigned)__builtin_ctz(fresh);
    ++irq_[(uint64_t)ivg << 32 | pc];
}

void Tracer::OnMmr(uint32_t addr, bool write, unsigned width, uint32_t value, uint32_t packet) {
    const uint64_t key = (uint64_t)addr | (uint64_t)write << 32 | (uint64_t)width << 33;
    MmrStat& m = mmr_[key];
    ++m.count;
    if (m.values.size() < 64) {
        if (std::find(m.values.begin(), m.values.end(), value) == m.values.end()) m.values.push_back(value);
    } else if (!m.valuesOverflow && std::find(m.values.begin(), m.values.end(), value) == m.values.end()) {
        m.valuesOverflow = true;
    }
    if (!m.sitesOverflow) {
        auto it = insnPc_.find((uint64_t)curStart_ << 32 | packet);
        if (packet <= 1 || it != insnPc_.end()) {
            const uint32_t pc = packet <= 1 ? curStart_ : it->second;
            if (std::find(m.sites.begin(), m.sites.end(), pc) == m.sites.end()) {
                if (m.sites.size() < 64) m.sites.push_back(pc); else m.sitesOverflow = true;
            }
        } else {
            pendingSites_.push_back({key, curStart_, packet});
        }
    }
    if (write) {
        const uint64_t site = ((uint64_t)curStart_ * 0x9E3779B97F4A7C15ull) ^ ((uint64_t)addr << 8) ^ packet;
        if (++orderedSeen_[site] > orderedPerSite_) ++orderedSkipped_;
        else if (ordered_.size() < orderedCap_)
            ordered_.push_back({addr, value, curStart_, packet, curCycles_, (uint8_t)width, 0});
        else
            ++orderedDropped_;
    }
}

void Tracer::WritePhase() {
    ResolvePending();
    char name[512];
    snprintf(name, sizeof name, "%s/%02d_%s", dir_.c_str(), phaseIndex_, phaseName_.c_str());
    const std::string d = name;
    mkdir(d.c_str(), 0755);
    if (FILE* f = fopen((d + "/meta.tsv").c_str(), "w")) {
        fprintf(f, "phase\t%s\nindex\t%d\nstart_cycles\t%llu\nend_cycles\t%llu\nblocks_run\t%llu\n", phaseName_.c_str(),
                phaseIndex_, (unsigned long long)phaseStartCycles_, (unsigned long long)curCycles_,
                (unsigned long long)phaseBlocks_);
        fclose(f);
    }
    if (FILE* f = fopen((d + "/blocks.tsv").c_str(), "w")) {
        fprintf(f, "start\tend\tpackets\tcount\tlast_pc\tlast_kind\n");
        for (const auto& [k, b] : blocks_)
            fprintf(f, "%08x\t%08x\t%u\t%llu\t%08x\t%s\n", (uint32_t)(k >> 32), b.end, (uint32_t)k,
                    (unsigned long long)b.count, b.lastPc, KindName(b.kind));
        fclose(f);
    }
    if (FILE* f = fopen((d + "/transfers.tsv").c_str(), "w")) {
        fprintf(f, "kind\tsource\ttarget\tcount\n");
        for (const auto& [k, n] : transfers_)
            fprintf(f, "%s\t%08x\t%08x\t%llu\n", KindName(transferKind_[k]), (uint32_t)(k >> 32), (uint32_t)k,
                    (unsigned long long)n);
        fclose(f);
    }
    if (FILE* f = fopen((d + "/mmr.tsv").c_str(), "w")) {
        fprintf(f, "addr\trw\twidth\tcount\tvalues\tsites\n");
        for (const auto& [k, m] : mmr_) {
            fprintf(f, "%08x\t%s\t%u\t%llu\t", (uint32_t)k, (k >> 32) & 1 ? "W" : "R", (unsigned)(k >> 33),
                    (unsigned long long)m.count);
            for (size_t i = 0; i < m.values.size(); ++i) fprintf(f, "%s%x", i ? "," : "", m.values[i]);
            fprintf(f, "%s\t", m.valuesOverflow ? ",+" : "");
            for (size_t i = 0; i < m.sites.size(); ++i) fprintf(f, "%s%08x", i ? "," : "", m.sites[i]);
            fprintf(f, "%s\n", m.sitesOverflow ? ",+" : "");
        }
        fclose(f);
    }
    if (FILE* f = fopen((d + "/irq.tsv").c_str(), "w")) {
        fprintf(f, "ivg\thandler\tcount\n");
        for (const auto& [k, n] : irq_) fprintf(f, "%u\t%08x\t%llu\n", (unsigned)(k >> 32), (uint32_t)k, (unsigned long long)n);
        fclose(f);
    }
}

void Tracer::Finish() {
    WritePhase();
    const std::string ordered = firstIndex_ ? "/ordered_mmr_writes_" + std::to_string(firstIndex_) + ".tsv" : "/ordered_mmr_writes.tsv";
    if (FILE* f = fopen((dir_ + ordered).c_str(), "w")) {
        fprintf(f, "# first %zu guest MMR writes in order; at most %u per (address, instruction), %llu more skipped "
                "that way; dropped after the cap of %zu: %llu\n", ordered_.size(), orderedPerSite_,
                (unsigned long long)orderedSkipped_, orderedCap_, (unsigned long long)orderedDropped_);
        fprintf(f, "cycles\taddr\twidth\tvalue\tinsn_pc\tblock\n");
        for (const auto& o : ordered_)
            fprintf(f, "%llu\t%08x\t%u\t%x\t%08x\t%08x\n", (unsigned long long)o.cycles, o.addr, o.width, o.value,
                    o.insnPc, o.start);
        fclose(f);
    }
    LogInfo("trace: phases %d..%d written to %s", firstIndex_, phaseIndex_, dir_.c_str());
}
