/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2019 EPFL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Jaidev Patwardhan
 *          Korey Sewell
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#ifndef __ARCH_RISCV_TLB_HH__
#define __ARCH_RISCV_TLB_HH__

#include <unordered_map>

#include "arch/generic/tlb.hh"
#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/table_walker.hh"
#include "arch/riscv/utility.hh"
#include "arch/riscv/vtophys.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/RiscvTLB.hh"
#include "sim/sim_object.hh"

class ThreadContext;

/* To maintain compatibility with other architectures, we'll
   simply create an ITLB and DTLB that will point to the real TLB */
namespace RiscvISA {

struct TlbEntry : public Serializable
{
    Addr va;
    Addr pa;
    Addr pte;
    int asid;

    TlbEntry() {}
    TlbEntry(Addr va, Addr pa, Addr pte, int asid);
    TlbEntry(TlbEntry * te);
    void invalidate();
    bool match(Addr vpn, int asn, bool ignore_asn);
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

class TableWalker;
class TLB : public BaseTLB
{
  protected:
    // The (local) Page Table
    std::unordered_map<Addr, RiscvISA::TlbEntry> unordTable;

    int size;                   // Max TLB Size
    TableWalker * tableWalker;  // TLB PT walker
    int rangeMRU;               // For lookup

    mutable Stats::Scalar read_hits;
    mutable Stats::Scalar read_misses;
    mutable Stats::Scalar read_acv;
    mutable Stats::Scalar read_accesses;
    mutable Stats::Scalar write_hits;
    mutable Stats::Scalar write_misses;
    mutable Stats::Scalar write_acv;
    mutable Stats::Scalar write_accesses;
    Stats::Formula hits;
    Stats::Formula misses;
    Stats::Formula accesses;

  public:
    typedef RiscvTLBParams Params;
    TLB(const Params * p);
    virtual ~TLB();
    void init() override;
    Fault translateFunctional(const RequestPtr &req, ThreadContext * tc,
            Mode mode) override;
    TableWalker * getTableWalker();
    int getsize() const;
    void flushAll() override;
    void flush(Addr va, int asn);
    void demapPage(Addr va, uint64_t asn) override;
    TlbEntry * lookup(Addr va, int asn) const;
    void insert(Addr va, Addr pa, Addr pte, int asid);
    Fault translateFs(const RequestPtr &req, ThreadContext *tc, Mode mode,
            bool &delay, bool timing, bool functional = false);
    Fault translateSe(const RequestPtr &req, ThreadContext *tc, Mode mode,
            bool &delay, bool timing);
    Fault translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode)
        override;
    void translateTiming(const RequestPtr &req, ThreadContext *tc,
        Translation *translation, Mode mode) override;
    Fault finalizePhysical(const RequestPtr &req, ThreadContext *tc, Mode mode)
        const override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    void regStats() override;
    void takeOverFrom(BaseTLB * tlb);

private:
    Fault determinePageFault(Mode mode, Addr addr);
    bool violatesPMAPMPCheck(ThreadContext * tc, Addr addr, Mode mode);
    void flushWorker(Addr va, bool useVa, int asid, bool useAsid);
    void incrementTlbStats(Mode mode, bool isHit);
};

}

#endif // __RISCV_MEMORY_HH__
