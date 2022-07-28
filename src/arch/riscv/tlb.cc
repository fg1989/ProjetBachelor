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
 *          Zhengxing Li
 *          Deyuan Guo
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#include "arch/riscv/tlb.hh"

#include <vector>

#include "arch/generic/mmapped_ipr.hh"
#include "arch/riscv/faults.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/pra_constants.hh"
#include "arch/riscv/registers.hh"
#include "arch/riscv/system.hh"
#include "arch/riscv/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/RiscvTLB.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "params/RiscvTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

using namespace std;
using namespace RiscvISA;

TlbEntry::TlbEntry(Addr va, Addr pa, Addr pte, int asid)
{
    this->va = va;
    this->pa = pa;
    this->pte = pte;
    this->asid = asid;
}

TlbEntry::TlbEntry(TlbEntry * te)
{
    this->va = te->va;
    this->pa = te->pa;
    this->pte = te->pte;
    this->asid = te->asid;
}

void
TlbEntry::invalidate()
{
    va = 0;
    pa = 0;
    pte = 0;
    asid = 0;
}

bool
TlbEntry::match(Addr vpn, int asn, bool ignore_asn)
{
    return (ignore_asn || (asn == asid)) && (vpn == va) &&
        (pte & 0x1);
}

void
TlbEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(va);
    SERIALIZE_SCALAR(pa);
    SERIALIZE_SCALAR(pte);
    SERIALIZE_SCALAR(asid);
}

void
TlbEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(va);
    UNSERIALIZE_SCALAR(pa);
    UNSERIALIZE_SCALAR(pte);
    UNSERIALIZE_SCALAR(asid);
}

TLB::TLB(const Params *p)
    : BaseTLB(p)
{
    tableWalker = p->walker;
    unordTable.reserve(p->size);
    size = p->size;
    rangeMRU = 1;
    tableWalker->setupWalker(this, p->sys, p->sys->getMasterId(p->walker));
}

TLB::~TLB()
{
}

void
TLB::init()
{
    BaseTLB::init();
}

TableWalker *
TLB::getTableWalker()
{
    return tableWalker;
}

int
TLB::getsize() const
{
    return size;
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "TLB::flushAll\n");
    flushWorker(0, false, 0, false);
}

void
TLB::flush(Addr va, int asn)
{
    DPRINTF(TLB, "TLB::flush(va = %x, asn = %x)\n", va, asn);
    if (asn != -1 && va == 0) flushWorker(0, false, asn, true);
    else if (va != 0 && asn == -1) flushWorker(va, true, 0, false);
    else flushWorker(va, true, asn, true);
}

inline void
TLB::flushWorker(Addr va, bool useVa, int asn, bool useAsid)
{
    if (!useVa && !useAsid) {
        unordTable.clear();
    } else if (!useAsid) { // Clear virtual address
        if (unordTable.count(va) > 0) unordTable[va].invalidate();
    } else if (!useVa) { // Clear by ASN
        unordTable.clear();
        for (const auto & entry: unordTable) {
            if (unordTable[entry.first].asid == asn) {
                unordTable[entry.first].invalidate();
            }
        }
    }
}

void
TLB::demapPage(Addr va, uint64_t asn)
{
    flush(va, (int)asn);
}

TlbEntry *
TLB::lookup(Addr va, int asn) const
{
    // Currently unused
    DPRINTF(TLB, "TLB::lookup(va = %x, asn = %x) called.\n", va, asn);
    return NULL;
}

void
TLB::insert(Addr va, Addr pa, Addr pte, int asid)
{
    DPRINTF(TLB, "TLB::insert(va = %x, pa = %x, pte = %x) called.\n", va, pa,
        pte);
    unordTable[va] = TlbEntry(va, pa, pte, asid);

}

Fault
TLB::translateFs(const RequestPtr & req, ThreadContext * tc, Mode mode,
            bool &delay, bool timing, bool functional)
{
    SATP satp = tc->readMiscReg(MISCREG_SATP);
    Addr va = req->getVaddr();

    /**
     * check if we simulate a bare metal system
     * if so, we have no tlb, phys addr == virt addr
     */
    if (static_cast<RiscvSystem *>(tc->getSystemPtr())->isBareMetal())
        req->setFlags(Request::PHYSICAL);

    if ((req->getFlags() & Request::PHYSICAL) || satp.mode == 0 ||
        (PrivilegeMode)tc->readMiscReg(MISCREG_PRV) == PRV_M) {
        /**
         * we simply set the virtual address to physical address
         */
        DPRINTF(TLB, "translateFs interpretting address %x as physical.\n",
                req->getVaddr());
        req->setPaddr(va);

        if (violatesPMAPMPCheck(tc, va, mode)) {
            switch (mode) {
                case Read:
                    DPRINTF(TLB, "LOAD_ACCESS fault detected!\n");
                    return std::make_shared<AccessFault>(
                        AccessFault(LOAD_ACCESS));
                case Execute:
                    DPRINTF(TLB, "INST_ACCESS fault detected!\n");
                    return std::make_shared<AccessFault>(
                        AccessFault(INST_ACCESS));
                case Write:
                    DPRINTF(TLB, "STORE_ACCESS fault detected!\n");
                    return std::make_shared<AccessFault>(
                        AccessFault(STORE_ACCESS));
                default:
                    panic("PMP/PMA check failed in unknown mode.");
                    break;
            }
        }

        return NoFault;
    }
    // SATP mode = 8 == SV39 virtual addressing.
    // Similarly, mode = 1 == SV32 and mode = 9 == SV48 virtual addressing.
    else if (satp.mode == 8) {
        DPRINTF(TLB, "translateFs interpretting address %x as virtual.\n",
                req->getVaddr());

        // Implementing address translation as in section 4.3.2 of the
        // privileged RISC-V specification, v1.11, with the parameters of
        // section 4.4.1 as this is for RV64 (SV39).
        const uint64_t LEVELS = 3;
        const uint64_t PTESIZE = 8;
        const uint64_t PAGESIZE = 4096;
        Sv39VAddr vaFormatted = Sv39VAddr(va);
        Sv39PAddr pa = Sv39PAddr(0);
        STATUS status = tc->readMiscReg(MISCREG_STATUS);
        PrivilegeMode pp = (PrivilegeMode)tc->readMiscReg(MISCREG_PRV);
        PTE pte;

        // CTOAT == "corresponding to original access type"
        // Step 1
        uint64_t a = satp.ppn * PAGESIZE;
        int i = LEVELS - 1;

        while (42) {
            // Step 2
            // if (unordTable.count(va & 0xfffffffffffff000) > 0) {
            //     TlbEntry te = unordTable[va & 0xfffffffffffff000];
            //     if (te.pte & 0x1 && te.asid == satp.asid) {
            //         incrementTlbStats(mode, true);
            //         DPRINTF(TLB, "PTE lookup of %x (va = %x; a = %x) yielded "
            //             "result: pte = %x.\n",
            //             a + vaFormatted.vpn[i] * PTESIZE,
            //             va & 0xfffffffffffff000, a, te.pte);

            //         // Set physical address and return NoFault if PTE valid
            //         req->setPaddr(te.pa | vaFormatted.page_offset);
            //         DPRINTF(TLB, "Translating vaddr %x to paddr %x.\n",
            //                 req->getVaddr(), req->getPaddr());
            //         return NoFault;
            //     }
            // }

            // TLB miss
            incrementTlbStats(mode, false);
            DPRINTF(TLB, "PTE lookup of %x (va = %x; a = %x) yielded "
                "null result.\n",
                a + vaFormatted.vpn[i] * PTESIZE, va & 0xfffffffffffff000, a);

            // Need to walk page table to get new PTE
            Addr walkerResult;
            Fault tableWalkerFault = tableWalker->walk(req, tc, satp.asid,
                mode, timing, functional, a + vaFormatted.vpn[i] * PTESIZE,
                req->getFlags(), walkerResult);

            DPRINTF(TLB, "Walker yielded result of %x.\n", walkerResult);
            pte = PTE(walkerResult);

            if (violatesPMAPMPCheck(tc, pte.insertAddr, mode) && pte.V) {
                switch (mode) {
                    case Read:
                        DPRINTF(TLB, "LOAD_ACCESS fault detected!\n");
                        return std::make_shared<AccessFault>(
                            AccessFault(LOAD_ACCESS));
                    case Execute:
                        DPRINTF(TLB, "INST_ACCESS fault detected!\n");
                        return std::make_shared<AccessFault>(
                            AccessFault(INST_ACCESS));
                    case Write:
                        DPRINTF(TLB, "STORE_ACCESS fault detected!\n");
                        return std::make_shared<AccessFault>(
                            AccessFault(STORE_ACCESS));
                    default:
                        panic("PMP/PMA check failed in unknown mode.");
                        break;
                }
            }

            // Step 3
            if (pte.V == 0 || (pte.R == 0 && pte.W == 1)) {
                DPRINTF(TLB, "Page fault at step 3 of address translation.  "
                    "pte = %x, VRW = %d%d%d.\n", pte.getAddr(), pte.V, pte.R,
                    pte.W);
                return determinePageFault(mode, req->getVaddr());
            }

            // Step 4
            if (pte.R == 1 || pte.X == 1)
                break;

            i = i - 1;
            if (i < 0) {
                DPRINTF(TLB, "Page fault at step 4 of address translation.\n");
                return determinePageFault(mode, req->getVaddr());
            } else {
                a = (pte.ppnCombo) * PAGESIZE;
            }
        }

        // Step 5
        if ((status.mxr == 0 && pte.R != 1) ||
            (status.mxr == 1 && pte.R != 1 && pte.X != 1) ||
            (status.sum == 0 && pte.U == 1 && pp == PRV_S) ||
            (status.sum == 0 && pte.U == 1 && status.mprv == 1
                && status.mpp == PRV_S) ||
            (pte.U != 1 && pp == PRV_U) ||
            (mode == Read && pte.R == 0) ||
            (mode == Write && pte.W == 0) ||
            (mode == Execute && pte.X == 0)) {
            DPRINTF(TLB, "Page fault at step 5 of address translation.\n");
            return determinePageFault(mode, req->getVaddr());
        }

        // Step 6
        for (int ii = 0; ii < i; ii++)
            if (pte.ppn[ii] != 0) {
                DPRINTF(TLB, "Page fault at step 6 of address translation.  "
                    "i = %x, pte = %x, pte.ppn[%d] = %x.\n", i, pte.getAddr(),
                    ii, pte.ppn[ii]);
                return determinePageFault(mode, req->getVaddr());
            }

        // Step 7 (option 1)
        if (pte.A == 0 || (mode == Write && pte.D == 0)) {
            DPRINTF(TLB, "Page fault at step 7 of address translation.\n");
            return determinePageFault(mode, req->getVaddr());
        }

        // Step 8
        pa.page_offset = vaFormatted.page_offset;
        if (i > 0)
            for (int ii = 0; ii < i; ii++)
                pa.ppn[ii] = vaFormatted.vpn[ii];
        for (int ii = i; ii < LEVELS; ii++)
            pa.ppn[ii] = pte.ppn[ii];

        // Insert new PTE into TLB
        insert(va & 0xfffffffffffff000, pa.formatAddr() & 0xfffffffffffff000,
            pte.getAddr(), pte.asid);

        // Set physical address and return without fault
        req->setPaddr(pa.formatAddr());
        DPRINTF(TLB, "Translating vaddr %x to paddr %x.\n",
                req->getVaddr(), req->getPaddr());
        return NoFault;
    } else {
        panic("Only Sv39 paging supported.  You're either not using a \
            39-bit virtual memory system or the satp register is set \
            incorrectly.");
        return NoFault;
    }
}

Fault
TLB::translateSe(const RequestPtr & req, ThreadContext * tc, Mode mode,
            bool &delay, bool timing)
{
    if (mode != Execute) {
        // bool write = (mode == Write); // Passed as arg for translateData

        // In the O3 CPU model, sometimes a memory access will be
        // speculatively executed along a branch that will end up not being
        // taken where the address is invalid.  In that case, return a
        // fault rather than trying to translate it (which will cause a
        // panic).  Since RISC-V allows unaligned memory accesses, this
        // should only happen if the request's length is long enough to
        // wrap around from the end of the memory to the start.
        assert(req->getSize() > 0);
        if (req->getVaddr() + req->getSize() - 1 < req->getVaddr())
            return make_shared<GenericPageTableFault>(req->getVaddr());
    }
    Process * p = tc->getProcessPtr();
    Fault fault = p->pTable->translate(req);
    return (fault != NoFault) ? fault : NoFault;
}

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode)
{
    DPRINTF(TLB,
            "translateAtomic called in mode %x to translate address %x.\n",
            mode, req->getVaddr());
    bool delay = false;
    Fault fault = FullSystem ? translateFs(req, tc, mode, delay, false,
        false) : translateSe(req, tc, mode, delay, false);
    assert(!delay);
    return fault;
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc, Mode mode)
{
    DPRINTF(TLB,
            "translateFunctional called in mode %x to translate address %x.\n",
            mode, req->getVaddr());
    bool delay = false;
    Fault fault = FullSystem ? translateFs(req, tc, mode, delay, false,
        true) : translateSe(req, tc, mode, delay, false);
    assert(!delay);
    return fault;
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    DPRINTF(TLB,
            "translateTiming called in mode %x to translate address %x.\n",
            mode, req->getVaddr());
    assert(translation);
    bool delay = false;
    Fault fault = FullSystem ? translateFs(req, tc, mode, delay, false,
        true) : translateSe(req, tc, mode, delay, false);
    assert(!delay);

    translation->finish(fault, req, tc, mode);
}

Fault
TLB::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, Mode mode) const
{
    return NoFault;
}

void
TLB::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(size);

    for (int i = 0; i < size; i++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", i));
        // TODO: Serialize unordTable
    }
}

void
TLB::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(size);

    for (int i = 0; i < size; i++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", i));
        // TODO: Unserialize unordTable
    }
}

void
TLB::regStats()
{
    BaseTLB::regStats();

    read_hits
        .name(name() + ".read_hits")
        .desc("DTB read hits")
        ;

    read_misses
        .name(name() + ".read_misses")
        .desc("DTB read misses")
        ;


    read_accesses
        .name(name() + ".read_accesses")
        .desc("DTB read accesses")
        ;

    write_hits
        .name(name() + ".write_hits")
        .desc("DTB write hits")
        ;

    write_misses
        .name(name() + ".write_misses")
        .desc("DTB write misses")
        ;


    write_accesses
        .name(name() + ".write_accesses")
        .desc("DTB write accesses")
        ;

    hits
        .name(name() + ".hits")
        .desc("DTB hits")
        ;

    misses
        .name(name() + ".misses")
        .desc("DTB misses")
        ;

    accesses
        .name(name() + ".accesses")
        .desc("DTB accesses")
        ;

    hits = read_hits + write_hits;
    misses = read_misses + write_misses;
    accesses = read_accesses + write_accesses;
}

void
TLB::takeOverFrom(BaseTLB * tlb)
{
    panic("TLB::takeOverFrom not implemented yet.\n");
}

bool
TLB::violatesPMAPMPCheck(ThreadContext * tc, Addr addr, Mode mode)
{
    // This method performs PMP/PMA checks according to Sections 3.5-3.6 of the
    // RISC-V privileged specification v1.11, pages 43-54, for RV64.  Note:
    // This doesn't conform to the RV32 flavor of ISA.
    // TODO: Create model for dedicated PMA/PMP checking.
    PrivilegeMode pp = (PrivilegeMode)tc->readMiscReg(MISCREG_PRV);
    uint64_t base = MISCREG_PMPADDR00;
    uint64_t cfg = tc->readMiscReg(MISCREG_PMPCFG0);
    uint64_t pmpAddr;
    uint8_t A;
    uint8_t pmpxcfg;

    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 8; i++, base++) {
            pmpAddr = 0x3fffffffffffff & tc->readMiscReg(base);
            pmpxcfg = cfg >> (8 * i);
            A = (pmpxcfg >> 0x3) & 0x3;

            if (A == 0) {
                // NULL (disabled) Region
                return false;
            } else if (A == 1 && ((i == 0 && j == 0) || addr <= pmpAddr)) {
                // Top of range (TOR)
                if ((mode == Read && pmpxcfg & 0x1) ||
                    (mode == Write && pmpxcfg & 0x2) ||
                    (mode == Execute && pmpxcfg & 0x4) ||
                    (pp == PRV_M && !(pmpxcfg >> 8)))
                    return false;
            } else { // A == 3
                // Naturally aligned 4B or power-of-two region (NA4/NAPOT)
                if ((addr & pmpAddr) == addr) {
                    if ((mode == Read && pmpxcfg & 0x1) ||
                    (mode == Write && pmpxcfg & 0x2) ||
                    (mode == Execute && pmpxcfg & 0x4) ||
                    (pp == PRV_M && !(pmpxcfg >> 8)))
                        return false;
                }
            }
        }

        // If we didn't reach the correct range within the first loop, now we
        // need to check the next set of pmp CSRs
        if (addr > pmpAddr) cfg = tc->readMiscReg(MISCREG_PMPCFG2);
        else break;
    }

    // We failed all opportunities to not violate PMP/PMA
    return true;
}

inline Fault
TLB::determinePageFault(Mode mode, Addr addr)
{
    // Helper function to select correct exception code and return associated
    // fault type.
    switch (mode) {
        case Execute:
            DPRINTF(TLB, "Page fault of type INST_PAGE detected!\n");
            return std::make_shared<AddressFault>(
                AddressFault(addr, INST_PAGE));
        case Write:
            DPRINTF(TLB, "Page fault of type STORE_PAGE detected!\n");
            return std::make_shared<AddressFault>(
                AddressFault(addr, STORE_PAGE));
        case Read:
            DPRINTF(TLB, "Page fault of type LOAD_PAGE detected!\n");
            return std::make_shared<AddressFault>(
                AddressFault(addr, LOAD_PAGE));
        default:
            return NoFault;
    }
    // Note: Also AMO_PAGE exception code
}

inline void
TLB::incrementTlbStats(Mode mode, bool isHit)
{
    switch (mode) {
        case Read:
        case Execute:
            (isHit) ? read_hits++ : read_misses++;
            break;
        case Write:
            (isHit) ? write_hits++ : write_misses++;
            break;
        default:
            panic("Computing stats of unknown mode.");
    }
}

RiscvISA::TLB *
RiscvTLBParams::create()
{
    return new TLB(this);
}
