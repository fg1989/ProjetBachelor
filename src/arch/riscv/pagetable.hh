/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#ifndef __ARCH_RISCV_PAGETABLE_H__
#define __ARCH_RISCV_PAGETABLE_H__

#include "base/logging.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace RiscvISA {

struct VAddr
{
    VAddr(Addr a) { panic("VAddr struct not implemented."); }
};

struct Sv39VAddr
{
    uint16_t vpn[3];
    uint16_t page_offset;
    Addr addr;

    Sv39VAddr(Addr va) {
        page_offset = va & 0xFFF;
        vpn[0] = (va >> 12) & 0x1FF;
        vpn[1] = (va >> 21) & 0x1FF;
        vpn[2] = (va >> 30) & 0x1FF;
        addr = va;
    }

    Addr getAddr() {
        return addr;
    }
};

struct Sv39PAddr
{
    uint32_t ppn[3];
    uint16_t page_offset;
    Addr addr;

    Sv39PAddr(Addr pa) {
        page_offset = pa & 0xFFF;
        ppn[0] = (pa >> 12) & 0x1FF;
        ppn[1] = (pa >> 21) & 0x1FF;
        ppn[2] = (pa >> 30) & 0x3FFFFFF;
        addr = pa;
    }

    Addr getAddr() {
        return addr;
    }

    Addr formatAddr() {
        return (page_offset | (ppn[0] << 12) | (ppn[1] << 21) |
            (ppn[2] << 30));
    }
};

// Adding Sv39 PTE, from the privileged specification, section 4.4.1, pages
// 70-71.
struct PTE : public Serializable
{
    Addr insertAddr;
    uint8_t asid;
    uint8_t rsw;
    uint32_t ppn[3]; // Physical page number; 9, 9, and 26 bits.
    uint64_t ppnCombo; // Combined PPN value
    bool D; // Dirty bit
    bool A; // Accessed bit
    bool G; // Already included; designates global mapping
    bool U; // Indicates if accessible in user mode
    bool X; // Indicates if PTE is executable
    bool W; // Indicates if PTE is writable
    bool R; // Indicates if PTE is readable
    bool V; // Valid bit; if 0, all other fields are don't cares

    /* Table 4.4 is copied here for reference.
     * X | W | R | Meaning
     * ------------------------------------------------
     * 0 | 0 | 0 | Pointer to next level of page table, otherwise a leaf node.
     * 0 | 0 | 1 | Read-only page.
     * 0 | 1 | 0 | Reserved.
     * 0 | 1 | 1 | Read-write page.
     * 1 | 0 | 0 | Execute-only page.
     * 1 | 0 | 1 | Read-execute page.
     * 1 | 1 | 0 | Reserved.
     * 1 | 1 | 1 | Read-write-execute page.
     */

    PTE() {}

    PTE(Addr va) {
        V = va & 0x1;
        R = (va >> 1) & 0x1;
        W = (va >> 2) & 0x1;
        X = (va >> 3) & 0x1;
        U = (va >> 4) & 0x1;
        G = (va >> 5) & 0x1;
        A = (va >> 6) & 0x1;
        D = (va >> 7) & 0x1;
        rsw = (va >> 8) & 0x3;
        ppn[0] = (va >> 10) & 0x1ff;
        ppn[1] = (va >> 19) & 0x1ff;
        ppn[2] = (va >> 28) & 0x3ffffff;
        asid = 0;
        insertAddr = va;
        ppnCombo = (va >> 10) & 0xfffffffffff;
    }

    PTE(PTE * pte) {
        V = pte->V;
        R = pte->R;
        W = pte->W;
        X = pte->X;
        U = pte->U;
        G = pte->G;
        A = pte->A;
        D = pte->D;
        rsw = pte->rsw;
        ppn[0] = pte->ppn[0];
        ppn[1] = pte->ppn[1];
        ppn[2] = pte->ppn[2];
        asid = pte->asid;
        insertAddr = pte->insertAddr;
        ppnCombo = pte->ppnCombo;
    }

    bool
    Valid()
    {
        return (V);
    }

    Addr
    getAddr() {
        return (V | (R << 1) | (W << 2) | (X << 3) | (U << 4) | (G << 5) |
        (A << 6) | (D << 7) | (rsw << 8) | (ppn[0] << 10) | (ppn[1] << 19) |
        (ppn[2] << 28));
    }

    bool
    match(Addr vpn) const
    {
        return match(vpn, 0, true);
    }

    bool
    match(Addr vpn, uint16_t asn, bool ignore_asn) const
    {
        bool s = (ignore_asn || (asn == asid)) &&
                 (vpn == insertAddr) &&
                 (V);
        return s;
    }

    void
    invalidate()
    {
        V = 0;
    }

    void serialize(CheckpointOut &cp) const;
    void unserialize(CheckpointIn &cp);
};

};
#endif // __ARCH_RISCV_PAGETABLE_H__

