/*
 * Copyright (c) 2010-2016 ARM Limited
 * Copyright (c) 2019 EPFL
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Ali Saidi
 *          Giacomo Gabrielli
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#ifndef __ARCH_RISCV_TABLE_WALKER_HH__
#define __ARCH_RISCV_TABLE_WALKER_HH__

#include <list>

#include "arch/riscv/pagetable.hh"
#include "arch/riscv/system.hh"
#include "arch/riscv/tlb.hh"
#include "dev/dma_device.hh"
#include "mem/request.hh"
#include "params/RiscvTableWalker.hh"
#include "sim/eventq.hh"
#include "sim/system.hh"

class ThreadContext;
class DmaPort;

namespace RiscvISA {

class TLB;
class TableWalker : public SimObject
{
protected:
    System * sys;   // System this device is apart of
    TLB * tlb;      // TLB this walker belongs to
    DmaPort * port; // Port for memory interfacing
    MasterID mid;   // TLB master ID

public:
    typedef RiscvTableWalkerParams Params;
    TableWalker(const Params *p);
    void init() override;
    virtual ~TableWalker();
    Port & getPort(const std::string & if_name, PortID idx) override;
    // TODO: Override other methods in sim/sim_object.hh
    // TODO: Add statistics

    void setupWalker(TLB * targetTlb, System * targetSys, MasterID masterId);
    Fault walk(const RequestPtr &_req, ThreadContext *_tc, uint16_t _asid,
            BaseTLB::Mode _mode, bool _timing, bool functional,
            Addr lookupAddr, Request::Flags flags, Addr & resultPte);
};

} // namespace RiscvISA

#endif //__ARCH_RISCV_TABLE_WALKER_HH__

