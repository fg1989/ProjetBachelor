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

#include "arch/riscv/table_walker.hh"

#include <memory>

#include "arch/riscv/faults.hh"
#include "arch/riscv/system.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "dev/dma_device.hh"

using namespace RiscvISA;

TableWalker::TableWalker(const Params * p) :
    SimObject(p), sys(p->sys), tlb(NULL), port(NULL)
{
}

void
TableWalker::init()
{
    fatal_if(!tlb, "Table walker must have a valid TLB.\n");
    fatal_if(!port, "Table walker must have a valid port.\n");
}

TableWalker::~TableWalker()
{
    if (port) delete [] port;
}

Port &
TableWalker::getPort(const std::string & if_name, PortID idx)
{
    if (if_name == "port") return * port;
    else return SimObject::getPort(if_name, idx);  // Ends in fatal crash
}

void
TableWalker::setupWalker(TLB * targetTlb, System * targetSys,
    MasterID masterId)
{
    tlb = targetTlb;
    port = new DmaPort(reinterpret_cast<ClockedObject *>(targetTlb),
                        (System *)targetSys, 0, 0);
    mid = masterId;
}

Fault
TableWalker::walk(const RequestPtr &_req, ThreadContext * tc, uint16_t _asid,
                  BaseTLB::Mode _mode, bool _timing, bool functional,
                  Addr lookupAddr, Request::Flags flags, Addr & resultPte)
{
    // TODO: Add tick delay here (8th arg).
    port->dmaAction(MemCmd::ReadReq, lookupAddr, 8, NULL, (uint8_t*)&resultPte,
                    0, 0, tc->getCpuPtr()->clockPeriod(), flags);
    return NoFault;
}

RiscvISA::TableWalker *
RiscvTableWalkerParams::create()
{
    return new TableWalker(this);
}
