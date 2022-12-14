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
 * Authors: Ali Saidi
 *          Nathan Binkert
 *          Jaidev Patwardhan
 *          Robert Scheffel
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#include "arch/riscv/system.hh"
#include "arch/riscv/registers.hh"
#include "arch/vtophys.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/hex_file.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "params/RiscvSystem.hh"
#include "sim/byteswap.hh"

using namespace LittleEndianGuest;

RiscvSystem::RiscvSystem(Params *p)
    : System(p),
      _isBareMetal(p->bare_metal),
      _resetVect(p->reset_vect)
{}

RiscvSystem::~RiscvSystem()
{}

Addr
RiscvSystem::fixFuncEventAddr(Addr addr)
{
    return addr;
}

// TODO: Possibly move this to a linux/system.cc::initState
void
RiscvSystem::initState()
{
    // Call the initialisation of the super class
    System::initState();
    return;
}

void
RiscvSystem::setRiscvAccess(Addr access)
{
}

bool
RiscvSystem::breakpoint()
{
    return 0;
}

Clint *
RiscvSystem::getClint()
{
    warn("Call to unimplemented RiscvSystem::getClint.");
    return NULL;
}

void
RiscvSystem::setClint(Clint * c)
{
    warn("Call to unimplemented RiscvSystem::setClint.");
}

Plic *
RiscvSystem::getPlic()
{
    warn("Call to unimplemented RiscvSystem::getPlic.");
    return NULL;
}

void
RiscvSystem::setPlic(Plic * p)
{
    warn("Call to unimplemented RiscvSystem::setPlic.");
}

RiscvSystem *
RiscvSystemParams::create()
{
    return new RiscvSystem(this);
}

