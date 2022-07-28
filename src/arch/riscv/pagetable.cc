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

#include "arch/riscv/pagetable.hh"

#include "sim/serialize.hh"

namespace RiscvISA
{

void
PTE::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(insertAddr);
    SERIALIZE_SCALAR(asid);
    SERIALIZE_SCALAR(rsw);
    SERIALIZE_SCALAR(G);
    SERIALIZE_SCALAR(D);
    SERIALIZE_SCALAR(A);
    SERIALIZE_SCALAR(U);
    SERIALIZE_SCALAR(X);
    SERIALIZE_SCALAR(W);
    SERIALIZE_SCALAR(R);
    SERIALIZE_SCALAR(V);
    SERIALIZE_SCALAR(ppn[0]);
    SERIALIZE_SCALAR(ppn[1]);
    SERIALIZE_SCALAR(ppn[2]);
    SERIALIZE_SCALAR(ppnCombo);
}

void
PTE::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(insertAddr);
    UNSERIALIZE_SCALAR(asid);
    UNSERIALIZE_SCALAR(rsw);
    UNSERIALIZE_SCALAR(G);
    UNSERIALIZE_SCALAR(D);
    UNSERIALIZE_SCALAR(A);
    UNSERIALIZE_SCALAR(U);
    UNSERIALIZE_SCALAR(X);
    UNSERIALIZE_SCALAR(W);
    UNSERIALIZE_SCALAR(R);
    UNSERIALIZE_SCALAR(V);
    UNSERIALIZE_SCALAR(ppn[0]);
    UNSERIALIZE_SCALAR(ppn[1]);
    UNSERIALIZE_SCALAR(ppn[2]);
    UNSERIALIZE_SCALAR(ppnCombo);
}

}
