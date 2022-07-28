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

#ifndef __ARCH_LINUX_RISCV_SYSTEM_HH__
#define __ARCH_LINUX_RISCV_SYSTEM_HH__

#include "arch/riscv/system.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/object_file.hh"
#include "params/LinuxRiscvSystem.hh"

class LinuxRiscvSystem : public RiscvSystem
{
  protected:
    ObjectFile * bootloader;
    SymbolTable * bootloaderSymtab;
    Clint * clint;
    Plic * plic;

  public:
    typedef LinuxRiscvSystemParams Params;
    LinuxRiscvSystem(Params *p);
    ~LinuxRiscvSystem();

    // Initialize system
    virtual void initState() override;

    Clint * getClint() override;
    void setClint(Clint * c) override;
    Plic * getPlic() override;
    void setPlic(Plic * p) override;

  protected:
    const Params *params() const { return (const Params *)_params; }
};

#endif // __ARCH_LINUX_RISCV_SYSTEM_HH__
