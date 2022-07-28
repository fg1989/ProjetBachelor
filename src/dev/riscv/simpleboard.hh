/*
 * Copyright (c) 2018 TU Dresden
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
 * Authors: Robert Scheffel
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#ifndef __DEV_RISCV_SIMPLEBOARD_HH__
#define __DEV_RISCV_SIMPLEBOARD_HH__

#include "arch/riscv/system.hh"
#include "cpu/intr_control.hh"
#include "dev/platform.hh"
#include "dev/riscv/plic.hh"
#include "params/SimpleBoard.hh"

class RiscvSystem;
class Plic;

class SimpleBoard : public Platform
{
  public:
    RiscvSystem * system;
    Plic * plic;
    int consoleInt;

  public:
    typedef SimpleBoardParams Params;
    const Params* params() const {
        return dynamic_cast<const Params *>(_params);
    }
    SimpleBoard(const Params *p);

    void setPlic(Plic * plic);

    // Required by platform
    void postConsoleInt() override;
    void clearConsoleInt() override;
    void postPciInt(int line) override;
    void clearPciInt(int line) override;
};

#endif // __DEV_RISCV_SIMPLEBOARD_HH__
