/*
Copyright (c) <2012>, <Georgia Institute of Technology> All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of the <Georgia Institue of Technology> nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**********************************************************************************************
 * File         : dram.cc
 * Author       : HPArch Research Group
 * Date         : 2/18/2013
 * SVN          : $Id: dram.h 867 2009-11-05 02:28:12Z kacear $:
 * Description  : Memory controller
 *********************************************************************************************/

#include "dram.h"
#include "all_knobs.h"
#include "statistics.h"

#include <cstdio>
#include "simplessd_interface.h"

#include "simplessd/log/log.hh"

dram_c::dram_c(macsim_c *simBase) : m_simBase(simBase) {}

dram_c::~dram_c() {}


ssd_interface_c::ssd_interface_c(macsim_c *simBase) {
  ssd_req_id = 0;
  m_cycle = 0;

  SimpleSSD::Logger::initLogSystem(std::cout, std::cerr, [this]() -> uint64_t {
    return m_cycle * 1000 / clock_freq;
  });

  if (!configReader.init((string)*simBase->m_knobs->KNOB_SIMPLESSD_CONFIG)) {
    printf("Failed to read SimpleSSD configuration file!\n");

    terminate();
  }

  clock_freq = *simBase->m_knobs->KNOB_CLOCK_MC;

  pHIL = new SimpleSSD::HIL::HIL(&configReader);

  pHIL->getLPNInfo(totalLogicalPages, logicalPageSize);
}

ssd_interface_c::~ssd_interface_c() {
  delete pHIL;
}


unsigned long long ssd_interface_c::insert_ssd_req(unsigned long long start_time, 
                        int m_id, Addr m_addr, bool rw){
  SimpleSSD::ICL::Request request;

  request.reqID = m_id;
  request.offset = m_addr % logicalPageSize;
  request.length = 128;
  request.range.slpn = m_addr / logicalPageSize;
  request.range.nlp = (128 + request.offset + logicalPageSize - 1) /
                      logicalPageSize;

  uint64_t finishTick =
      static_cast<uint64_t>(start_time * 1000 / clock_freq);

  SimpleSSD::Logger::info("Request %d arrived at %d cycle (%" PRIu64 " ps)",
                          request.reqID, m_cycle, finishTick);

  if (rw)
    pHIL->write(request, finishTick);
  else
    pHIL->read(request, finishTick);

  finishTick = finishTick / 1000 * clock_freq;
  SimpleSSD::Logger::info("Request finished at %d cycle, delay %d cycle", 
                                  finishTick, finishTick - m_cycle); 
  return static_cast<unsigned long long>(finishTick);
}
