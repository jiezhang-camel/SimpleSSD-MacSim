#ifndef SIMPLESSD_INTERFACE_H_
#define SIMPLESSD_INTERFACE_H_
#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <list>
#include <map>

#include "macsim.h"
#include "dram.h"
#include "global_types.h"
#include "global_defs.h"

#include "SimpleSSD/ftl/abstract_ftl.hh"
#include "SimpleSSD/ftl/config.hh"
#include "SimpleSSD/ftl/ftl.hh"
#include "SimpleSSD/ftl/page_mapping.hh"
#include "SimpleSSD/ftl/common/block.hh"
#include "SimpleSSD/hil/hil.hh"
#include "SimpleSSD/hil/nvme/config.hh"
#include "SimpleSSD/hil/nvme/controller.hh"
#include "SimpleSSD/hil/nvme/def.hh"
#include "SimpleSSD/hil/nvme/dma.hh"
#include "SimpleSSD/hil/nvme/interface.hh"
#include "SimpleSSD/hil/nvme/namespace.hh"
#include "SimpleSSD/hil/nvme/queue.hh"
#include "SimpleSSD/hil/nvme/subsystem.hh"
#include "SimpleSSD/icl/abstract_cache.hh"
#include "SimpleSSD/icl/config.hh"
#include "SimpleSSD/icl/generic_cache.hh"
#include "SimpleSSD/icl/icl.hh"
#include "SimpleSSD/lib/ini/ini.h"
#include "SimpleSSD/log/log.hh"
#include "SimpleSSD/log/stat.hh"
#include "SimpleSSD/log/trace.hh"
#include "SimpleSSD/pal/abstract_pal.hh"
#include "SimpleSSD/pal/config.hh"
#include "SimpleSSD/pal/pal.hh"
#include "SimpleSSD/pal/pal_old.hh"
#include "SimpleSSD/pal/old/Latency.h"
#include "SimpleSSD/pal/old/LatencyMLC.h"
#include "SimpleSSD/pal/old/LatencySLC.h"
#include "SimpleSSD/pal/old/LatencyTLC.h"
#include "SimpleSSD/pal/old/PAL2.h"
#include "SimpleSSD/pal/old/PAL2_TimeSlot.h"
#include "SimpleSSD/pal/old/PALStatistics.h"
#include "SimpleSSD/util/algorithm.hh"
#include "SimpleSSD/util/base_config.hh"
#include "SimpleSSD/util/config.hh"
#include "SimpleSSD/util/def.hh"
#include "SimpleSSD/util/disk.hh"
#include "SimpleSSD/util/hash_table.hh"
#include "SimpleSSD/util/list.hh"
#include "SimpleSSD/util/vector.hh"

class simplessd_interface_c : public dram_c
{
  public:
    simplessd_interface_c(macsim_c* simBase);
    ~simplessd_interface_c();
    void init(int id);
    void run_a_cycle(bool);
    void send(void);
    void receive(void);
    void print_req(void);
    bool insert_new_req(mem_req_s*);

  private:
    string SSD_config;
    SimpleSSD::ConfigReader *cfg;
    SimpleSSD::HIL::HIL *m_hil;
    float clock_freq;
    map<unsigned long long, mem_req_s*> *m_output_buffer;
};

#endif

