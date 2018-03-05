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

#include "simplessd/hil/hil.hh"

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
    SimpleSSD::ConfigReader configReader;
    SimpleSSD::HIL::HIL *pHIL;
    uint64_t totalLogicalPages;
    uint32_t logicalPageSize;

    float clock_freq;
    map<unsigned long long, mem_req_s*> *m_output_buffer;
};

#endif
