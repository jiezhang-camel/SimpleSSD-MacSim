#ifndef SIMPLESSD_INTERFACE_H_
#define SIMPLESSD_INTERFACE_H_
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <string>

#include "dram.h"
#include "global_defs.h"
#include "global_types.h"
#include "macsim.h"

#include "simplessd/hil/hil.hh"


class simplessd_interface_c : public dram_c {
 public:
  simplessd_interface_c(macsim_c *simBase);
  ~simplessd_interface_c();
  void init(int id);
  void run_a_cycle(bool);
  void send(void);
  void receive(void);
  void print_req(void);
  uint64_t totalLogicalPages;
  uint32_t logicalPageSize;
 private:
  float clock_freq;
  map<unsigned long long, mem_req_s *> m_input_buffer;
  list<mem_req_s *> m_output_buffer;
  list<mem_req_s *> m_buffer;
  SimpleSSD::PAL::Parameter *palparam;
};

class flash_interface_c : public dram_c {
 public:
  flash_interface_c(macsim_c *simBase);
  ~flash_interface_c();
  void init(int id);
  void run_a_cycle(bool);
  void send(void);
  void receive(void);
  void print_req(void);
  bool insert_new_req(unsigned long long &, mem_req_s *);

  //SimpleSSD::ConfigReader configReader;
  //SimpleSSD::HIL::HIL *pHIL;
  uint64_t totalLogicalPages;
  uint32_t logicalPageSize;
  uint32_t totalDie;
  uint32_t totalPlane;
  SimpleSSD::PAL::Parameter *palparam;
  struct _pageregInternal{
    Addr ppn;
    uint32_t page;
    uint64_t available_time;
    bool valid;
    bool dirty;
  };
  struct _pageregInternal **pageregInternal;
  bool FindCandidateSlot(struct _pageregInternal *pageregInternal, 
                         int &idx, uint32_t search_page);
  priority_queue<uint64_t, vector<uint64_t>, greater<uint64_t>>
      *PackageIO, *DieIO, *PackageFlash, *DieFlash, *PlaneFlash;
  void IOportAccess(uint32_t Package, uint32_t Die, 
                                        uint64_t &finishTick);
  void FlashportGet(uint32_t Plane, uint64_t &finishTick);                                         
  void FlashportUpdate(uint32_t Plane, uint64_t &finishTick); 

 private:
  float clock_freq;
  map<unsigned long long, queue<mem_req_s *>> *m_input_buffer;
  map<unsigned long long, mem_req_s *> *m_output_buffer;
  map<unsigned long long, unsigned long long> m_lpn_ppn;
  list<mem_req_s *> m_buffer;
};

#endif
