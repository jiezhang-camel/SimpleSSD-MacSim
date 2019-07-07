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
#include <set>
#include <string>
#include <bitset>

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
    Addr page;
    uint64_t available_time;
    bool valid;
    bool dirty;
    uint32_t usedSector;
    uint32_t reaccess;
  };
  struct _pageregInternal **pageregInternal;
  bool FindCandidateSlot(struct _pageregInternal *pageregInternal, 
      int &cache_idx, int &data_idx, uint64_t search_page, bool isWrite);
  bool FindCandidateSlot_HBNET(int readPlaneIdx, int reqPlaneIdx, 
      int &cache_idx, int &data_idx, uint64_t search_page, bool isWrite);    
  uint64_t *planeAvailableTime, *flashportAvailableTime; 
  typedef struct _TimeSlot{
    uint64_t StartTick;
    uint64_t EndTick;
  } TimeSlot;
  set<uint64_t> *IOportTimeSlot;
  void allocateIOport(int offset, int size, uint64_t &startTime, 
                                          uint64_t &finishTime);
  uint32_t converttoPlaneIdx(uint32_t channel, uint32_t package,
    uint32_t die, uint32_t plane);
  uint32_t converttoDieIdx(uint32_t channel, uint32_t package,
    uint32_t die); 
  uint32_t converttoPackageIdx(uint32_t channel, uint32_t package); 
  uint32_t cleanup_counter; 
  void cleanIOPort();

 private:
  float clock_freq;
  map<unsigned long long, queue<mem_req_s *>> *m_input_buffer;
  map<unsigned long long, mem_req_s *> *m_output_buffer;
  map<unsigned long long, unsigned long long> m_lpn_ppn;
  list<mem_req_s *> m_buffer;
};

#endif
