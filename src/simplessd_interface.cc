#include "assert_macros.h"
#include "bug_detector.h"
#include "debug_macros.h"
#include "dram_ctrl.h"
#include "memory.h"
#include "memreq_info.h"
#include "network.h"
#include "progress_checker.h"
#include "utils.h"

#include "all_knobs.h"
#include "statistics.h"

#include <cstdio>
#include "simplessd_interface.h"

#include "simplessd/log/log.hh"

#define DEBUG(args...) _DEBUG(*m_simBase->m_knobs->KNOB_DEBUG_DRAM, ##args)

dram_c *simplessd_interface(macsim_c *simBase) {
  dram_c *simplessd_interface_tmp = new simplessd_interface_c(simBase);
  return simplessd_interface_tmp;
}

dram_c *flash_interface(macsim_c *simBase) {
  dram_c *flash_interface_tmp = new flash_interface_c(simBase);
  return flash_interface_tmp;
}

simplessd_interface_c::simplessd_interface_c(macsim_c *simBase)
    : dram_c(simBase) {
  m_cycle = 0;
  clock_freq = *m_simBase->m_knobs->KNOB_CLOCK_MC;
}

flash_interface_c::flash_interface_c(macsim_c *simBase)
    : dram_c(simBase) {
  m_cycle = 0;
  SimpleSSD::Logger::initLogSystem(std::cout, std::cerr, [this]() -> uint64_t {
    return m_cycle * 1000 / clock_freq;
  });

  // if (!configReader.init((string)*m_simBase->m_knobs->KNOB_SIMPLESSD_CONFIG)) {
  //   printf("Failed to read SimpleSSD configuration file!\n");

  //   terminate();
  // }

  clock_freq = *m_simBase->m_knobs->KNOB_CLOCK_MC;

  //pHIL = new SimpleSSD::HIL::HIL(&configReader);
  m_input_buffer = new map<unsigned long long, queue<mem_req_s *>>;
  m_output_buffer = new map<unsigned long long, mem_req_s *>;

  // uint64_t FinishTime;
  // printf("Jie: FinishTime %lu\n",FinishTime);
  // pHIL->forward(1,1,1,1,1,1,FinishTime);
  // printf("Jie: FinishTime %lu\n",FinishTime);
}

simplessd_interface_c::~simplessd_interface_c() {
}

flash_interface_c::~flash_interface_c() {
  //delete pHIL;
  delete m_output_buffer;
  delete m_input_buffer;
}

void simplessd_interface_c::init(int id) {
  m_id = id;
  pHIL->getLPNInfo(totalLogicalPages, logicalPageSize);
}

void flash_interface_c::init(int id) {
  m_id = id;
  pHIL->getLPNInfo(totalLogicalPages, logicalPageSize);
}

void simplessd_interface_c::send(void) {
 mem_req_s *req = NIF_NETWORK->receive(MEM_MC, m_id);
  // if (!req)
  //   return;

  if (req) {
    //cout<<"Janalysis: flash-SSD latency "<<m_cycle - req->m_in<<endl;
    //Janalysis
    //req->m_in = m_cycle;
    
    //cout << "Jie: send "<< req->m_id << endl; 
    m_output_buffer.push_back(req);
    NIF_NETWORK->receive_pop(MEM_MC, m_id);
    if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
      m_simBase->m_bug_detector->deallocate_noc(req);
    }
    m_simBase->m_progress_checker->increment_outstanding_requests();
  }

  bool req_type_checked[2];
  req_type_checked[0] = false;
  req_type_checked[1] = false;

  bool req_type_allowed[2];
  req_type_allowed[0] = true;
  req_type_allowed[1] = true;

  int max_iter = 1;
  if (*KNOB(KNOB_ENABLE_NOC_VC_PARTITION))
    max_iter = 2;  
  // when virtual channels are partitioned for CPU and GPU requests,
  // we need to check individual buffer entries
  // if not, sequential search would be good enough
  for (int ii = 0; ii < max_iter; ++ii) {
    req_type_allowed[0] = !req_type_checked[0];
    req_type_allowed[1] = !req_type_checked[1];
    // check both CPU and GPU requests
    if (req_type_checked[0] == true && req_type_checked[1] == true)
      break;
    for (auto I = m_output_buffer.begin(), E = m_output_buffer.end();
         I != E;) {
      mem_req_s *req = *I;
      if (req_type_allowed[req->m_ptx] == false) {
        ++I;
        continue;
      }

      req_type_checked[req->m_ptx] = true;
      req->m_msg_type = NOC_FILL;

      req->m_in = m_cycle;
      bool insert_packet =
          NETWORK->send(req, MEM_MC, m_id, MEM_L3, req->m_cache_id[MEM_L3]);

      if (!insert_packet) {
        DEBUG("MC[%d] req:%d addr:0x%llx type:%s noc busy\n", m_id, req->m_id,
              req->m_addr, mem_req_c::mem_req_type_name[req->m_type]);
        break;
      } 

      // cout << "Jie: ioreq latency "<< m_cycle - req->m_in << endl;

      if (*KNOB(KNOB_BUG_DETECTOR_ENABLE) && *KNOB(KNOB_ENABLE_NEW_NOC)) {
        m_simBase->m_bug_detector->allocate_noc(req);
      }
      I = m_output_buffer.erase(I);

      m_simBase->m_progress_checker->decrement_outstanding_requests();
      m_simBase->m_progress_checker->update_dram_progress_info(
          m_simBase->m_simulation_cycle);
    }
  }
}

void flash_interface_c::send(void) {
  bool req_type_checked[2];
  req_type_checked[0] = false;
  req_type_checked[1] = false;

  bool req_type_allowed[2];
  req_type_allowed[0] = true;
  req_type_allowed[1] = true;

  int max_iter = 1;
  if (*KNOB(KNOB_ENABLE_NOC_VC_PARTITION))
    max_iter = 2;

  // when virtual channels are partitioned for CPU and GPU requests,
  // we need to check individual buffer entries
  // if not, sequential search would be good enough
  for (int ii = 0; ii < max_iter; ++ii) {
    req_type_allowed[0] = !req_type_checked[0];
    req_type_allowed[1] = !req_type_checked[1];
    // check both CPU and GPU requests
    if (req_type_checked[0] == true && req_type_checked[1] == true)
      break;
    for (auto I = m_output_buffer->begin(), E = m_output_buffer->end();
         I != E;) {
      if (I->first > m_cycle)
        break;

      mem_req_s *req = I->second;
      if (req_type_allowed[req->m_ptx] == false) {
        ++I;
        continue;
      }

      req_type_checked[req->m_ptx] = true;
      req->m_msg_type = NOC_FILL;
      req->m_in = m_cycle;
      bool insert_packet =
          NIF_NETWORK->send(req, MEM_FLASH, req->m_cache_id[MEM_FLASH], MEM_MC, req->m_cache_id[MEM_MC]);

      if (!insert_packet) {
        DEBUG("MC[%d] req:%d addr:0x%llx type:%s noc busy\n", m_id, req->m_id,
              req->m_addr, mem_req_c::mem_req_type_name[req->m_type]);
        break;
      }
      //cout << "NIF_NETWORK send: flash_id "<< req->m_cache_id[MEM_FLASH] << " mc_id "<< req->m_cache_id[MEM_MC] << endl;

      if (*KNOB(KNOB_BUG_DETECTOR_ENABLE) && *KNOB(KNOB_ENABLE_NEW_NOC)) {
        m_simBase->m_bug_detector->allocate_noc(req);
      }
      I = m_output_buffer->erase(I);

      m_simBase->m_progress_checker->decrement_outstanding_requests();
      m_simBase->m_progress_checker->update_dram_progress_info(
          m_simBase->m_simulation_cycle);
    }
  }
}

void simplessd_interface_c::receive(void) {
  // check router queue every cycle
  // first, receive message from GPU network
  mem_req_s *req = NETWORK->receive(MEM_MC, m_id);
  // if (!req)
  //   return;

  if (req) {
    // cout<<"Janalysis: L3-SSD latency "<<m_cycle - req->m_in<<endl;
    // Janalysis
    // req->m_in = m_cycle;
    
    //cout << "Jie: receive "<< req->m_id << endl;
    SimpleSSD::ICL::Request request;
    request.reqID = req->m_id;
    request.offset = req->m_addr % logicalPageSize;
    request.length = req->m_size;
    request.range.slpn = req->m_addr / logicalPageSize;
    request.range.nlp = (req->m_size + request.offset + logicalPageSize - 1) /
                        logicalPageSize;
    uint64_t finishTick =
        static_cast<unsigned long long>(m_cycle * 1000 / clock_freq);
    uint32_t ppn;
    uint32_t channel;
    uint32_t package;
    uint32_t die;
    uint32_t plane;
    uint32_t block;
    uint32_t page;      
    pHIL->collectPPN(request, ppn, channel, package, die, plane, 
                        block, page, finishTick);
    req->m_cache_id[MEM_FLASH] = package;
    finishTick = finishTick / 1000 * clock_freq;
    while (1){
      auto iter = m_input_buffer.find(finishTick);
      if (iter != m_input_buffer.end()) finishTick++;
      else break;
    }  
    m_input_buffer.insert(pair<unsigned long long, mem_req_s *>(
      static_cast<unsigned long long>(finishTick), req));
    NETWORK->receive_pop(MEM_MC, m_id);
    if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
      m_simBase->m_bug_detector->deallocate_noc(req);
    }
    m_simBase->m_progress_checker->increment_outstanding_requests();
  }
  // second, send message to flash network
  bool req_type_checked[2];
  req_type_checked[0] = false;
  req_type_checked[1] = false;

  bool req_type_allowed[2];
  req_type_allowed[0] = true;
  req_type_allowed[1] = true;

  int max_iter = 1;
  if (*KNOB(KNOB_ENABLE_NOC_VC_PARTITION))
    max_iter = 2;  
  // when virtual channels are partitioned for CPU and GPU requests,
  // we need to check individual buffer entries
  // if not, sequential search would be good enough
  for (int ii = 0; ii < max_iter; ++ii) {
    req_type_allowed[0] = !req_type_checked[0];
    req_type_allowed[1] = !req_type_checked[1];
    // check both CPU and GPU requests
    if (req_type_checked[0] == true && req_type_checked[1] == true)
      break;
    for (auto I = m_input_buffer.begin(), E = m_input_buffer.end();
         I != E;) {
      if (I->first > m_cycle)
        break;
      mem_req_s *req = I->second;
      if (req_type_allowed[req->m_ptx] == false) {
        ++I;
        continue;
      }

      req_type_checked[req->m_ptx] = true;
      req->m_msg_type = NOC_FILL;
      bool insert_packet =
          NIF_NETWORK->send(req, MEM_MC, m_id, MEM_FLASH, 
                                req->m_cache_id[MEM_FLASH]);

      if (!insert_packet) {
        DEBUG("MC[%d] req:%d addr:0x%llx type:%s noc busy\n", m_id, req->m_id,
              req->m_addr, mem_req_c::mem_req_type_name[req->m_type]);
        break;
      }
      //cout<<"Janalysis: SSDengine latency "<<m_cycle - req->m_in<<endl;
      //Janalysis
      //req->m_in = m_cycle;
      
      if (*KNOB(KNOB_BUG_DETECTOR_ENABLE) && *KNOB(KNOB_ENABLE_NEW_NOC)) {
        m_simBase->m_bug_detector->allocate_noc(req);
      }
      I = m_input_buffer.erase(I);

      m_simBase->m_progress_checker->decrement_outstanding_requests();
      m_simBase->m_progress_checker->update_dram_progress_info(
          m_simBase->m_simulation_cycle);
    }
  }  
}

void flash_interface_c::receive(void) {
  for (auto flash_id = 0; flash_id < pHIL->getFlashNum(); flash_id++){
    // check router queue every cycle
    mem_req_s *req = NIF_NETWORK->receive(MEM_FLASH, flash_id);
    if (req){
      //cout<<"Janalysis: SSD-flash latency "<<m_cycle - req->m_in<<endl;
      //Janalysis
      //req->m_in = m_cycle;
      
      // check if req has same slot in the input buffer
      bool input_buffer_hit = false;
      bool output_buffer_hit = false;
      for (auto I = m_input_buffer->begin(), E = m_input_buffer->end();
            I != E; I++){
        if ((I->second).front()->m_addr / logicalPageSize 
                                    == req->m_addr / logicalPageSize){
          input_buffer_hit = true;
          (I->second).push(req);
          NIF_NETWORK->receive_pop(MEM_FLASH, flash_id);
          if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
            m_simBase->m_bug_detector->deallocate_noc(req);
          }
          m_simBase->m_progress_checker->increment_outstanding_requests();
          break;
        }
      }
      if (input_buffer_hit == false){
        for (auto I = m_output_buffer->begin(), E = m_output_buffer->end();
              I != E; I++){
          if (I->second->m_addr / logicalPageSize 
                                    == req->m_addr / logicalPageSize){
            unsigned long long tmp_time = I->first+1;
            while (1){
              auto iter = m_input_buffer->find(tmp_time);
              if (iter != m_input_buffer->end()) tmp_time++;
              else break;
            }
            queue<mem_req_s *> tmp_queue;
            tmp_queue.push(req);
            m_input_buffer->insert( pair<unsigned long long, queue<mem_req_s *>>(
                  tmp_time, tmp_queue));
            output_buffer_hit = true;
            NIF_NETWORK->receive_pop(MEM_FLASH, flash_id);
            if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
              m_simBase->m_bug_detector->deallocate_noc(req);
            }
            m_simBase->m_progress_checker->increment_outstanding_requests();
            break;
          }
        }    
      }
      if (input_buffer_hit == false && output_buffer_hit == false){
        unsigned long long finishTime;
        //cout<<"Janalysis: queue latency "<<m_cycle - req->m_in<<endl;
        //Janalysis
        //req->m_in = m_cycle;
        
        if (req && insert_new_req(finishTime,req)) {
          NIF_NETWORK->receive_pop(MEM_FLASH, flash_id);
          if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
            m_simBase->m_bug_detector->deallocate_noc(req);
          }
          m_simBase->m_progress_checker->increment_outstanding_requests();
        }
      }  
    }
    auto I = m_input_buffer->begin();
    auto E = m_input_buffer->end();
    if ((I != E) && (I->first <= m_cycle)){
      unsigned long long finishTime;
      //cout<<"Janalysis: queue latency "<<m_cycle - (I->second).front()->m_in<<endl;
      (I->second).front()->m_in = m_cycle;  
      insert_new_req(finishTime,(I->second).front());
      finishTime++;
      if((I->second).size() == 1) m_input_buffer->erase(I);
      else{
        (I->second).pop();
        while (1){
          auto iter = m_input_buffer->find(finishTime);
          if (iter != m_input_buffer->end())finishTime++;
          else break;
        }
        queue<mem_req_s *> tmp_queue;
        while (!(I->second).empty()){
          tmp_queue.push((I->second).front());
          (I->second).pop();
        }
        m_input_buffer->erase(I);
        m_input_buffer->insert( pair<unsigned long long, queue<mem_req_s *>>(
              finishTime, tmp_queue));      
      }
    }  
    
  }
}

bool flash_interface_c::insert_new_req(unsigned long long &finishTime, 
                                             mem_req_s *mem_req) {
  SimpleSSD::ICL::Request request;
  request.reqID = mem_req->m_id;
  request.offset = mem_req->m_addr % logicalPageSize;
  request.length = mem_req->m_size;
  request.range.slpn = mem_req->m_addr / logicalPageSize;
  request.range.nlp = (mem_req->m_size + request.offset + logicalPageSize - 1) /
                      logicalPageSize;
                      
  uint64_t finishTick =
      static_cast<unsigned long long>(m_cycle * 1000 / clock_freq);
  uint32_t ppn;
  uint32_t channel;
  uint32_t package;
  uint32_t die;
  uint32_t plane;
  uint32_t block;
  uint32_t page;
  uint32_t destPlane = (uint32_t)-1;
  SimpleSSD::Logger::info("Request %d arrived at %d cycle (%" PRIu64 " ps)",
                        request.reqID, m_cycle, finishTick);
  pHIL->collectPPN(request, ppn, channel, package, die, plane, 
                          block, page, finishTick);
  finishTick =
      static_cast<unsigned long long>(m_cycle * 1000 / clock_freq);
  // if (mem_req->m_dirty)
  //   printf("Jie: write lpn %lu ppn %u channel %u package %u die %u plane %u \
  //             block %u page %u\n", request.range.slpn, ppn, channel, package,
  //           die, plane, block, page);
  // else
  //   printf("Jie: read lpn %lu ppn %u channel %u package %u die %u plane %u \
  //             block %u page %u\n", request.range.slpn, ppn, channel, package,
  //           die, plane, block, page);
  
  bool isHit = pHIL->pageregCheck(ppn, channel, package, die, plane,
                         block, page, destPlane);
  if (mem_req->m_dirty){
      if (isHit){
        SimpleSSD::Logger::debugprint(SimpleSSD::Logger::LOG_HIL,
                     "WRITE | REQ %7u | LCA %" PRIu64 " + %" PRIu64
                     " | BYTE %" PRIu64 " + %" PRIu64,
                     request.reqID, request.range.slpn, request.range.nlp, 
                     request.offset, request.length);      
        pHIL->forward(channel, package, die, plane,
                         block, page, finishTick);
      }   
      else{
        pHIL->write(request, finishTick);
      } 
  }
  else{
    pHIL->read(request, finishTick);
  }  
  finishTick = finishTick / 1000 * clock_freq;
  SimpleSSD::Logger::info("Request finished at %d cycle, delay %d cycle", 
                                  finishTick, finishTick - m_cycle);
  while (1){
    auto iter = m_output_buffer->find(finishTick);
    if (iter != m_output_buffer->end()) finishTick++;
    else break;
  }
  m_output_buffer->insert(pair<unsigned long long, mem_req_s *>(
      static_cast<unsigned long long>(finishTick), mem_req));
  finishTime = static_cast<unsigned long long>(finishTick);
  //cout << "NIF_NETWORK finishTime "<< finishTime << endl;
  return true;
}

// tick a cycle
void simplessd_interface_c::run_a_cycle(bool pll_lock) {
  if (pll_lock) {
    ++m_cycle;
    return;
  }
  send();

  receive();

  ++m_cycle;
}

// tick a cycle
void flash_interface_c::run_a_cycle(bool pll_lock) {
  if (pll_lock) {
    ++m_cycle;
    return;
  }
  send();

  receive();

  ++m_cycle;
}

void simplessd_interface_c::print_req(void) {
  FILE *fp = fopen("bug_detect_dram.out", "w");

  fprintf(fp, "Current cycle:%llu\n", m_cycle);
  // fprintf(fp, "Total req:%d\n", m_total_req);
  fprintf(fp, "\n");
  fclose(fp);

  //  g_memory->print_mshr();
}

void flash_interface_c::print_req(void) {
  FILE *fp = fopen("bug_detect_flash.out", "w");

  fprintf(fp, "Current cycle:%llu\n", m_cycle);
  // fprintf(fp, "Total req:%d\n", m_total_req);
  fprintf(fp, "\n");
  fclose(fp);

  //  g_memory->print_mshr();
}
