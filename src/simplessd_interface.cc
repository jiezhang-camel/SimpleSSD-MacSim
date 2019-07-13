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
#define NO_NET 0
#define FC_NET 1
#define HB_NET_OLD 2
#define HB_NET 3
#define tBUSY 3000 * 1.6

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
  if (palparam->pageReg != 0){
    delete pageregInternal;
  }
}

void simplessd_interface_c::init(int id) {
  m_id = id;
  pHIL->getLPNInfo(totalLogicalPages, logicalPageSize);
  palparam = pHIL->getPALInfo();
}

void flash_interface_c::init(int id) {
  m_id = id;
  cleanup_counter = 0;
  pHIL->getLPNInfo(totalLogicalPages, logicalPageSize);
  palparam = pHIL->getPALInfo();
  totalDie = palparam->channel * palparam->package * palparam->die;
  totalPlane = palparam->channel * palparam->package * palparam->die * palparam->plane
                * palparam->nandMultiapp;
  // Initialize the page registers
  if (palparam->pageReg != 0){
    pageregInternal = new struct _pageregInternal *[totalPlane *
                                  palparam->pageReg / palparam->pageRegAssoc];
    for (unsigned i = 0; i < totalPlane* palparam->pageReg 
                                  / palparam->pageRegAssoc; i++){
      pageregInternal[i] = new struct _pageregInternal [palparam->pageRegAssoc];
      for (unsigned j = 0; j < palparam->pageRegAssoc; j++){
        pageregInternal[i][j].valid = false;
        pageregInternal[i][j].available_time = 0;
        pageregInternal[i][j].usedSector = 0;
        pageregInternal[i][j].reaccess = 0;
      }
    }
  }
  printf("pageReg %u pageRegAssoc %u slcBuffer %u regSwapper %u nandSplit %u\n",
    palparam->pageReg, palparam->pageRegAssoc, palparam->slcBuffer, palparam->regSwapper,
    palparam->nandSplit);
  planeAvailableTime = new uint64_t[totalPlane];
  for (unsigned i = 0; i < totalPlane; i++)
    planeAvailableTime[i] = 0;
  if (palparam->pageRegNet == HB_NET || palparam->pageRegNet == HB_NET_OLD){
    flashportAvailableTime = new uint64_t[totalPlane];
    for (unsigned i = 0; i < totalPlane; i++)
      flashportAvailableTime[i] = 0;
  }
  if (palparam->pageRegNet == NO_NET || palparam->pageRegNet == FC_NET) 
    IOportTimeSlot = new set<uint64_t>[palparam->channel * palparam->package * 2];
  if (palparam->pageRegNet == HB_NET || palparam->pageRegNet == HB_NET_OLD)
    IOportTimeSlot = new set<uint64_t>[palparam->channel * palparam->package];
}

bool flash_interface_c::FindCandidateSlot(struct _pageregInternal *pageregInternal,
                    int &cache_idx, int &data_idx, uint64_t search_page, bool isWrite) {
  int cache_invalid_idx = (uint32_t)-1;
  int cache_lru_idx = (uint32_t)-1;
  uint64_t cache_minTime = (uint64_t)-1;
  int data_invalid_idx = (uint32_t)-1;
  int data_lru_idx = (uint32_t)-1;
  uint64_t data_minTime = (uint64_t)-1;
  // Find out hit slot
  if (isWrite == 0){ //read
    for (unsigned i = 0; i < palparam->readReg; i++) {
      if ((pageregInternal+i)->valid == true && 
                          search_page == (pageregInternal+i)->page){
        data_idx = i;
        return true;
      }
      if ((pageregInternal+i)->valid == false){
        data_invalid_idx = i;
      }
      else {
        if (data_minTime == (uint64_t)-1){
          data_minTime = (pageregInternal+i)->available_time;
          data_lru_idx = i;
        }
        else{
          if (data_minTime > (pageregInternal+i)->available_time){
            data_minTime = (pageregInternal+i)->available_time;
            data_lru_idx = i;
          }
        }      
      }
    }
    if (data_invalid_idx != (uint32_t)-1){
      data_idx = data_invalid_idx;
      return false;
    }
    else{
      data_idx = data_lru_idx;
      return false;
    }
  }
  else{ //write
    for (unsigned i = palparam->readReg; i < palparam->pageRegAssoc; i++) {
      if ((pageregInternal+i)->valid == true && 
                          search_page == (pageregInternal+i)->page){
        cache_idx = i;
        return true;
      }
      if ((pageregInternal+i)->valid == false){
        cache_invalid_idx = i;
      }
      else {
        if (cache_minTime == (uint64_t)-1){
          cache_minTime = (pageregInternal+i)->available_time;
          cache_lru_idx = i;
        }
        else{
          if (cache_minTime > (pageregInternal+i)->available_time){
            cache_minTime = (pageregInternal+i)->available_time;
            cache_lru_idx = i;
          }
        }      
      }
    }
    if (cache_invalid_idx != (uint32_t)-1){
      cache_idx = cache_invalid_idx;
      return false;
    }
    else{
      cache_idx = cache_lru_idx;
      for (unsigned i = 0; i < palparam->readReg; i++){
        if ((pageregInternal+i)->valid == false){
          data_invalid_idx = i;
        }
        else {
          if (data_minTime == (uint64_t)-1){
            data_minTime = (pageregInternal+i)->available_time;
            data_lru_idx = i;
          }
          else{
            if (data_minTime > (pageregInternal+i)->available_time){
              data_minTime = (pageregInternal+i)->available_time;
              data_lru_idx = i;
            }
          }      
        }        
      }
      if (data_invalid_idx != (uint32_t)-1)
        data_idx = data_invalid_idx;
      else
        data_idx = data_lru_idx;
      return false;
    }    
  }
}

bool flash_interface_c::FindCandidateSlot_HBNET(int readPlaneIdx, int reqPlaneIdx,
                    int &cache_idx, int &data_idx, uint64_t search_page, bool isWrite) {
  struct _pageregInternal *pageregGroup =  pageregInternal[reqPlaneIdx];                   
  int cache_invalid_idx = (uint32_t)-1;
  int cache_lru_idx = (uint32_t)-1;
  int cache_block_idx = (uint32_t)-1;
  uint64_t cache_minTime = (uint64_t)-1;
  int data_invalid_idx = (uint32_t)-1;
  int data_lru_idx = (uint32_t)-1;
  int data_block_idx = (uint32_t)-1;
  uint64_t data_minTime = (uint64_t)-1;
  uint64_t diff_minTime = (uint64_t)-1;
  // Find out hit slot
  if (isWrite == 0){ //read
    for (unsigned i = 0; i < palparam->readReg; i++) {
      if ((pageregGroup+i)->valid == true && 
                          search_page == (pageregGroup+i)->page){
        data_idx = i;
        return true;
      }
      if ((pageregGroup+i)->valid == false){
        data_invalid_idx = i;
      }
      else {
        if (flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc / palparam->pageReg + i / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)] <= planeAvailableTime[readPlaneIdx]){
          if (data_minTime == (uint64_t)-1){
            data_minTime = (pageregGroup+i)->available_time;
            data_lru_idx = i;
          }
          else{
            if (data_minTime > (pageregGroup+i)->available_time){
              data_minTime = (pageregGroup+i)->available_time;
              data_lru_idx = i;
            }
          } 
        }
        else{
          if (diff_minTime == (uint64_t)-1){
            diff_minTime = flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc / palparam->pageReg + i / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)] - planeAvailableTime[readPlaneIdx];
            data_block_idx = i;
          }
          else {
            if (diff_minTime > flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc / palparam->pageReg + i / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)] - planeAvailableTime[readPlaneIdx]){
              diff_minTime = flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc / palparam->pageReg + i / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)] - planeAvailableTime[readPlaneIdx];
              data_block_idx = i;              
            }
          }
        }
      }
    }
    if (data_invalid_idx != (uint32_t)-1){
      data_idx = data_invalid_idx;
      return false;
    }
    else if (data_lru_idx != (uint32_t)-1){
      data_idx = data_lru_idx;
      return false;
    }
    else {
      data_idx = data_block_idx;
      return false;
    }
  }
  // else{ //write
  //   for (unsigned i = palparam->readReg; i < palparam->pageRegAssoc; i++) {
  //     if ((pageregGroup+i)->valid == true && 
  //                         search_page == (pageregGroup+i)->page){
  //       cache_idx = i;
  //       return true;
  //     }
  //     if ((pageregGroup+i)->valid == false){
  //       cache_invalid_idx = i;
  //     }
  //     else {
  //       uint32_t evicted_ppn;
  //       uint32_t evicted_channel;
  //       uint32_t evicted_package;
  //       uint32_t evicted_die;
  //       uint32_t evicted_plane;     
  //       uint32_t evicted_block;
  //       uint32_t evicted_page;
  //       evicted_ppn = (pageregGroup+i)->ppn;
  //       evicted_page = evicted_ppn % palparam->page;
  //       evicted_ppn = evicted_ppn / palparam->page;
  //       evicted_block = evicted_ppn % palparam->block;
  //       evicted_ppn = evicted_ppn / palparam->block;
  //       evicted_plane = evicted_ppn % (palparam->plane * palparam->nandMultiapp);
  //       evicted_ppn = evicted_ppn / (palparam->plane * palparam->nandMultiapp);
  //       evicted_die = evicted_ppn % palparam->die;
  //       evicted_ppn = evicted_ppn / palparam->die;
  //       evicted_package = evicted_ppn % palparam->package;
  //       evicted_ppn = evicted_ppn / palparam->package; 
  //       evicted_channel = evicted_ppn % palparam->channel;          
  //       if (flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
  //                             / palparam->pageReg + (i - palparam->readReg)
  //                             / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)] <= planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)]){
  //         if (cache_minTime == (uint64_t)-1){
  //           cache_minTime = (pageregGroup+i)->available_time;
  //           cache_lru_idx = i;
  //         }
  //         else{
  //           if (cache_minTime > (pageregGroup+i)->available_time){
  //             cache_minTime = (pageregGroup+i)->available_time;
  //             cache_lru_idx = i;
  //           }
  //         } 
  //       }
  //       else{
  //         if (diff_minTime == (uint64_t)-1){
  //           diff_minTime = flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
  //                             / palparam->pageReg + (i - palparam->readReg)
  //                             / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)] - planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)];
  //           cache_block_idx = i;
  //         }
  //         else {
  //           if (diff_minTime > flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
  //                             / palparam->pageReg + (i - palparam->readReg)
  //                             / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)] - planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)]){
  //             diff_minTime = flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
  //                             / palparam->pageReg + (i - palparam->readReg)
  //                             / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)] - planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)];
  //             cache_block_idx = i;              
  //           }
  //         }
  //       }
  //     }
  //   }
  //   if (cache_invalid_idx != (uint32_t)-1){
  //     cache_idx = cache_invalid_idx;
  //     return false;
  //   }
  //   else if (cache_lru_idx != (uint32_t)-1){
  //     cache_idx = cache_lru_idx;
  //     return false;
  //   }
  //   else {
  //     cache_idx = cache_block_idx;
  //     return false;
  //   }      
  // }
  else{ //write
    for (unsigned i = palparam->readReg; i < palparam->pageRegAssoc; i++) {
      if ((pageregGroup+i)->valid == true && 
                          search_page == (pageregGroup+i)->page){
        cache_idx = i;
        return true;
      }
      if ((pageregGroup+i)->valid == false){
        cache_invalid_idx = i;
      }
      else {
        if (cache_minTime == (uint64_t)-1){
          cache_minTime = (pageregGroup+i)->available_time;
          cache_lru_idx = i;
        }
        else{
          if (cache_minTime > (pageregGroup+i)->available_time){
            cache_minTime = (pageregGroup+i)->available_time;
            cache_lru_idx = i;
          }
        }      
      }
    }
    if (cache_invalid_idx != (uint32_t)-1){
      cache_idx = cache_invalid_idx;
      return false;
    }
    else{
      cache_idx = cache_lru_idx;
      return false;
    }    
  }
}

void simplessd_interface_c::send(void) {
 mem_req_s *req = NIF_NETWORK->receive(MEM_MC, m_id);
  // if (!req)
  //   return;

  if (req) {
    //cout<<"Janalysis: flash-SSD latency "<<m_cycle - req->m_in<<endl;
    //Janalysis
    //req->m_in = m_cycle;
    
    NIF_NETWORK->receive_pop(MEM_MC, m_id);
    m_simBase->m_progress_checker->decrement_outstanding_layered_requests(7); //NIF_NETWORK
    if (req->m_type == MRT_WB){
      MEMORY->free_req(req->m_core_id, req);
      m_simBase->m_progress_checker->decrement_outstanding_layered_requests(MEM_MC);      
    }
    else{
      //cout << "Jie: send "<< req->m_id << endl; 
      m_output_buffer.push_back(req);
    }
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

      //req->m_in = m_cycle; //Jie_analysis
      bool insert_packet =
          NETWORK->send(req, MEM_MC, m_id, MEM_L3, req->m_cache_id[MEM_L3]);

      if (!insert_packet) {
        DEBUG("MC[%d] req:%d addr:0x%llx type:%s noc busy\n", m_id, req->m_id,
              req->m_addr, mem_req_c::mem_req_type_name[req->m_type]);
        break;
      } 
      m_simBase->m_progress_checker->increment_outstanding_layered_requests(6); //NETWORK
      m_simBase->m_progress_checker->decrement_outstanding_layered_requests(MEM_MC);
      m_simBase->m_progress_checker->increment_outstanding_layered_requests(MEM_L3);
      // cout << "ZJ: delete L" << MEM_MC << " m_id " << m_id << " req_id " << req->m_id << endl;
      // cout << "ZJ: insert L" << MEM_L3 << " m_id " << req->m_cache_id[MEM_L3] << " req_id " << req->m_id << endl;      

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
      //req->m_in = m_cycle;
      bool insert_packet =
          NIF_NETWORK->send(req, MEM_FLASH, req->m_cache_id[MEM_FLASH], MEM_MC, req->m_cache_id[MEM_MC]);
      if (!insert_packet) {
        DEBUG("MC[%d] req:%d addr:0x%llx type:%s noc busy\n", m_id, req->m_id,
              req->m_addr, mem_req_c::mem_req_type_name[req->m_type]);
        break;
      }
      m_simBase->m_progress_checker->increment_outstanding_layered_requests(7); //NIF_NETWORK
      m_simBase->m_progress_checker->decrement_outstanding_layered_requests(MEM_FLASH);
      m_simBase->m_progress_checker->increment_outstanding_layered_requests(MEM_MC);
      // cout << "ZJ: delete L" << MEM_FLASH << " m_id " << req->m_cache_id[MEM_FLASH] << " req_id " << req->m_id << endl;
      // cout << "ZJ: insert L" << MEM_MC << " m_id " << req->m_cache_id[MEM_MC] << " req_id " << req->m_id << endl;      
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
    // if (req->m_dirty) cout << "Jie: write request " << req->m_id << " type " << req->m_type << endl;
    // else cout << "Jie: read request " << req->m_id << " type " << req->m_type << endl;
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
    pHIL->collectPPN(req->m_appl_id, request, ppn, channel, package,  
                        die, plane, block, page, finishTick, 0);
    package = package + channel * palparam->package;
    req->m_cache_id[MEM_FLASH] = package;
    finishTick = m_cycle;
    // while (1){
    //   auto iter = m_input_buffer.find(finishTick);
    //   if (iter != m_input_buffer.end()){
    //     finishTick++;
    //     printf("Error in m_input_buffer\n");
    //   } 
    //   else break;
    // }  
    m_input_buffer.insert(pair<unsigned long long, mem_req_s *>(
      static_cast<unsigned long long>(finishTick), req));
    NETWORK->receive_pop(MEM_MC, m_id);
    m_simBase->m_progress_checker->decrement_outstanding_layered_requests(6); //NETWORK
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
      m_simBase->m_progress_checker->increment_outstanding_layered_requests(7); //NIF_NETWORK
      m_simBase->m_progress_checker->decrement_outstanding_layered_requests(MEM_MC);
      m_simBase->m_progress_checker->increment_outstanding_layered_requests(MEM_FLASH);      
      // cout << "ZJ: delete L" << MEM_MC << " m_id " << m_id << " req_id " << req->m_id << endl;
      // cout << "ZJ: insert L" << MEM_FLASH << " m_id " << req->m_cache_id[MEM_FLASH]<< " req_id " << req->m_id << endl;
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
    unsigned long long finishTime;
    if (req && insert_new_req(finishTime,req)) {
      NIF_NETWORK->receive_pop(MEM_FLASH, flash_id);
      m_simBase->m_progress_checker->decrement_outstanding_layered_requests(7); //NIF_NETWORK
      if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
        m_simBase->m_bug_detector->deallocate_noc(req);
      }
      m_simBase->m_progress_checker->increment_outstanding_requests();
    }


    // if (req){      
    //   //cout<<"Janalysis: SSD-flash latency "<<m_cycle - req->m_in<<endl;
    //   //Janalysis
    //   //req->m_in = m_cycle;
      
    //   // check if req has same slot in the input buffer
    //   bool input_buffer_hit = false;
    //   bool output_buffer_hit = false;
    //   for (auto I = m_input_buffer->begin(), E = m_input_buffer->end();
    //         I != E; I++){
    //     if ((I->second).front()->m_addr / logicalPageSize 
    //                                 == req->m_addr / logicalPageSize){
    //       input_buffer_hit = true;
    //       (I->second).push(req);
    //       NIF_NETWORK->receive_pop(MEM_FLASH, flash_id);
    //       m_simBase->m_progress_checker->decrement_outstanding_layered_requests(7); //NIF_NETWORK
    //       if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
    //         m_simBase->m_bug_detector->deallocate_noc(req);
    //       }
    //       m_simBase->m_progress_checker->increment_outstanding_requests();
    //       break;
    //     }
    //   }
    //   if (input_buffer_hit == false){
    //     for (auto I = m_output_buffer->begin(), E = m_output_buffer->end();
    //           I != E; I++){
    //       if (I->second->m_addr / logicalPageSize 
    //                                 == req->m_addr / logicalPageSize){
    //         unsigned long long tmp_time = I->first+1;
    //         while (1){
    //           auto iter = m_input_buffer->find(tmp_time);
    //           if (iter != m_input_buffer->end()) tmp_time++;
    //           else break;
    //         }
    //         queue<mem_req_s *> tmp_queue;
    //         tmp_queue.push(req);
    //         m_input_buffer->insert( pair<unsigned long long, queue<mem_req_s *>>(
    //               tmp_time, tmp_queue));
    //         output_buffer_hit = true;
    //         NIF_NETWORK->receive_pop(MEM_FLASH, flash_id);
    //         m_simBase->m_progress_checker->decrement_outstanding_layered_requests(7); //NIF_NETWORK
    //         if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
    //           m_simBase->m_bug_detector->deallocate_noc(req);
    //         }
    //         m_simBase->m_progress_checker->increment_outstanding_requests();
    //         break;
    //       }
    //     }    
    //   }
    //   if (input_buffer_hit == false && output_buffer_hit == false){
    //     unsigned long long finishTime;
    //     //cout<<"Janalysis: queue latency "<<m_cycle - req->m_in<<endl;
    //     //Janalysis
    //     //req->m_in = m_cycle;
        
    //     if (req && insert_new_req(finishTime,req)) {
    //       NIF_NETWORK->receive_pop(MEM_FLASH, flash_id);
    //       m_simBase->m_progress_checker->decrement_outstanding_layered_requests(7); //NIF_NETWORK
    //       if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
    //         m_simBase->m_bug_detector->deallocate_noc(req);
    //       }
    //       m_simBase->m_progress_checker->increment_outstanding_requests();
    //     }
    //   }  
    // }
    // auto I = m_input_buffer->begin();
    // auto E = m_input_buffer->end();
    // if ((I != E) && (I->first <= m_cycle)){
    //   unsigned long long finishTime;
    //   //cout<<"Janalysis: queue latency "<<m_cycle - (I->second).front()->m_in<<endl;
    //   (I->second).front()->m_in = m_cycle;  
    //   insert_new_req(finishTime,(I->second).front());
    //   finishTime++;
    //   if((I->second).size() == 1) m_input_buffer->erase(I);
    //   else{
    //     (I->second).pop();
    //     while (1){
    //       auto iter = m_input_buffer->find(finishTime);
    //       if (iter != m_input_buffer->end())finishTime++;
    //       else break;
    //     }
    //     queue<mem_req_s *> tmp_queue;
    //     while (!(I->second).empty()){
    //       tmp_queue.push((I->second).front());
    //       (I->second).pop();
    //     }
    //     m_input_buffer->erase(I);
    //     m_input_buffer->insert( pair<unsigned long long, queue<mem_req_s *>>(
    //           finishTime, tmp_queue));      
    //   }
    // }  
    
  }
}

uint32_t flash_interface_c::converttoPlaneIdx(uint32_t channel, uint32_t package,
    uint32_t die, uint32_t plane){
  uint32_t ret = 0;
  ret += plane;
  ret += die * (palparam->plane * palparam->nandMultiapp);
  ret += package * palparam->die 
                      * (palparam->plane * palparam->nandMultiapp);
  ret += channel * palparam->die * palparam->package
                    * (palparam->plane * palparam->nandMultiapp);
  return ret;  
}

uint32_t flash_interface_c::converttoDieIdx(uint32_t channel, uint32_t package,
    uint32_t die){
  uint32_t ret = 0;
  ret += die;
  ret += package * palparam->die;
  ret += channel * palparam->die * palparam->package;

  return ret;      
}

uint32_t flash_interface_c::converttoPackageIdx(uint32_t channel, uint32_t package){
  uint32_t ret = 0;
  ret += package;
  ret += channel * palparam->package;

  return ret;
}

void flash_interface_c::allocateIOport(int offset, int size, uint64_t &startTime, 
                                                        uint64_t &finishTime){
  int DMATime;
  uint64_t candidateTime = (uint64_t)-1;
  if (palparam->pageRegNet == NO_NET || palparam->pageRegNet == FC_NET) 
    DMATime = size / 8;
  else DMATime = size / 8 / 2;
  if (startTime % DMATime != 0) startTime = (startTime / DMATime + 1) * DMATime;
  while (1){
    auto iter = IOportTimeSlot[offset].find(startTime);
    if (iter != IOportTimeSlot[offset].end()) startTime += DMATime;
    else{
      IOportTimeSlot[offset].insert(startTime);
      break;
    } 
  }
  finishTime = startTime + DMATime;    
}

void flash_interface_c::cleanIOPort(){
  int numIOPort, numReg;
  if (palparam->pageRegNet == NO_NET || palparam->pageRegNet == FC_NET){
    numIOPort = palparam->channel * palparam->package * 2;
    numReg = totalPlane * palparam->pageReg / numIOPort;
  }     
  else{
    numIOPort = palparam->channel * palparam->package;
    numReg = totalPlane * palparam->pageReg / numIOPort;
  } 
  for (auto i = 0; i < numIOPort; i++){
    uint64_t minTime = (uint64_t)-1;
    // for (auto j = i * numReg; j < (i+1) * numReg; j++){
    //   if (minTime == (uint64_t)-1)
    //     minTime = ((*pageregInternal)+j)->available_time;
    //   else if (minTime > ((*pageregInternal)+j)->available_time)
    //     minTime = ((*pageregInternal)+j)->available_time;
    // }
    // printf("m_cycle %lu minTime %lu\n", m_cycle, minTime); 
    // if (minTime < m_cycle) minTime = m_cycle;
    minTime = m_cycle;
    for (auto iter = IOportTimeSlot[i].begin(); iter != IOportTimeSlot[i].end();){
      if (*iter < minTime) iter = IOportTimeSlot[i].erase(iter);
      else break;
    }    
  }

}

bool flash_interface_c::insert_new_req(unsigned long long &finishTime, 
                                             mem_req_s *mem_req) {
  cleanup_counter++;
  if (cleanup_counter == 1000){
    cleanup_counter = 0;
    cleanIOPort();
  }                                             
  SimpleSSD::ICL::Request request;
  request.reqID = mem_req->m_id;
  request.offset = mem_req->m_addr % logicalPageSize;
  request.length = mem_req->m_size;
  request.range.slpn = mem_req->m_addr / logicalPageSize;
  request.range.nlp = (mem_req->m_size + request.offset + logicalPageSize - 1) /
                      logicalPageSize;
  uint64_t finishTick = m_cycle;    
  uint64_t availableTime = m_cycle;                
  uint32_t ppn;
  uint32_t channel;
  uint32_t package;
  uint32_t die;
  uint32_t plane;
  uint32_t block;
  uint32_t page;
  uint32_t destPlane = (uint32_t)-1;
  SimpleSSD::Logger::info("Request %d arrived at %d cycle",
                        request.reqID, m_cycle); 
  availableTime = static_cast<unsigned long long>((double)availableTime * (double)1000 / (double)clock_freq);                                           
  pHIL->collectPPN(mem_req->m_appl_id, request, ppn, channel, package,  
                        die, plane, block, page, availableTime, 0);
  availableTime = static_cast<unsigned long long>((double)availableTime * (double)clock_freq / (double)1000);                   
  // if (mem_req->m_dirty == 0)
  //   pHIL->collectPPN(mem_req->m_appl_id, request, ppn, channel, package,  
  //                         die, plane, block, page, finishTick);
  // else
  //   pHIL->allocatePPN(mem_req->m_appl_id, request, ppn, channel, package,  
  //                         die, plane, block, page, finishTick);
  if (mem_req->m_dirty)
    printf("Jie: write lpn %lu ppn %u channel %u package %u die %u plane %u \
              block %u page %u\n", request.range.slpn, ppn, channel, package,
            die, plane, block, page);
  else
    printf("Jie: read lpn %lu ppn %u channel %u package %u die %u plane %u \
              block %u page %u\n", request.range.slpn, ppn, channel, package,
            die, plane, block, page);
  
  // if (mem_req->m_addr >= (unsigned long)UINT_MAX) 
  //                               printf("Jie: APP 1 app_id %d core_id %d lpn %lu\n", 
  //                                       mem_req->m_appl_id, mem_req->m_core_id,
  //                                                 request.range.slpn);
  // else printf("Jie: APP 0 app_id %d core_id %d lpn %lu\n", 
  //                                       mem_req->m_appl_id, mem_req->m_core_id,
  //                                                 request.range.slpn);

  uint32_t reqPlaneIdx = converttoPlaneIdx(channel, package, die, plane);
  reqPlaneIdx = reqPlaneIdx*palparam->pageReg / palparam->pageRegAssoc;
  uint64_t reqPageIdx = request.range.slpn;
  if (palparam->pageReg != 0){
    int candidateCacheIdx = -1, candidateDataIdx = -1;
    bool isHit =false;
    isHit = (palparam->pageRegNet == HB_NET)?
            FindCandidateSlot_HBNET(converttoPlaneIdx(channel, package, die, plane),
                        reqPlaneIdx, candidateCacheIdx,
                        candidateDataIdx, reqPageIdx, mem_req->m_dirty):
            FindCandidateSlot(pageregInternal[reqPlaneIdx], candidateCacheIdx,
                        candidateDataIdx, reqPageIdx, mem_req->m_dirty);
  if (isHit){      
      if (mem_req->m_dirty == 0)
        printf("flash_interface: Pagereg read hit @ index %d\n", candidateDataIdx);
      else
        printf("flash_interface: Pagereg write hit @ index %d\n", candidateCacheIdx);      
      if (mem_req->m_dirty == 0){
        assert(candidateCacheIdx == -1);
        assert(pageregInternal[reqPlaneIdx][candidateDataIdx].valid);
        if (availableTime < pageregInternal[reqPlaneIdx][candidateDataIdx].available_time)
          availableTime = pageregInternal[reqPlaneIdx][candidateDataIdx].available_time;
      }
      else{
        assert(candidateDataIdx == -1);
        assert(pageregInternal[reqPlaneIdx][candidateCacheIdx].valid);
        if ( availableTime < pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time)
          availableTime = pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time;
      } 
      switch (palparam->pageRegNet) {
        case NO_NET:
          allocateIOport( 
                converttoPackageIdx(channel, package) * 2 +
                converttoPlaneIdx(channel, package, die, plane) % 2,
                        mem_req->m_size, availableTime, finishTick);
          break;
        case FC_NET:
          allocateIOport( 
                converttoPackageIdx(channel, package) * 2 +
                converttoPlaneIdx(channel, package, die, plane) % 2,
                mem_req->m_size, availableTime, finishTick);        
          break;
        case HB_NET_OLD:
          allocateIOport( 
                converttoPackageIdx(channel, package),
                mem_req->m_size, availableTime, finishTick);       
          break;          
        case HB_NET:
          allocateIOport( 
                converttoPackageIdx(channel, package),
                mem_req->m_size, availableTime, finishTick);       
          break;
        default:
          printf("Wrong option for pageRegNet!\n");
          assert(0);
      }    
        
      if (mem_req->m_dirty == 0){ //read operation
        pageregInternal[reqPlaneIdx][candidateDataIdx].available_time = finishTick;
      }
      else { // write operation
        pageregInternal[reqPlaneIdx][candidateCacheIdx].dirty = true;
        pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time = finishTick;
        pageregInternal[reqPlaneIdx][candidateCacheIdx].usedSector |= 
          0x1 << (request.offset / 128);
        pageregInternal[reqPlaneIdx][candidateCacheIdx].reaccess++;          
      }
      //printf("Janalysis: flash latency %lu\n",0);
    }
    else { // no page registers hit
      if (mem_req->m_dirty == 0)
        printf("flash_interface: Pagereg read miss @ index %d\n", candidateDataIdx);
      else printf("flash_interface: Pagereg write miss @ index %d\n", candidateCacheIdx);
      if (mem_req->m_dirty == 0){
        assert(candidateCacheIdx == -1);
        if (availableTime < pageregInternal[reqPlaneIdx][candidateDataIdx].available_time)
          availableTime = pageregInternal[reqPlaneIdx][candidateDataIdx].available_time; 
        if (availableTime < planeAvailableTime[converttoPlaneIdx(channel, package, die, plane)])
          availableTime = planeAvailableTime[converttoPlaneIdx(channel, package, die, plane)];
        if ( palparam->pageRegNet == HB_NET || palparam->pageRegNet == HB_NET_OLD )
          if (availableTime < flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
                                    / palparam->pageReg + candidateDataIdx 
                                    / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)]){
            cout << "Jie_analysis: flash port delay " << flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc / palparam->pageReg + candidateDataIdx / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)] - availableTime << endl;                                                                       
            availableTime = flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
                                    / palparam->pageReg + candidateDataIdx 
                                    / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)];
          }
        // if ( palparam->pageRegNet == HB_NET )
        //   if (availableTime < flashportAvailableTime[converttoPlaneIdx(channel, package, die, plane)]){
        //     cout << "Jie_analysis: flash port delay " << flashportAvailableTime[converttoPlaneIdx(channel, package, die, plane)] - availableTime << endl;                                                                       
        //     availableTime = flashportAvailableTime[converttoPlaneIdx(channel, package, die, plane)];
        //   }        

        availableTime = static_cast<unsigned long long>((double)availableTime * (double)1000 / (double)clock_freq);        
        pHIL->schedulePPN(0, 0, (uint32_t &)request.offset, (uint32_t &)request.length, 
            ppn, channel, package, die, plane, block, page, availableTime);
        availableTime = static_cast<unsigned long long>((double)availableTime * (double)clock_freq / (double)1000);    
        planeAvailableTime[converttoPlaneIdx(channel, package, die, plane)] = availableTime;
        switch (palparam->pageRegNet) {
          case NO_NET:
            allocateIOport( 
                  converttoPackageIdx(channel, package) * 2 +
                  converttoPlaneIdx(channel, package, die, plane) % 2,
                          mem_req->m_size, availableTime, finishTick);
            break;
          case FC_NET:
            allocateIOport( 
                  converttoPackageIdx(channel, package) * 2 +
                  converttoPlaneIdx(channel, package, die, plane) % 2,
                  mem_req->m_size, availableTime, finishTick);        
            break;
          case HB_NET_OLD:
            allocateIOport( 
                  converttoPackageIdx(channel, package),
                  mem_req->m_size, availableTime, finishTick);       
            break;            
          case HB_NET:
            allocateIOport( 
                  converttoPackageIdx(channel, package),
                  mem_req->m_size, availableTime, finishTick);       
            break;
          default:
            printf("Wrong option for pageRegNet!\n");
            assert(0);
        } 
        if ( palparam->pageRegNet == HB_NET || palparam->pageRegNet == HB_NET_OLD )
          flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
                  / palparam->pageReg + candidateDataIdx 
                  / (palparam->readReg * palparam->pageReg / palparam->pageRegAssoc)] = availableTime;
        // if ( palparam->pageRegNet == HB_NET )
        //   flashportAvailableTime[converttoPlaneIdx(channel, package, die, plane)] = availableTime;        
        pageregInternal[reqPlaneIdx][candidateDataIdx].valid = true;
        pageregInternal[reqPlaneIdx][candidateDataIdx].ppn = ppn;
        pageregInternal[reqPlaneIdx][candidateDataIdx].page = reqPageIdx;
        pageregInternal[reqPlaneIdx][candidateDataIdx].dirty = false;
        pageregInternal[reqPlaneIdx][candidateDataIdx].available_time = finishTick;               
      }
      else{
        if (pageregInternal[reqPlaneIdx][candidateCacheIdx].valid &&  //need to evict dirty page
                                pageregInternal[reqPlaneIdx][candidateCacheIdx].dirty){
          switch (palparam->pageRegNet) {
            case NO_NET:
              if (availableTime < pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time)
                availableTime = pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time;
              if (availableTime < pageregInternal[reqPlaneIdx][candidateDataIdx].available_time)
                availableTime = pageregInternal[reqPlaneIdx][candidateDataIdx].available_time;
              availableTime += tBUSY;            
              allocateIOport( 
                    converttoPackageIdx(channel, package) * 2 +
                    converttoPlaneIdx(channel, package, die, plane) % 2,
                            mem_req->m_size, availableTime, finishTick);
              break;
            case FC_NET:
              if (availableTime < pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time)
                availableTime = pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time;            
              break;
            case HB_NET_OLD:
              if (availableTime < pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time)
                availableTime = pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time;                   
              break;              
            case HB_NET:
              if (availableTime < pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time)
                availableTime = pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time;                   
              break;
            default:
              printf("Wrong option for pageRegNet!\n");
              assert(0);
          }
          // uint32_t prev_plane = converttoPlaneIdx(channel, package, die, plane);
          // cout << "Jie_analysis: prev channel " << channel << " package " << package << " die " << die << " plane " << plane << " block " << block << " page " << page << endl;
          pHIL->allocatePPN(mem_req->m_appl_id, request, ppn, channel, package,  
                                                        die, plane, block, page, finishTick); 
          //Jie: analysis
          // cout << "Jie_analysis: new channel " << channel << " package " << package << " die " << die << " plane " << plane << " block " << block << " page " << page << endl;
          // if (prev_plane != converttoPlaneIdx(channel, package, die, plane)) 
          //   cout << "Jie_analysis: incorrect page allocation in FTL" << endl;       
          cout << "Jie_analysis: evicted pages lpn " <<
                          pageregInternal[reqPlaneIdx][candidateCacheIdx].page <<
                          " usedSector " << bitset<sizeof(int)*8>(
                          pageregInternal[reqPlaneIdx][candidateCacheIdx].usedSector) <<
                          " reaccess " <<
                          pageregInternal[reqPlaneIdx][candidateCacheIdx].reaccess << endl;
              
          uint32_t evicted_ppn;
          uint32_t evicted_channel;
          uint32_t evicted_package;
          uint32_t evicted_die;
          uint32_t evicted_plane;
          uint32_t evicted_block;
          uint32_t evicted_page;
          uint32_t evicted_offset = 0;
          uint32_t evicted_size = 4096;
          bool oper;
          oper = 1; //write
          evicted_ppn = pageregInternal[reqPlaneIdx][candidateCacheIdx].ppn;
          evicted_page = evicted_ppn % palparam->page;
          evicted_ppn = evicted_ppn / palparam->page;
          evicted_block = evicted_ppn % palparam->block;
          evicted_ppn = evicted_ppn / palparam->block;
          evicted_plane = evicted_ppn % (palparam->plane * palparam->nandMultiapp);
          evicted_ppn = evicted_ppn / (palparam->plane * palparam->nandMultiapp);
          evicted_die = evicted_ppn % palparam->die;
          evicted_ppn = evicted_ppn / palparam->die;
          evicted_package = evicted_ppn % palparam->package;
          evicted_ppn = evicted_ppn / palparam->package; 
          evicted_channel = evicted_ppn % palparam->channel;
          printf("flash_interface: Pagereg eviction @ ppn %u\n", pageregInternal[reqPlaneIdx][candidateCacheIdx].ppn);
          // if (converttoPackageIdx(evicted_channel, evicted_package) != converttoPackageIdx(channel, package))
          //   cout << "Jie_analysis: mismatch between evicted ppn and ppn" << endl; 
          if ( palparam->pageRegNet == HB_NET ) availableTime += 2 * tBUSY;
          if (availableTime < planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)])
            availableTime = planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)];           
          if ( palparam->pageRegNet == HB_NET_OLD )
            if (availableTime < flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
                                      / palparam->pageReg + (candidateCacheIdx - palparam->readReg)
                                      / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)]){
              cout << "Jie_analysis: flash port delay " << flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc / palparam->pageReg + (candidateCacheIdx - palparam->readReg) / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)] - availableTime << endl;
              availableTime = flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc 
                                      / palparam->pageReg + (candidateCacheIdx - palparam->readReg)
                                      / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)]; 
            }
          if ( palparam->pageRegNet == HB_NET )
            if (availableTime < flashportAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)]){
              cout << "Jie_analysis: flash port delay " << flashportAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)] - availableTime << endl;
              availableTime = flashportAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)]; 
            }

          availableTime =
                    static_cast<unsigned long long>((double)availableTime * (double)1000 / (double)clock_freq);          
          pHIL->schedulePPN(0, oper, evicted_offset, evicted_size, evicted_ppn, 
            evicted_channel, evicted_package, evicted_die, evicted_plane, evicted_block,
            evicted_page, availableTime);
          availableTime = static_cast<unsigned long long>((double)availableTime * (double)clock_freq / (double)1000);
          switch (palparam->pageRegNet) {
            case NO_NET:
              break;
            case FC_NET:
              allocateIOport( 
                    converttoPackageIdx(channel, package) * 2 +
                    converttoPlaneIdx(channel, package, die, plane) % 2,
                    mem_req->m_size, availableTime, finishTick);        
              break;
            case HB_NET_OLD:
              allocateIOport( 
                    converttoPackageIdx(channel, package),
                    mem_req->m_size, availableTime, finishTick);
              break;                 
            case HB_NET:
              allocateIOport( 
                    converttoPackageIdx(channel, package),
                    mem_req->m_size, availableTime, finishTick);
              break;            
            default:       
              printf("Wrong option for pageRegNet!\n");
              assert(0);
          }               
          if (palparam->pageRegNet == NO_NET){
            pageregInternal[reqPlaneIdx][candidateDataIdx].valid = false;
            pageregInternal[reqPlaneIdx][candidateDataIdx].ppn = 0;
            pageregInternal[reqPlaneIdx][candidateDataIdx].page = 0;
            pageregInternal[reqPlaneIdx][candidateDataIdx].dirty = false;
            pageregInternal[reqPlaneIdx][candidateDataIdx].available_time = availableTime; 
            planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)] = availableTime;          
            pageregInternal[reqPlaneIdx][candidateCacheIdx].valid = true;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].ppn = ppn;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].page = reqPageIdx;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].dirty = true;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time = finishTick;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].usedSector = 
              0x1 << (request.offset / 128); 
            pageregInternal[reqPlaneIdx][candidateCacheIdx].reaccess = 1;
          }
          else {
            if ( palparam->pageRegNet == HB_NET_OLD )
              flashportAvailableTime[reqPlaneIdx * palparam->pageRegAssoc / palparam->pageReg
                                       + (candidateCacheIdx - palparam->readReg)
                                      / ( (palparam->pageRegAssoc-palparam->readReg) * palparam->pageReg / palparam->pageRegAssoc)]
                                      = availableTime;        
            if ( palparam->pageRegNet == HB_NET )
              flashportAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)]
                                      = availableTime;                    
            planeAvailableTime[converttoPlaneIdx(evicted_channel, evicted_package, evicted_die, evicted_plane)] = availableTime;          
            pageregInternal[reqPlaneIdx][candidateCacheIdx].valid = true;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].ppn = ppn;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].page = reqPageIdx;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].dirty = true;
            pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time = finishTick;  
            pageregInternal[reqPlaneIdx][candidateCacheIdx].usedSector = 
              0x1 << (request.offset / 128); 
            pageregInternal[reqPlaneIdx][candidateCacheIdx].reaccess = 1;                       
          }

        }
        else {
          if (availableTime < pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time)
            availableTime = pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time;
          switch (palparam->pageRegNet) {
            case NO_NET:
              allocateIOport( 
                    converttoPackageIdx(channel, package) * 2 +
                    converttoPlaneIdx(channel, package, die, plane) % 2,
                            mem_req->m_size, availableTime, finishTick);
              break;
            case FC_NET:
              allocateIOport( 
                    converttoPackageIdx(channel, package) * 2 +
                    converttoPlaneIdx(channel, package, die, plane) % 2,
                    mem_req->m_size, availableTime, finishTick);        
              break;
            case HB_NET_OLD:
              allocateIOport( 
                    converttoPackageIdx(channel, package),
                    mem_req->m_size, availableTime, finishTick);       
              break;              
            case HB_NET:
              allocateIOport( 
                    converttoPackageIdx(channel, package),
                    mem_req->m_size, availableTime, finishTick);       
              break;
            default:
              printf("Wrong option for pageRegNet!\n");
              assert(0);
          } 
          // uint32_t prev_plane = converttoPlaneIdx(channel, package, die, plane);
          // cout << "Jie_analysis: prev channel " << channel << " package " << package << " die " << die << " plane " << plane << " block " << block << " page " << page << endl;
          pHIL->allocatePPN(mem_req->m_appl_id, request, ppn, channel, package,  
                                                        die, plane, block, page, finishTick);  
          //Jie: analysis
          // cout << "Jie_analysis: new channel " << channel << " package " << package << " die " << die << " plane " << plane << " block " << block << " page " << page << endl;
          // if (prev_plane != converttoPlaneIdx(channel, package, die, plane)) 
          //   cout << "Jie_analysis: incorrect page allocation in FTL" << endl;                                                                    
          pageregInternal[reqPlaneIdx][candidateCacheIdx].valid = true;
          pageregInternal[reqPlaneIdx][candidateCacheIdx].ppn = ppn;
          pageregInternal[reqPlaneIdx][candidateCacheIdx].page = reqPageIdx;
          pageregInternal[reqPlaneIdx][candidateCacheIdx].dirty = true;
          pageregInternal[reqPlaneIdx][candidateCacheIdx].available_time = finishTick; 
          pageregInternal[reqPlaneIdx][candidateCacheIdx].usedSector = 
            0x1 << (request.offset / 128); 
          pageregInternal[reqPlaneIdx][candidateCacheIdx].reaccess = 1;                           
        }          
      }
    }
  }
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
