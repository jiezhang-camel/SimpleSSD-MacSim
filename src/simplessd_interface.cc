#include "assert_macros.h"
#include "debug_macros.h"
#include "dram_ctrl.h"
#include "memory.h"
#include "memreq_info.h"
#include "utils.h"
#include "bug_detector.h"
#include "network.h"

#include "all_knobs.h"
#include "statistics.h"

#include "simplessd_interface.h"
#include <cstdio>


#define DEBUG(args...) _DEBUG(*m_simBase->m_knobs->KNOB_DEBUG_SSD, ## args)

dram_c* simplessd_interface(macsim_c* simBase)
{
  dram_c* simplessd_interface_tmp = new simplessd_interface_c(simBase);
  return simplessd_interface_tmp;
}

simplessd_interface_c::simplessd_interface_c(macsim_c* simBase) 
	: dram_c(simBase)
{
  cfg = new SimpleSSD::ConfigReader();
  SSD_config = (string)*m_simBase->m_knobs->KNOB_SIMPLESSD_CONFIG;
  cfg->init(SSD_config);
	clock_freq = *m_simBase->m_knobs->KNOB_CLOCK_MC;
  m_hil = new SimpleSSD::HIL::HIL(cfg);
	m_output_buffer = new map<unsigned long long, mem_req_s*>;
}

simplessd_interface_c::~simplessd_interface_c()
{
}

void simplessd_interface_c::init(int id)
{
  m_id = id;
}

void simplessd_interface_c::send(void)
{
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
    for (auto I = m_output_buffer->begin(), E = m_output_buffer->end(); I != E; ) {
       if (I->first > m_cycle) break;
	     mem_req_s* req = I->second;
         if (req_type_allowed[req->m_ptx] == false){
		   ++I;
		   continue;
	     }

      req_type_checked[req->m_ptx] = true;
      req->m_msg_type = NOC_FILL;
      
      bool insert_packet = NETWORK->send(req, MEM_MC, m_id, MEM_L3, req->m_cache_id[MEM_L3]);

      if (!insert_packet) {
        DEBUG("MC[%d] req:%d addr:0x%llx type:%s noc busy\n", 
            m_id, req->m_id, req->m_addr, mem_req_c::mem_req_type_name[req->m_type]);
        break;
      }

      if (*KNOB(KNOB_BUG_DETECTOR_ENABLE) && *KNOB(KNOB_ENABLE_NEW_NOC)) {
        m_simBase->m_bug_detector->allocate_noc(req);
      }
	    I = m_output_buffer->erase(I);
    }
  }

}

void simplessd_interface_c::receive(void)
{
  // check router queue every cycle
  mem_req_s* req = NETWORK->receive(MEM_MC, m_id);
  if (!req)
    return ;
  
  if (req && insert_new_req(req)) {
    NETWORK->receive_pop(MEM_MC, m_id);
    if (*KNOB(KNOB_BUG_DETECTOR_ENABLE)) {
      m_simBase->m_bug_detector->deallocate_noc(req);
    }
  }
}

bool simplessd_interface_c::insert_new_req(mem_req_s* mem_req)
{
  SimpleSSD::ICL::Request request;
  SimpleSSD::HIL::NVMe::Namespace::Information *info;

  uint64_t ssd_size = cfg->palConfig.getSSDSize() * (uint64_t)(1-cfg->ftlConfig.readFloat(SimpleSSD::FTL::FTL_OVERPROVISION_RATIO));
  uint64_t slba = (uint64_t)((mem_req->m_addr % (ssd_size)));

  uint32_t lbaratio = logicalPageSize / info->lbaSize;
  uint64_t slpn = slba / lbaratio;
  uint64_t off = slba % lbaratio;
  uint64_t nlp = (nlblk + off + lbaratio - 1) / lbaratio;

  request.reqID = mem_req->m_id;
  request.range.slpn = slpn + info->range.slpn;
  request.range.nlp = nlp;
  request.offset = off * info->lbaSize;
  request.length = nlblk * info->lbaSize;

  Tick finishTick = static_cast<unsigned long long>(m_cycle*1000/clock_freq);

  if (mem_req->m_dirty)
    m_hil->write(req, finishTick);
  else:
    m_hil->read(req, finishTick);

  m_output_buffer->insert(pair<unsigned long long, mem_req_s*>(\
        static_cast<unsigned long long>(curTick*clock_freq/1000), mem_req));

	return true;
}


// tick a cycle
void simplessd_interface_c::run_a_cycle(bool pll_lock)
{
  if (pll_lock) {
    ++m_cycle;
    return ;
  }
  send();

  receive();

  ++m_cycle;
}

void simplessd_interface_c::print_req(void)
{
  FILE* fp = fopen("bug_detect_dram.out", "w");

  fprintf(fp, "Current cycle:%llu\n", m_cycle);
  //fprintf(fp, "Total req:%d\n", m_total_req);
  fprintf(fp, "\n");
  fclose(fp);

//  g_memory->print_mshr();
}  

