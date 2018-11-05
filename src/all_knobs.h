#ifndef __ALL_KNOBS_H_INCLUDED__
#define __ALL_KNOBS_H_INCLUDED__

#include "global_types.h"
#include "knob.h"

#define KNOB(var) m_simBase->m_knobs->var

///////////////////////////////////////////////////////////////////////////////////////////////
/// \brief knob variables holder
///////////////////////////////////////////////////////////////////////////////////////////////
class all_knobs_c {
	public:
		/**
		 * Constructor
		 */
		all_knobs_c();

		/**
		 * Destructor
		 */
		~all_knobs_c();

		/**
		 * Register Knob Variables
		 */
		void registerKnobs(KnobsContainer *container);

	public:
		

	// =========== ../def/bp.param.def ===========
		KnobTemplate< uns >* KNOB_BP_HIST_LENGTH;
		KnobTemplate< uns >* KNOB_PHT_CTR_BITS;
		KnobTemplate< string >* KNOB_BP_DIR_MECH;
		KnobTemplate< uns >* KNOB_EXTRA_RECOVERY_CYCLES;
		KnobTemplate< bool >* KNOB_USE_BRANCH_PREDICTION;
		KnobTemplate< bool >* KNOB_PERFECT_BP;
		KnobTemplate< bool >* KNOB_PERFECT_BTB;
		KnobTemplate< uns >* KNOB_BTB_ENTRIES;
		KnobTemplate< uns >* KNOB_BTB_ASSOC;
		KnobTemplate< uns >* KNOB_BTB_BANK_NUM;
		KnobTemplate< bool >* KNOB_ENABLE_BTB;
		

	// =========== ../def/core.param.def ===========
		KnobTemplate< uns32 >* KNOB_FE_SIZE;
		KnobTemplate< uns32 >* KNOB_SCHED_CLOCK;
		KnobTemplate< uns32 >* KNOB_ALLOC_TO_EXEC_LATENCY;
		KnobTemplate< uns32 >* KNOB_GIAQ_SIZE;
		KnobTemplate< uns32 >* KNOB_MIAQ_SIZE;
		KnobTemplate< uns32 >* KNOB_FQ_SIZE;
		KnobTemplate< uns32 >* KNOB_GIAQ_MEDIUM_SIZE;
		KnobTemplate< uns32 >* KNOB_MIAQ_MEDIUM_SIZE;
		KnobTemplate< uns32 >* KNOB_FQ_MEDIUM_SIZE;
		KnobTemplate< uns32 >* KNOB_GIAQ_LARGE_SIZE;
		KnobTemplate< uns32 >* KNOB_MIAQ_LARGE_SIZE;
		KnobTemplate< uns32 >* KNOB_FQ_LARGE_SIZE;
		KnobTemplate< int >* KNOB_GEN_ALLOCQ_INDEX;
		KnobTemplate< int >* KNOB_MEM_ALLOCQ_INDEX;
		KnobTemplate< int >* KNOB_FLOAT_ALLOCQ_INDEX;
		KnobTemplate< bool >* KNOB_ONE_CYCLE_EXEC;
		KnobTemplate< uns32 >* KNOB_MAX_INSTS;
		KnobTemplate< uns32 >* KNOB_SIM_CYCLE_COUNT;
		KnobTemplate< uns64 >* KNOB_FORWARD_PROGRESS_LIMIT;
		KnobTemplate< int >* KNOB_MAX_BLOCK_PER_CORE_SUPER;
		KnobTemplate< uns16 >* KNOB_ROB_SIZE;
		KnobTemplate< uns16 >* KNOB_ROB_MEDIUM_SIZE;
		KnobTemplate< uns16 >* KNOB_ROB_LARGE_SIZE;
		KnobTemplate< uns16 >* KNOB_INT_REGFILE_SIZE;
		KnobTemplate< uns16 >* KNOB_FP_REGFILE_SIZE;
		KnobTemplate< uns16 >* KNOB_MEU_NSB;
		KnobTemplate< uns16 >* KNOB_MEU_NLB;
		KnobTemplate< uns16 >* KNOB_MEU_MEDIUM_NSB;
		KnobTemplate< uns16 >* KNOB_MEU_MEDIUM_NLB;
		KnobTemplate< uns16 >* KNOB_MEU_LARGE_NSB;
		KnobTemplate< uns16 >* KNOB_MEU_LARGE_NLB;
		KnobTemplate< uns16 >* KNOB_WIDTH;
		KnobTemplate< uns16 >* KNOB_MEDIUM_WIDTH;
		KnobTemplate< uns16 >* KNOB_LARGE_WIDTH;
		KnobTemplate< int >* KNOB_EXEC_RETIRE_LATENCY;
		KnobTemplate< bool >* KNOB_MEM_OBEY_STORE_DEP;
		KnobTemplate< bool >* KNOB_MEM_OOO_STORES;
		KnobTemplate< bool >* KNOB_USE_NEW_ORACLE;
		KnobTemplate< bool >* KNOB_IGNORE_DEP;
		KnobTemplate< uns >* KNOB_HEARTBEAT_INTERVAL;
		KnobTemplate< int >* KNOB_GPU_FETCH_RATIO;
		KnobTemplate< int >* KNOB_GPU_SCHEDULE_RATIO;
		KnobTemplate< int >* KNOB_CPU_FETCH_RATIO;
		KnobTemplate< int >* KNOB_PTX_EXEC_RATIO;
		KnobTemplate< int >* KNOB_PTX_INST_LATENCY;
		KnobTemplate< int >* KNOB_PTX_DISPATCH_LATENCY_FACTOR;
		KnobTemplate< int >* KNOB_NUM_SIM_CORES;
		KnobTemplate< string >* KNOB_CORE_TYPE;
		KnobTemplate< string >* KNOB_MEDIUM_CORE_TYPE;
		KnobTemplate< string >* KNOB_LARGE_CORE_TYPE;
		KnobTemplate< int >* KNOB_NUM_SIM_SMALL_CORES;
		KnobTemplate< int >* KNOB_NUM_SIM_LARGE_CORES;
		KnobTemplate< int >* KNOB_NUM_SIM_MEDIUM_CORES;
		KnobTemplate< int >* KNOB_MAX_THREADS_PER_CORE;
		KnobTemplate< int >* KNOB_MAX_THREADS_PER_MEDIUM_CORE;
		KnobTemplate< int >* KNOB_MAX_THREADS_PER_LARGE_CORE;
		KnobTemplate< string >* KNOB_SCHEDULE;
		KnobTemplate< string >* KNOB_MEDIUM_CORE_SCHEDULE;
		KnobTemplate< string >* KNOB_LARGE_CORE_SCHEDULE;
		KnobTemplate< uns32 >* KNOB_FETCH_LATENCY;
		KnobTemplate< uns32 >* KNOB_ALLOC_LATENCY;
		KnobTemplate< uns32 >* KNOB_MEDIUM_CORE_ALLOC_LATENCY;
		KnobTemplate< uns32 >* KNOB_MEDIUM_CORE_FETCH_LATENCY;
		KnobTemplate< uns32 >* KNOB_LARGE_CORE_ALLOC_LATENCY;
		KnobTemplate< uns32 >* KNOB_LARGE_CORE_FETCH_LATENCY;
		KnobTemplate< bool >* KNOB_PRINT_HEARTBEAT;
		KnobTemplate< bool >* KNOB_GPU_SCHED;
		KnobTemplate< bool >* KNOB_GPU_USE_SINGLE_ALLOCQ_TYPE;
		KnobTemplate< bool >* KNOB_GPU_SHARE_ALLOCQS_BETWEEN_THREADS;
		KnobTemplate< int >* KNOB_MAX_WARP_PER_SM;
		KnobTemplate< int >* KNOB_SP_PER_SM;
		KnobTemplate< int >* KNOB_SFU_PER_SM;
		KnobTemplate< int >* KNOB_LDST_PER_SM;
		KnobTemplate< int >* KNOB_32_64_ISA;
		KnobTemplate< int >* KNOB_PHY_ADDR_WIDTH;
		KnobTemplate< int >* KNOB_FEATURE_SIZE;
		KnobTemplate< int >* KNOB_PCIE_BUS_SIZE;
		KnobTemplate< int >* KNOB_PCIE_TR;
		KnobTemplate< int >* KNOB_PCIE_INIT;
		KnobTemplate< int >* KNOB_INST_LENGTH;
		KnobTemplate< int >* KNOB_OPCODE_WIDTH;
		KnobTemplate< int >* KNOB_MICRO_OPCODE_WIDTH;
		KnobTemplate< int >* KNOB_INST_BUF_SIZE;
		KnobTemplate< int >* KNOB_DEC_STREAM_BUF_SIZE;
		KnobTemplate< int >* KNOB_FP_INST_WINDOW_SIZE;
		KnobTemplate< int >* KNOB_RAS_SIZE;
		KnobTemplate< float >* KNOB_IFU_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_LSU_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_MEM_I_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_MEM_D_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_ALU_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_MUL_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_FPU_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_ALU_CDB_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_MUL_CDB_DUTY_CYCLE;
		KnobTemplate< float >* KNOB_FPU_CDB_DUTY_CYCLE;
		KnobTemplate< int >* KNOB_ICACHE_THROUGHPUT;
		KnobTemplate< int >* KNOB_DCACHE_THROUGHPUT;
		KnobTemplate< int >* KNOB_L2_THROUGHPUT;
		KnobTemplate< int >* KNOB_L3_THROUGHPUT;
		KnobTemplate< float >* KNOB_L2_CLOCKRATE;
		KnobTemplate< float >* KNOB_L3_CLOCKRATE;
		KnobTemplate< float >* KNOB_DRAM_CLOCKRATE;
		KnobTemplate< int >* KNOB_IS_GPU;
		KnobTemplate< int >* KNOB_GPU_WIDTH;
		KnobTemplate< int >* KNOB_ORIG_PIPELINE_STAGES;
		KnobTemplate< int >* KNOB_EI_DECODE_WIDTH;
		KnobTemplate< int >* KNOB_EI_ISSUE_WIDTH;
		KnobTemplate< int >* KNOB_EI_EXEC_WIDTH;
		KnobTemplate< int >* KNOB_EI_COMMIT_WIDTH;
		KnobTemplate< bool >* KNOB_IS_FERMI;
		KnobTemplate< bool >* KNOB_FENCE_ENABLE;
		KnobTemplate< bool >* KNOB_FENCE_PREF_ENABLE;
		KnobTemplate< bool >* KNOB_ACQ_REL;
		KnobTemplate< bool >* KNOB_USE_WB;
		KnobTemplate< int >* KNOB_WB_SIZE;
		KnobTemplate< bool >* KNOB_WB_FIFO;
		KnobTemplate< string >* KNOB_QSIM_STATE;
		KnobTemplate< string >* KNOB_QSIM_BENCH;
		

	// =========== ../def/debug.param.def ===========
		KnobTemplate< uns >* KNOB_DEBUG_CYCLE_START;
		KnobTemplate< uns >* KNOB_DEBUG_CYCLE_STOP;
		KnobTemplate< uns >* KNOB_DEBUG_INST_START;
		KnobTemplate< uns >* KNOB_DEBUG_INST_STOP;
		KnobTemplate< uns >* KNOB_DEBUG_FRONT_STAGE;
		KnobTemplate< uns >* KNOB_DEBUG_ALLOC_STAGE;
		KnobTemplate< uns >* KNOB_DEBUG_SCHEDULE_STAGE;
		KnobTemplate< uns >* KNOB_DEBUG_EXEC_STAGE;
		KnobTemplate< uns >* KNOB_DEBUG_DCU_STAGE;
		KnobTemplate< uns >* KNOB_DEBUG_RETIRE_STAGE;
		KnobTemplate< uns >* KNOB_DEBUG_MEM;
		KnobTemplate< uns >* KNOB_DEBUG_TRACE_READ;
		KnobTemplate< uns >* KNOB_DEBUG_SIM;
		KnobTemplate< uns >* KNOB_DEBUG_CACHE_LIB;
		KnobTemplate< uns >* KNOB_DEBUG_BP_DIR;
		KnobTemplate< uns >* KNOB_DEBUG_BTB;
		KnobTemplate< uns >* KNOB_DEBUG_MAP_STAGE;
		KnobTemplate< uns >* KNOB_DEBUG_PORT;
		KnobTemplate< int >* KNOB_DEBUG_CORE_ID;
		KnobTemplate< uns >* KNOB_DEBUG_SIM_THREAD_SCHEDULE;
		KnobTemplate< uns >* KNOB_DEBUG_DRAM;
		KnobTemplate< uns >* KNOB_DEBUG_PRINT_TRACE;
		KnobTemplate< bool >* KNOB_DEBUG_MEM_TRACE;
		KnobTemplate< bool >* KNOB_DEBUG_NOC;
		KnobTemplate< bool >* KNOB_DEBUG_HMC;
		

	// =========== ../def/dyfr.param.def ===========
		KnobTemplate< bool >* KNOB_ENABLE_DYFR;
		KnobTemplate< int >* KNOB_NUM_CPU_APPLICATION;
		KnobTemplate< int >* KNOB_DYFR_GPU_FREQ_MIN;
		KnobTemplate< int >* KNOB_DYFR_GPU_FREQ_MAX;
		KnobTemplate< int >* KNOB_DYFR_CPU_FREQ_MIN;
		KnobTemplate< int >* KNOB_DYFR_CPU_FREQ_MAX;
		KnobTemplate< int >* KNOB_DYFR_SAMPLE_PERIOD;
		KnobTemplate< int >* KNOB_DYFR_PLL_LOCK;
		KnobTemplate< int >* KNOB_DYFR_CPU_BUDGET;
		KnobTemplate< int >* KNOB_DYFR_GPU_BUDGET;
		KnobTemplate< int >* KNOB_DYFR_MEM_BUDGET;
		

	// =========== ../def/frontend.param.def ===========
		KnobTemplate< uns16 >* KNOB_FETCH_WDITH;
		KnobTemplate< uns16 >* KNOB_FETCH_MEDIUM_WDITH;
		KnobTemplate< uns16 >* KNOB_FETCH_LARGE_WDITH;
		KnobTemplate< bool >* KNOB_FETCH_ONLY_LOAD_READY;
		KnobTemplate< bool >* KNOB_FETCH_ONLY_SCHED_READY;
		KnobTemplate< bool >* KNOB_MT_NO_FETCH_BR;
		KnobTemplate< bool >* KNOB_NO_FETCH_ON_ICACHE_MISS;
		KnobTemplate< string >* KNOB_FETCH_POLICY;
		KnobTemplate< uns >* KNOB_DEC_RR_FREQ;
		KnobTemplate< uns >* KNOB_MT_STOP_FAIR_INIT;
		KnobTemplate< uns >* KNOB_FETCH_FAIR_PERIOD;
		KnobTemplate< uns >* KNOB_FETCH_FAIR_MERGE_TH;
		KnobTemplate< uns >* KNOB_FETCH_FAIR_TSHARE_TH;
		KnobTemplate< bool >* KNOB_FETCH_FAIR_MERGE;
		KnobTemplate< bool >* KNOB_FETCH_FAIR_TSHARE;
		KnobTemplate< uns >* KNOB_BLOCK_KEY_SIZE;
		KnobTemplate< uns >* KNOB_FETCH_FAIR_TSHARE_FREQ;
		KnobTemplate< int >* KNOB_NUM_INST_TO_FETCH_AFTER_LOAD;
		

	// =========== ../def/general.param.def ===========
		KnobTemplate< bool >* KNOB_ENABLE_ENERGY_INTROSPECTOR;
		KnobTemplate< int >* KNOB_POWER_PRINT_LEVEL;
		KnobTemplate< uns >* KNOB_BLOCKS_TO_SIMULATE;
		KnobTemplate< bool >* KNOB_SIMULATE_LAST_BLOCKS;
		KnobTemplate< bool >* KNOB_REPEAT_TRACE;
		KnobTemplate< int >* KNOB_REPEAT_TRACE_N;
		KnobTemplate< int >* KNOB_MAX_NUM_CORE_PER_APPL;
		KnobTemplate< int >* KNOB_MAX_BLOCKS_TO_SIMULATE;
		KnobTemplate< string >* KNOB_STDERR_FILE;
		KnobTemplate< string >* KNOB_STDOUT_FILE;
		KnobTemplate< string >* KNOB_FILE_TAG;
		KnobTemplate< string >* KNOB_STATISTICS_OUT_DIRECTORY;
		KnobTemplate< int >* KNOB_MAX_BLOCK_PER_CORE;
		KnobTemplate< uns64 >* KNOB_MAX_INSTS1;
		KnobTemplate< bool >* KNOB_ENABLE_CONDITIONAL_EXECUTION;
		KnobTemplate< int >* KNOB_TEMPERATURE;
		KnobTemplate< bool >* KNOB_ASSIGN_BLOCKS_GREEDILY_INITIALLY;
		KnobTemplate< float >* KNOB_CLOCK_CPU;
		KnobTemplate< float >* KNOB_CLOCK_GPU;
		KnobTemplate< float >* KNOB_CLOCK_L3;
		KnobTemplate< float >* KNOB_CLOCK_NOC;
		KnobTemplate< float >* KNOB_CLOCK_MC;
		KnobTemplate< float >* KNOB_COMPUTE_CAPABILITY;
		KnobTemplate< int >* KNOB_GPU_WARP_SIZE;
		KnobTemplate< bool >* KNOB_TRACE_USES_64_BIT_ADDR;
		KnobTemplate< bool >* KNOB_ENABLE_FAST_FORWARD_MODE;
		KnobTemplate< uns >* KNOB_FAST_FORWARD_MODE_THRESHOLD;
		

	// =========== ../def/hmc.param.def ===========
		KnobTemplate< bool >* KNOB_ENABLE_HMC_INST;
		KnobTemplate< bool >* KNOB_ENABLE_HMC_INST_SKIP;
		KnobTemplate< bool >* KNOB_ENABLE_HMC_BYPASS_CACHE;
		KnobTemplate< bool >* KNOB_ENABLE_NONHMC_STAT;
		KnobTemplate< bool >* KNOB_ENABLE_LOCK_SKIP;
		KnobTemplate< bool >* KNOB_ENABLE_HMC_TRANS;
		KnobTemplate< bool >* KNOB_ENABLE_HMC_FENCE;
		KnobTemplate< bool >* KNOB_ENABLE_HMC_DOUBLE_FENCE;
		KnobTemplate< bool >* KNOB_COUNT_HMC_REMOVED_IN_MAX_INSTS;
		KnobTemplate< bool >* KNOB_HMC_ADD_DEP;
		KnobTemplate< bool >* KNOB_ENABLE_TRACE_MAX_THREAD;
		KnobTemplate< int >* KNOB_TRACE_MAX_THREAD_COUNT;
		KnobTemplate< int >* KNOB_HMC_TEST_OVERHEAD;
		

	// =========== ../def/memory.param.def ===========
		KnobTemplate< bool >* KNOB_PERFECT_ICACHE;
		KnobTemplate< uns >* KNOB_ICACHE_NUM_SET;
		KnobTemplate< uns >* KNOB_ICACHE_ASSOC;
		KnobTemplate< uns >* KNOB_ICACHE_LINE_SIZE;
		KnobTemplate< uns >* KNOB_ICACHE_BANKS;
		KnobTemplate< bool >* KNOB_ICACHE_BY_PASS;
		KnobTemplate< uns >* KNOB_ICACHE_CYCLES;
		KnobTemplate< uns >* KNOB_ICACHE_MEDIUM_NUM_SET;
		KnobTemplate< uns >* KNOB_ICACHE_MEDIUM_ASSOC;
		KnobTemplate< uns >* KNOB_ICACHE_MEDIUM_LINE_SIZE;
		KnobTemplate< uns >* KNOB_ICACHE_MEDIUM_BANKS;
		KnobTemplate< bool >* KNOB_ICACHE_MEDIUM_BY_PASS;
		KnobTemplate< uns >* KNOB_ICACHE_MEDIUM_CYCLES;
		KnobTemplate< uns >* KNOB_ICACHE_LARGE_NUM_SET;
		KnobTemplate< uns >* KNOB_ICACHE_LARGE_ASSOC;
		KnobTemplate< uns >* KNOB_ICACHE_LARGE_LINE_SIZE;
		KnobTemplate< uns >* KNOB_ICACHE_LARGE_BANKS;
		KnobTemplate< bool >* KNOB_ICACHE_LARGE_BY_PASS;
		KnobTemplate< uns >* KNOB_ICACHE_LARGE_CYCLES;
		KnobTemplate< uns >* KNOB_ICACHE_READ_PORTS;
		KnobTemplate< uns >* KNOB_ICACHE_WRITE_PORTS;
		KnobTemplate< bool >* KNOB_PERFECT_DCACHE;
		KnobTemplate< int >* KNOB_L1_READ_PORTS;
		KnobTemplate< int >* KNOB_L2_READ_PORTS;
		KnobTemplate< int >* KNOB_L3_READ_PORTS;
		KnobTemplate< int >* KNOB_L1_WRITE_PORTS;
		KnobTemplate< int >* KNOB_L2_WRITE_PORTS;
		KnobTemplate< int >* KNOB_L3_WRITE_PORTS;
		KnobTemplate< int >* KNOB_L1_SMALL_NUM_SET;
		KnobTemplate< int >* KNOB_L1_SMALL_ASSOC;
		KnobTemplate< int >* KNOB_L1_SMALL_LINE_SIZE;
		KnobTemplate< int >* KNOB_L1_SMALL_NUM_BANK;
		KnobTemplate< int >* KNOB_L1_SMALL_LATENCY;
		KnobTemplate< bool >* KNOB_L1_SMALL_BYPASS;
		KnobTemplate< int >* KNOB_L2_SMALL_NUM_SET;
		KnobTemplate< int >* KNOB_L2_SMALL_ASSOC;
		KnobTemplate< int >* KNOB_L2_SMALL_LINE_SIZE;
		KnobTemplate< int >* KNOB_L2_SMALL_NUM_BANK;
		KnobTemplate< int >* KNOB_L2_SMALL_LATENCY;
		KnobTemplate< bool >* KNOB_L2_SMALL_BYPASS;
		KnobTemplate< int >* KNOB_L1_MEDIUM_NUM_SET;
		KnobTemplate< int >* KNOB_L1_MEDIUM_ASSOC;
		KnobTemplate< int >* KNOB_L1_MEDIUM_LINE_SIZE;
		KnobTemplate< int >* KNOB_L1_MEDIUM_NUM_BANK;
		KnobTemplate< int >* KNOB_L1_MEDIUM_LATENCY;
		KnobTemplate< bool >* KNOB_L1_MEDIUM_BYPASS;
		KnobTemplate< int >* KNOB_L2_MEDIUM_NUM_SET;
		KnobTemplate< int >* KNOB_L2_MEDIUM_ASSOC;
		KnobTemplate< int >* KNOB_L2_MEDIUM_LINE_SIZE;
		KnobTemplate< int >* KNOB_L2_MEDIUM_NUM_BANK;
		KnobTemplate< int >* KNOB_L2_MEDIUM_LATENCY;
		KnobTemplate< bool >* KNOB_L2_MEDIUM_BYPASS;
		KnobTemplate< int >* KNOB_L1_LARGE_NUM_SET;
		KnobTemplate< int >* KNOB_L1_LARGE_ASSOC;
		KnobTemplate< int >* KNOB_L1_LARGE_LINE_SIZE;
		KnobTemplate< int >* KNOB_L1_LARGE_NUM_BANK;
		KnobTemplate< int >* KNOB_L1_LARGE_LATENCY;
		KnobTemplate< bool >* KNOB_L1_LARGE_BYPASS;
		KnobTemplate< int >* KNOB_L2_LARGE_NUM_SET;
		KnobTemplate< int >* KNOB_L2_LARGE_ASSOC;
		KnobTemplate< int >* KNOB_L2_LARGE_LINE_SIZE;
		KnobTemplate< int >* KNOB_L2_LARGE_NUM_BANK;
		KnobTemplate< int >* KNOB_L2_LARGE_LATENCY;
		KnobTemplate< bool >* KNOB_L2_LARGE_BYPASS;
		KnobTemplate< int >* KNOB_NUM_L3;
		KnobTemplate< int >* KNOB_L3_INTERLEAVE_FACTOR;
		KnobTemplate< int >* KNOB_L3_NUM_SET;
		KnobTemplate< int >* KNOB_L3_ASSOC;
		KnobTemplate< int >* KNOB_L3_LINE_SIZE;
		KnobTemplate< int >* KNOB_L3_NUM_BANK;
		KnobTemplate< int >* KNOB_L3_LATENCY;
		KnobTemplate< bool >* KNOB_DCACHE_INFINITE_PORT;
		KnobTemplate< string >* KNOB_SIMPLESSD_CONFIG;
		KnobTemplate< int >* KNOB_DRAM_BUFFER_SIZE;
		KnobTemplate< bool >* KNOB_DRAM_BANK_XOR_INDEX;
		KnobTemplate< bool >* KNOB_DRAM_MERGE_REQUESTS;
		KnobTemplate< int >* KNOB_DRAM_ROWBUFFER_SIZE;
		KnobTemplate< string >* KNOB_DRAM_SCHEDULING_POLICY;
		KnobTemplate< int >* KNOB_DRAM_NUM_CHANNEL;
		KnobTemplate< int >* KNOB_DRAM_NUM_BANKS;
		KnobTemplate< int >* KNOB_DRAM_NUM_ROWS;
		KnobTemplate< uns >* KNOB_DRAM_ONE_CYCLE;
		KnobTemplate< uns >* KNOB_DRAM_DDR_FACTOR;
		KnobTemplate< uns >* KNOB_DRAM_BUS_WIDTH;
		KnobTemplate< uns >* KNOB_DRAM_COLUMN;
		KnobTemplate< uns >* KNOB_DRAM_PRECHARGE;
		KnobTemplate< uns >* KNOB_DRAM_ACTIVATE;
		KnobTemplate< int >* KNOB_DRAM_NUM_MC;
		KnobTemplate< int >* KNOB_DRAM_INTERLEAVE_FACTOR;
		KnobTemplate< int >* KNOB_DRAM_ADDITIONAL_LATENCY;
		KnobTemplate< string >* KNOB_MEMORY_TYPE;
		KnobTemplate< int >* KNOB_MEM_MSHR_SIZE;
		KnobTemplate< int >* KNOB_MEM_QUEUE_SIZE;
		KnobTemplate< bool >* KNOB_ENABLE_PREF_SMALL_CORE;
		KnobTemplate< bool >* KNOB_ENABLE_PREF_MEDIUM_CORE;
		KnobTemplate< bool >* KNOB_ENABLE_PREF_LARGE_CORE;
		KnobTemplate< int >* KNOB_MEM_SIZE_AMP;
		KnobTemplate< bool >* KNOB_PTX_COMMON_CACHE;
		KnobTemplate< int >* KNOB_MAX_TRANSACTION_SIZE;
		KnobTemplate< bool >* KNOB_BYTE_LEVEL_ACCESS;
		KnobTemplate< bool >* KNOB_INFINITE_PORT;
		KnobTemplate< int >* KNOB_EXTRA_LD_LATENCY;
		KnobTemplate< bool >* KNOB_USE_CONST_AND_TEX_CACHES;
		KnobTemplate< uns32 >* KNOB_CONST_CACHE_SIZE;
		KnobTemplate< uns8 >* KNOB_CONST_CACHE_ASSOC;
		KnobTemplate< uns8 >* KNOB_CONST_CACHE_LINE_SIZE;
		KnobTemplate< uns8 >* KNOB_CONST_CACHE_BANKS;
		KnobTemplate< uns8 >* KNOB_CONST_CACHE_CYCLES;
		KnobTemplate< uns32 >* KNOB_TEXTURE_CACHE_SIZE;
		KnobTemplate< uns8 >* KNOB_TEXTURE_CACHE_ASSOC;
		KnobTemplate< uns8 >* KNOB_TEXTURE_CACHE_LINE_SIZE;
		KnobTemplate< uns8 >* KNOB_TEXTURE_CACHE_BANKS;
		KnobTemplate< uns8 >* KNOB_TEXTURE_CACHE_CYCLES;
		KnobTemplate< uns32 >* KNOB_SHARED_MEM_SIZE;
		KnobTemplate< uns8 >* KNOB_SHARED_MEM_ASSOC;
		KnobTemplate< uns8 >* KNOB_SHARED_MEM_LINE_SIZE;
		KnobTemplate< uns8 >* KNOB_SHARED_MEM_BANKS;
		KnobTemplate< uns8 >* KNOB_SHARED_MEM_CYCLES;
		KnobTemplate< uns >* KNOB_SHARED_MEM_PORTS;
		KnobTemplate< bool >* KNOB_ENABLE_CACHE_COHERENCE;
		KnobTemplate< string >* KNOB_LLC_TYPE;
		KnobTemplate< int >* KNOB_COLLECT_CACHE_INFO;
		KnobTemplate< bool >* KNOB_HETERO_STATIC_CACHE_PARTITION;
		KnobTemplate< int >* KNOB_HETERO_STATIC_CPU_PARTITION;
		KnobTemplate< int >* KNOB_HETERO_STATIC_GPU_PARTITION;
		KnobTemplate< int >* KNOB_HETERO_GPU_CORE_DISABLE;
		KnobTemplate< bool >* KNOB_HETERO_NOC_USE_SAME_QUEUE;
		KnobTemplate< bool >* KNOB_HETERO_MEM_PRIORITY_CPU;
		KnobTemplate< bool >* KNOB_HETERO_MEM_PRIORITY_GPU;
		KnobTemplate< bool >* KNOB_CACHE_USE_PSEUDO_LRU;
		KnobTemplate< int >* KNOB_LOAD_BUF_SIZE;
		KnobTemplate< int >* KNOB_STORE_BUF_SIZE;
		KnobTemplate< bool >* KNOB_USE_INCOMING_TID_CID_FOR_WB;
		KnobTemplate< bool >* KNOB_DEFAULT_INTERLEAVING;
		KnobTemplate< bool >* KNOB_NEW_INTERLEAVING_SAME_GRANULARITY;
		KnobTemplate< bool >* KNOB_NEW_INTERLEAVING_DIFF_GRANULARITY;
		KnobTemplate< bool >* KNOB_USE_NEW_COALESCING;
		KnobTemplate< bool >* KNOB_ENABLE_PHYSICAL_MAPPING;
		KnobTemplate< int >* KNOB_PAGE_SIZE;
		KnobTemplate< int >* KNOB_REGION_SIZE;
		KnobTemplate< string >* KNOB_PAGE_MAPPING_POLICY;
		KnobTemplate< bool >* KNOB_USE_MEMHIERARCHY;
		KnobTemplate< bool >* KNOB_USE_VAULTSIM_LINK;
		

	// =========== ../def/network.param.def ===========
		KnobTemplate< bool >* KNOB_ENABLE_IRIS;
		KnobTemplate< string >* KNOB_IRIS_TOPOLOGY;
		KnobTemplate< string >* KNOB_IRIS_GRIDSIZE;
		KnobTemplate< string >* KNOB_IRIS_NUM_VC;
		KnobTemplate< string >* KNOB_IRIS_CREDIT;
		KnobTemplate< string >* KNOB_IRIS_INT_BUFF_WIDTH;
		KnobTemplate< string >* KNOB_IRIS_LINK_WIDTH;
		KnobTemplate< string >* KNOB_IRIS_RC_METHOD;
		KnobTemplate< string >* KNOB_IRIS_SELF_ASSIGN_DEST_ID;
		KnobTemplate< string >* KNOB_IRIS_RESP_PAYLOAD_LEN;
		KnobTemplate< string >* KNOB_IRIS_MEMORY_LATENCY;
		KnobTemplate< bool >* KNOB_ENABLE_NEW_NOC;
		KnobTemplate< int >* KNOB_NUM_VC;
		KnobTemplate< int >* KNOB_NUM_PORT;
		KnobTemplate< int >* KNOB_LINK_LATENCY;
		KnobTemplate< int >* KNOB_LINK_WIDTH;
		KnobTemplate< int >* KNOB_NOC_DIMENSION;
		KnobTemplate< string >* KNOB_NOC_TOPOLOGY;
		KnobTemplate< bool >* KNOB_ENABLE_HETEROGENEOUS_LINK;
		KnobTemplate< int >* KNOB_NUM_SWITCH;
		KnobTemplate< int >* KNOB_NUM_SWITCH_CPU;
		KnobTemplate< int >* KNOB_NUM_SWITCH_GPU;
		KnobTemplate< int >* KNOB_NUM_SWITCH_MEM;
		KnobTemplate< bool >* KNOB_ENABLE_HETEROGENEOUS_LINK_WIDTH;
		KnobTemplate< int >* KNOB_NUM_SWITCH_ITER;
		KnobTemplate< int >* KNOB_NUM_SWITCH_ITER_CPU;
		KnobTemplate< int >* KNOB_NUM_SWITCH_ITER_GPU;
		KnobTemplate< int >* KNOB_NUM_SWITCH_ITER_MEM;
		KnobTemplate< bool >* KNOB_ENABLE_NOC_VC_PARTITION;
		KnobTemplate< int >* KNOB_CPU_VC_PARTITION;
		KnobTemplate< int >* KNOB_GPU_VC_PARTITION;
		KnobTemplate< bool >* KNOB_ENABLE_CHANNEL_PARTITION;
		KnobTemplate< int >* KNOB_NUM_CHANNEL_CPU;
		KnobTemplate< int >* KNOB_NUM_CHANNEL_GPU;
		KnobTemplate< bool >* KNOB_ENABLE_ADAPTIVE_RING_ROUTING;
		KnobTemplate< int >* KNOB_ARR_THRESHOLD;
		KnobTemplate< bool >* KNOB_ARR_DELTA;
		KnobTemplate< bool >* KNOB_ARR_ADV;
		KnobTemplate< bool >* KNOB_ARR_100;
		KnobTemplate< int >* KNOB_ARBITRATION_POLICY;
		KnobTemplate< int >* KNOB_ROUTER_PLACEMENT;
		KnobTemplate< int >* KNOB_CORE_ENABLE_BEGIN;
		KnobTemplate< int >* KNOB_CORE_ENABLE_END;
		KnobTemplate< bool >* KNOB_IDEAL_NOC;
		KnobTemplate< int >* KNOB_IDEAL_NOC_LATENCY;
		KnobTemplate< bool >* KNOB_USE_ZERO_LATENCY_NOC;
		

	// =========== ../def/pref.param.def ===========
		KnobTemplate< bool >* KNOB_PREF_FRAMEWORK_ON;
		KnobTemplate< bool >* KNOB_PREF_TRACE_ON;
		KnobTemplate< bool >* KNOB_DEBUG_PREF;
		KnobTemplate< int >* KNOB_PREF_DL0REQ_QUEUE_SIZE;
		KnobTemplate< int >* KNOB_PREF_UL1REQ_QUEUE_SIZE;
		KnobTemplate< bool >* KNOB_PREF_DL0_MISS_ON;
		KnobTemplate< bool >* KNOB_PREF_DL0_HIT_ON;
		KnobTemplate< bool >* KNOB_PREF_DL0REQ_QUEUE_FILTER_ON;
		KnobTemplate< bool >* KNOB_PREF_UL1REQ_QUEUE_FILTER_ON;
		KnobTemplate< bool >* KNOB_PREF_DL0REQ_ADD_FILTER_ON;
		KnobTemplate< bool >* KNOB_PREF_UL1REQ_ADD_FILTER_ON;
		KnobTemplate< bool >* KNOB_PREF_DL0REQ_QUEUE_OVERWRITE_ON_FULL;
		KnobTemplate< bool >* KNOB_PREF_UL1REQ_QUEUE_OVERWRITE_ON_FULL;
		KnobTemplate< int >* KNOB_PREF_DL0SCHEDULE_NUM;
		KnobTemplate< int >* KNOB_PREF_UL1SCHEDULE_NUM;
		KnobTemplate< bool >* KNOB_PREF_REGION_ON;
		KnobTemplate< bool >* KNOB_PREF_USEREGION_TOCALC_ACC;
		KnobTemplate< int >* KNOB_PREF_NUMTRACKING_REGIONS;
		KnobTemplate< int >* KNOB_PREF_REGION_SIZE;
		KnobTemplate< bool >* KNOB_PREF_HYBRID_ON;
		KnobTemplate< uns8 >* KNOB_PREF_HYBRID_DEFAULT;
		KnobTemplate< uns64 >* KNOB_PREF_HYBRID_DEFAULT_TIMEPERIOD;
		KnobTemplate< uns >* KNOB_PREF_HYBRID_UPDATE_MULTIPLE;
		KnobTemplate< uns >* KNOB_PREF_HYBRID_MIN_SENT;
		KnobTemplate< uns >* KNOB_PREF_HYBRID_MIN_MEMUSED;
		KnobTemplate< bool >* KNOB_PREF_HYBRID_SORT_ON_ACC;
		KnobTemplate< bool >* KNOB_PREF_HYBRID_SORT_ON_COV;
		KnobTemplate< float >* KNOB_PREF_ACC_THRESH_1;
		KnobTemplate< float >* KNOB_PREF_ACC_THRESH_2;
		KnobTemplate< float >* KNOB_PREF_ACC_THRESH_3;
		KnobTemplate< uns64 >* KNOB_PREF_UPDATE_INTERVAL;
		KnobTemplate< bool >* KNOB_PREF_ACC_STUDY;
		KnobTemplate< uns64 >* KNOB_PREF_ACC_UPDATE_INTERVAL;
		KnobTemplate< bool >* KNOB_PREF_ANALYZE_LOAD;
		KnobTemplate< float >* KNOB_PREF_POL_THRESH_1;
		KnobTemplate< float >* KNOB_PREF_POL_THRESH_2;
		KnobTemplate< bool >* KNOB_PREF_POLBV_ON;
		KnobTemplate< uns >* KNOB_PREF_POLBV_SIZE;
		KnobTemplate< uns >* KNOB_LOG2_PREF_POLBV_SIZE;
		KnobTemplate< float >* KNOB_PREF_TIMELY_THRESH;
		KnobTemplate< float >* KNOB_PREF_POLPF_THRESH;
		KnobTemplate< bool >* KNOB_PREF_DEGFB_USEONLYACC;
		KnobTemplate< bool >* KNOB_PREF_DEGFB_USEONLYPOL;
		KnobTemplate< bool >* KNOB_PREF_DEGFB_USEONLYLATE;
		KnobTemplate< float >* KNOB_PREF_TIMELY_THRESH_2;
		KnobTemplate< bool >* KNOB_PREF_DEGFB_STATPHASEFILE;
		KnobTemplate< bool >* KNOB_PREF_DHAL;
		KnobTemplate< uns >* KNOB_PREF_DHAL_SENTTHRESH;
		KnobTemplate< uns >* KNOB_PREF_DHAL_USETHRESH_MAX;
		KnobTemplate< uns >* KNOB_PREF_DHAL_USETHRESH_MIN2;
		KnobTemplate< uns >* KNOB_PREF_DHAL_USETHRESH_MIN1;
		KnobTemplate< uns >* KNOB_PREF_DHAL_MAXDEG;
		KnobTemplate< bool >* KNOB_PREF_THREAD_INDEX;
		KnobTemplate< bool >* KNOB_PREF_TRAIN_INST_ONCE;
		KnobTemplate< bool >* KNOB_DEBUG_PREF_STRIDE;
		KnobTemplate< int >* KNOB_PREF_STRIDE_TABLE_N;
		KnobTemplate< uns >* KNOB_PREF_STRIDE_REGION_BITS;
		KnobTemplate< int >* KNOB_PREF_STRIDE_DEGREE;
		KnobTemplate< int >* KNOB_PREF_STRIDE_DISTANCE;
		KnobTemplate< int >* KNOB_PREF_STRIDE_STARTDISTANCE;
		KnobTemplate< int >* KNOB_PREF_STRIDE_SINGLE_THRESH;
		KnobTemplate< int >* KNOB_PREF_STRIDE_MULTI_THRESH;
		KnobTemplate< bool >* KNOB_PREF_STRIDE_SINGLE_STRIDE_MODE;
		KnobTemplate< bool >* KNOB_PREF_STRIDE_ON;
		KnobTemplate< bool >* KNOB_PREF_STRIDE_ON_MEDIUM_CORE;
		KnobTemplate< bool >* KNOB_PREF_STRIDE_ON_LARGE_CORE;
		

	// =========== ../def/ramulator.param.def ===========
		KnobTemplate< string >* KNOB_RAMULATOR_CONFIG_FILE;
		KnobTemplate< int >* KNOB_RAMULATOR_CACHELINE_SIZE;
		

	// =========== ../def/schedule.param.def ===========
		KnobTemplate< uns16 >* KNOB_ISCHED_SIZE;
		KnobTemplate< uns16 >* KNOB_MSCHED_SIZE;
		KnobTemplate< uns16 >* KNOB_FSCHED_SIZE;
		KnobTemplate< uns16 >* KNOB_ISCHED_MEDIUM_SIZE;
		KnobTemplate< uns16 >* KNOB_MSCHED_MEDIUM_SIZE;
		KnobTemplate< uns16 >* KNOB_FSCHED_MEDIUM_SIZE;
		KnobTemplate< uns16 >* KNOB_ISCHED_LARGE_SIZE;
		KnobTemplate< uns16 >* KNOB_MSCHED_LARGE_SIZE;
		KnobTemplate< uns16 >* KNOB_FSCHED_LARGE_SIZE;
		KnobTemplate< uns16 >* KNOB_ISCHED_RATE;
		KnobTemplate< uns16 >* KNOB_MSCHED_RATE;
		KnobTemplate< uns16 >* KNOB_FSCHED_RATE;
		KnobTemplate< uns16 >* KNOB_ISCHED_MEDIUM_RATE;
		KnobTemplate< uns16 >* KNOB_MSCHED_MEDIUM_RATE;
		KnobTemplate< uns16 >* KNOB_FSCHED_MEDIUM_RATE;
		KnobTemplate< uns16 >* KNOB_ISCHED_LARGE_RATE;
		KnobTemplate< uns16 >* KNOB_MSCHED_LARGE_RATE;
		KnobTemplate< uns16 >* KNOB_FSCHED_LARGE_RATE;
		KnobTemplate< uns16 >* KNOB_SCHED_TO_WIDTH;
		KnobTemplate< uns16 >* KNOB_SCHED_TO_MEDIUM_WIDTH;
		KnobTemplate< uns16 >* KNOB_SCHED_TO_LARGE_WIDTH;
		KnobTemplate< string >* KNOB_TRACE_NAME_FILE;
		KnobTemplate< int >* KNOB_NUM_WARP_SCHEDULER;
		

	// =========== ../def/utils.param.def ===========
		KnobTemplate< bool >* KNOB_BUG_DETECTOR_ENABLE;
		KnobTemplate< int >* KNOB_COLLECT_CPI_INFO;
		KnobTemplate< int >* KNOB_COLLECT_CPI_INFO_FOR_MULTI_GPU;

};
#endif //__ALL_KNOBS_H_INCLUDED__
