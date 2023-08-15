/*
 * Copyright 2017-2021 vh
 * Copyright 2021-2023 kano
 * Copyright 2023 Hestiia
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "math.h"
#include "miner.h"
#include "usbutils.h"
#include "uart_utils.h"
#include "klist.h"

#include "driver_bm1397_utils.h"

#define JOB_MAX      0x7F
#define BUFFER_MAX   0xFF
#define SAMPLE_SIZE  0x78
#define MS_SECOND_1  1000

#define MS_SECOND_5  (MS_SECOND_1 * 5)
#define MS_SECOND_10 (MS_SECOND_1 * 10)
#define MS_SECOND_15 (MS_SECOND_1 * 15)
#define MS_SECOND_30 (MS_SECOND_1 * 30)
#define MS_MINUTE_1  (MS_SECOND_1 * 60)

#define MS_MINUTE_2  (MS_MINUTE_1 * 2)
#define MS_MINUTE_3  (MS_MINUTE_1 * 3)
#define MS_MINUTE_4  (MS_MINUTE_1 * 4)
#define MS_MINUTE_5  (MS_MINUTE_1 * 5)
#define MS_MINUTE_10 (MS_MINUTE_1 * 10)
#define MS_MINUTE_30 (MS_MINUTE_1 * 30)
#define MS_HOUR_1    (MS_MINUTE_1 * 60)

enum miner_state {
	MINER_INIT = 1,
	MINER_CHIP_COUNT,	// 2
	MINER_CHIP_COUNT_XX,	// 3
	MINER_CHIP_COUNT_OK,	// 4
	MINER_OPEN_CORE,	// 5
	MINER_OPEN_CORE_OK,	// 6
	MINER_MINING,		// 7
	MINER_MINING_DUPS,	// 8
	MINER_SHUTDOWN,		// 9
	MINER_SHUTDOWN_OK,	// 10
	MINER_RESET		// 11
};

enum miner_asic {
	BM1397
};

enum plateau_type {
	PT_NONONCE = 1,
	PT_FREQSET,
	PT_FREQNR,
	PT_DUPNONCE
};

enum micro_command {
	M1_GET_FAN    = (0x00 << 3),
	M1_GET_RPM    = (0x01 << 3),
	M1_GET_VIN    = (0x02 << 3),
	M1_GET_IIN    = (0x03 << 3),
	M1_GET_TEMP   = (0x04 << 3),
	M1_GET_VNODE0 = (0x05 << 3),

	M1_CLR_BEN    = (0x08 << 3),
	M1_SET_BEN    = (0x09 << 3),
	M1_CLR_LED    = (0x0A << 3),
	M1_SET_LED    = (0x0B << 3),
	M1_CLR_RST    = (0x0C << 3),
	M1_SET_RST    = (0x0D << 3),

	M2_SET_FAN    = (0x18 << 3),
	M2_SET_VCORE  = (0x1C << 3)
};

enum asic_state {
	ASIC_HEALTHY = 0,
	ASIC_HALFDEAD,
	ASIC_ALMOST_DEAD,
	ASIC_DEAD
};

// N.B. at 2TH/s R909 ticket 64 = ~1.2 nonces per chip per second
//  thus 20mins is only ~1000 nonces - so variance isn't very low
// time range of each value = 10 minutes
#define CHTIME 600
// number of ranges thus total 1hr
#define CHNUM 6

#define CHOFF(n) (((n) + CHNUM) % CHNUM)

// N.B. uses CLOCK_MONOTONIC
#define CHBASE(_t1) ((int)((_t1) / CHTIME))

#define CHCMP(_t1, _t2) (CHBASE(_t1) == CHBASE(_t2))

struct S_BM1397_CHIP
{
	time_t time_t_zero_sec; // seconds time of [offset]
	int32_t i32_offset; // the position of [0]
	int32_t t_i32_nb_nonces_ranges[CHNUM]; // number of nonces in each range
	int32_t i32_nb_nonces; // number of nonces in 0..last-1
	int32_t i32_last_used_offset; // last used offset 0 based
};
struct S_ASIC_INFO {
	struct timeval s_tv_last_nonce;              	// Last time nonce was found
	float f_frequency;                        		// Current frequency
	float f_frequency_set;                    		// set_frequency
	bool b_frequency_updated;                 		// Initiate check for new frequency
	uint32_t u32_frequency_attempt;             	// attempts of set_frequency
	uint32_t u32_duplicate_nonce_countr;            // Duplicate nonce counter
	uint32_t u32_total_duplicate_nonce_counter;		// Total duplicate nonce counter
	enum asic_state e_asic_state;					// Current i32_asic state
	enum asic_state e_asic_state_last;				// Previous i32_asic state
	struct timeval s_tv_startup_time;       		// Device startup time
	struct timeval s_tv_last_frequency_adjust;   	// Last time of frequency adjust
	struct timeval s_tv_last_frequency_ping;     	// Last time of frequency ping
	struct timeval s_tv_last_frequency_reply;    	// Last time of frequency reply
	uint32_t u32_last_found_nonce;         		// Last nonce found
	float f_fullscan_duration_ms;           	// Estimated time(ms) for full nonce range
	uint32_t u32_fullscan_duration;        		// Estimated time(us) for full nonce range
	uint64_t u64_hashrate;          		 	// Estimated hashrate = cores x chips x frequency
	float f_frequency_reply;					// TODO - to document
	
	int32_t i32_nonces;		// TODO - to document
	struct S_BM1397_CHIP s_bm1397_chip;	// running nonce buffer
};

struct COMPAC_NONCE
{
	int32_t i32_asic;
	unsigned char tu8_rx_buffer[BUFFER_MAX];
	size_t sizet_len;
	size_t sizet_previous_len;
	struct timeval s_tv_when;
};

#define DATA_NONCE(_item) ((struct COMPAC_NONCE *)(_item->data))
#define ALLOC_NLIST_ITEMS 256
#define LIMIT_NLIST_ITEMS 0

// BM1397 info->job_id offsets to check (when job_id is wrong)
static int cur_attempt[] = { 0, -4, -8, -12 };
#define CUR_ATTEMPT (sizeof(cur_attempt)/sizeof(int))

// macro to adjust frequency choices to be an integer multple of info->freq_base
#define FREQ_BASE(_f) (ceil((float)(_f) / info->freq_base) * info->freq_base)

// macro to add/subtract from the job_id but roll in the min...max range
#define JOB_ID_ROLL(_jid, _add, _info) \
	((_info)->min_job_id + (((_jid) + (_add) - (_info)->min_job_id) % \
		((_info)->max_job_id + 1 - (_info)->min_job_id)))

// convert chip to address for the 1397
#define CHIPPY1397(_inf, _chi) (((double)(_inf->chips) == 0) ? \
	(unsigned char)0 : \
	((unsigned char)(floor((double)0x100 / (double)(_inf->chips))) * (unsigned char)(_chi)))

// convert address to chip for the 1397
#define TOCHIPPY1397(_inf, _adr) (((double)(_inf->chips) == 0) ? \
	0 : (int)floor((double)(_adr) \
		/ floor((double)0x100 / (double)(_inf->chips))) )

// BM1397 registers
#define BM1397FREQ 0x08
#define BM1397TICKET 0x14

#define GHNUM (60*5)
#define GHOFF(n) (((n) + GHNUM) % GHNUM)
// a time jump without any nonces will reset the GEKKOHASH data
//  this would normally be a miner failure, so should reset anyway,
//  however under normal mining operation, using 10sec,
//   a 6GH/s i32_asic will have this happen, on average, about once every 10 days
//   a 30GH/s i32_asic is unlikely to have this happen in the life of the universe
#define GHLIMsec 10

// number of nonces that should give better than 80% accuracy
// CDF[ERlang] 400 0.8 = 9.0991e-06
#define GHNONCES 400

// a loss of this much hash rate will reduce requested freq and reset
#define GHREQUIRE 0.65

// number of nonces needed before using as the rolling hash rate
// N.B. 200Mhz ticket 16 GSF is around 2/sec
// also, 8 has high variance ... but resets shouldn't be common
// code adds 1 to this value since the first nonce isn't part of the H/s calc
#define GHNONCENEEDED 8

// running 5min nonce diff buffer (for GH/s)
// offset = current second, GHOFF(offset-1) = previous second
// GHOFF(offset-(GHNUM-1)) = GHOFF(offset+1) = oldest possible
// GHOFF(offset-last) = oldest used
// code is all linear except one loop that is almost always only
//  one interation or total max one interation per second elapsed real time
struct GEKKOHASH
{
	// seconds time of [offset]
	time_t zerosec;
	// the position of [0]
	int offset;
	// total diff in each second
	int64_t diff[GHNUM];
	// time of the first nonce in each second
	struct timeval firstt[GHNUM];
	// diff of first nonce in each second
	int64_t firstd[GHNUM];
	// time of the last nonce in each second
	struct timeval lastt[GHNUM];
	// number of nonces in each second
	int t_i32_nb_nonces_ranges[GHNUM];
	// sum of diff[0..last-1]
	int64_t diffsum;
	// number of nonces in 0..last-1
	int i32_nb_nonces;
	// last used offset 0 based
	int last;
};

#define JOBMIN 5
#define JOBOFF(n) (((n) + JOBMIN) % JOBMIN)
// a time jump without any work will reset the GEKKOJOB data
//  this would normally be all pool failure or power down due to heat
//  3 = 3 minutes so should never happen
#define JOBLIMn 3

// the arrays are minutes of data
// N.B. uses CLOCK_MONOTONIC
#define JOBTIME(_sec) ((int)((int)(_sec)/(int)60))

struct GEKKOJOB
{
	// JOBTIME of [offset]
	time_t zeromin;
	// time of last job added
	struct timeval lastjob;
	// the position of [0]
	int offset;
	// time of the first job in each
	struct timeval firstj[JOBMIN];
	// time of the last job in each
	struct timeval lastj[JOBMIN];
	// number of job items in each
	int jobnum[JOBMIN];
	// average ms between jobs
	double avgms[JOBMIN];
	// min ms
	double minms[JOBMIN];
	// max ms
	double maxms[JOBMIN];
	// number of jobs in 0..last-1
	int jobsnum;
	// last used offset 0 based
	int last;
};

struct COMPAC_INFO {

	enum sub_ident ident;		// Miner identity
	enum miner_state mining_state;	// Miner state
	enum miner_asic asic_type;	// ASIC Type
	struct thr_info *p_running_thrd;		// Running Thread
	struct thr_info listening_thrd;		// Listening Thread
	struct thr_info work_thrd;		// Miner Work Thread

	pthread_mutex_t lock;		// Mutex
	pthread_mutex_t wlock;		// Mutex Serialize Writes
	pthread_mutex_t rlock;		// Mutex Serialize Reads

	struct thr_info nthr;		// GSF Nonce Thread
	K_LIST *nlist;			// GSF Nonce list
	K_LIST *nstore;			// GSF Nonce store
	pthread_mutex_t nlock;		// GSF lock
	pthread_cond_t ncond;		// GSF wait
	uint64_t ntimeout;		// GSF number of cond timeouts
	uint64_t ntrigger;		// GSF number of cond tiggered

	float freq_mult;	     // frequency multiplier
	float freq_base;	     // frequency mod value
	float step_freq;	     // frequency step value
	float min_freq;              // Lowest frequency mine2 will tune down to
	int ramp_time;               // time to allow for initial frequency ramp
	float frequency;             // Chip Average Frequency
	float frequency_asic;        // Highest of current asics.
	float frequency_default;     // ASIC Frequency on RESET
	float frequency_requested;   // Requested Frequency
	float frequency_selected;    // Initial Requested Frequency
	float frequency_start;       // Starting Frequency
	float frequency_fail_high;   // Highest Frequency of Chip Failure
	float frequency_fail_low;    // Lowest Frequency of Chip Failure
	float frequency_computed;    // Highest hashrate seen as a frequency value
	float eff_gs;                // hash : expected hash
	float eff_tm;                // hash : expected hash
	float eff_li;                // hash : expected hash
	float eff_1m;                // hash : expected hash
	float eff_5m;                // hash : expected hash
	float eff_15;                // hash : expected hash
	float eff_wu;                // wu : expected wu
	float tune_up;               // Increase frequency when eff_gs is above value
	float tune_down;             // Decrease frequency when eff_gs is below value
	float freq_fail;	     	 // last freq set failure on BM1397
	float hr_scale;		     	 // scale adjustment for hashrate

	float micro_temp;            // Micro Reported Temp
	float wait_factor0;          // Base setting from opt value
	float wait_factor;           // Used to compute max_task_wait
	bool lock_freq;		     	 // When true disable all but safety,reset,shutdown and API freq changes
	int usb_prop;		     	 // Number of usec to wait after certain usb commands

	float fullscan_ms;           // Estimated time(ms) for full nonce range
	float task_ms;               // Avg time(ms) between task sent to device
	uint32_t fullscan_us;        // Estimated time(us) for full nonce range
	uint64_t hashrate;           // Estimated hashrate = cores x chips x frequency x hr_scale
	uint64_t busy_work;

	uint64_t task_hcn;           // Hash Count Number - max nonce iter.
	uint32_t prev_nonce;         // Last nonce found

	int failing;                 // Flag failing sticks
	int fail_count;              // Track failures = resets
	int frequency_fo;            // Frequency check token
	int frequency_of;            // Frequency check token
	int accepted;                // Nonces accepted
	int dups;                    // Duplicates found (for plateau code)
	int dupsall;                 // Duplicate nonce counter (total)
	int dupsreset;		     	 // Duplicates since reset
	int tracker;                 // Track code execution path
	int interface;               // USB interface
	int init_count;              // USB interface initialization counter
	int low_eff_resets;          // Count of low_eff resets
	int midstates;               // Number of midstates
	int nonceless;               // Tasks sent.  Resets when nonce is found.
	int nonces;                  // Nonces found
	int plateau_reset;           // Count plateau based resets
	int zero_check;              // Received nonces from zero work
	int vcore;                   // Core voltage
	int micro_found;             // Found a micro to communicate with

	bool can_boost;		     	 // true if boost is possible
	bool vmask;                  // Current pool's vmask
	bool boosted;                // Good nonce found for midstate2/3/4
	bool report;
	bool frequency_syncd;        // All asics share same frequency

	double wu;
	double wu_max;               // Max WU since last frequency change
	double rolling;

	uint32_t bauddiv;            // Baudrate divider
	uint32_t chips;              // Stores number of chips found
	uint32_t cores;              // Stores number of core per chp
	uint32_t difficulty;         // For computing hashrate
	float nonce_expect;	     	 // For PT_NONONCE
	float nonce_limit;	     	 // For PT_NONONCE
	uint32_t expected_chips;     // Number of chips for device
	uint64_t hashes;             // Hashes completed
	uint64_t xhashes;            // Hashes completed / 0xffffffffull
	uint32_t job_id;             // JobId incrementer
	int32_t log_wide;            // Extra output in widescreen mode
	uint32_t low_hash;           // Tracks of low hashrate
	uint32_t min_job_id;         // JobId start/rollover
	uint32_t add_job_id;         // JobId increment
	uint32_t max_job_id;         // JobId cap
	uint64_t max_task_wait;      // Micro seconds to wait before next task is sent
	uint32_t ramping;            // Ramping incrementer
	uint32_t rx_len;             // tu8_rx_buffer length
	uint32_t task_len;           // task length
	uint32_t ticket_mask;        // Used to reduce flashes per second
	uint32_t update_work;        // Notification of work update

	struct timeval start_time;              // Device startup time
	struct timeval monitor_time;            // Health check reference point
	struct timeval last_computed_increase;  // Last frequency computed change
	struct timeval last_scanhash;           // Last time inside scanhash loop
	struct timeval last_dup_time;           // Last time nonce dup detected was attempted
	struct timeval last_reset;              // Last time reset was triggered
	struct timeval last_task;               // Last time work was sent
	struct timeval s_tv_last_nonce;              // Last time nonce was found
	struct timeval last_hwerror;            // Last time hw error was detected
	struct timeval last_fast_forward;       // Last time of ramp jump to peak
	struct timeval last_frequency_adjust;   // Last time of frequency adjust
	struct timeval last_frequency_ping;     // Last time of frequency poll
	struct timeval last_frequency_report;   // Last change of frequency report
	struct timeval last_frequency_invalid;  // Last change of frequency report anomaly
	struct timeval last_chain_inactive;     // Last sent chain inactive
	struct timeval last_low_eff_reset;      // Last time responded to low_eff condition
	struct timeval last_micro_ping;         // Last time of micro controller poll
	struct timeval last_write_error;        // Last usb write error message
	struct timeval last_wu_increase;        // Last wu_max change
	struct timeval last_pool_lost;          // Last time we lost pool
	struct timeval last_update_rates;       // Last time we called compac_update_rates()

	struct timeval first_task;
	uint64_t tasks;
	uint64_t cur_off[CUR_ATTEMPT];
	double work_usec_avg;
	uint64_t work_usec_num;

	double last_work_diff;			// Diff of last work sent
	struct timeval last_ticket_attempt;	// List attempt to set ticket
	int ticket_number;				// offset in ticket array
	int ticket_work;				// work sent since ticket set
	int64_t ticket_nonces;			// nonces since ticket set
	int64_t below_nonces;			// nonces too low since ticket set
	bool ticket_ok;					// ticket working ok
	bool ticket_got_low;			// nonce found close to but >= diff
	int ticket_failures;			// Must not exceed MAX_TICKET_CHECK
	struct S_ASIC_INFO asics[255];
	int64_t noncebyte[256];			// Count of nonces with the given byte[3] value
	bool nb2c_setup;				// BM1397 true = nb2chip is setup (false = all 0)
	uint16_t nb2chip[256];			// BM1397 map nonce byte to: chip that produced it
	bool active_work[JOB_MAX+1];            // Tag good and stale work
	struct work *work[JOB_MAX+1];           // Work ring buffer

	pthread_mutex_t ghlock;			// Mutex for all access to gh
	struct GEKKOHASH gh;			// running hash rate buffer
	float ghrequire;				// Ratio of expected HR required (GHREQUIRE) 0.0-0.8
	pthread_mutex_t joblock;		// Mutex for all access to jb
	struct GEKKOJOB job;			// running job rate buffer

	pthread_mutex_t slock;			// usleep() stats
	uint64_t num0;
	uint64_t num;
	double req;
	double fac;
	uint64_t num1_1;
	double req1_1;
	double fac1_1;
	uint64_t num1_5;
	double req1_5;
	double fac1_5;
	uint64_t inv;

	double workgen;					// work timing overrun stats
	int64_t over1num;
	double over1amt;
	int64_t over2num;
	double over2amt;

	struct timeval tune_limit;			// time between tune checks
	struct timeval last_tune_up;		// time of last tune up attempt

	unsigned char task[BUFFER_MAX];         // Task transmit buffer
	unsigned char cmd[BUFFER_MAX];          // Command transmit buffer
	unsigned char tu8_rx_buffer[BUFFER_MAX];           // Receive buffer
	unsigned char tu8_tx_buffer[BUFFER_MAX];           // Transmit buffer
	unsigned char end[1024];                // buffer overrun test
};


