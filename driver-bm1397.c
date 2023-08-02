/*
 * Copyright 2017-2021 vh
 * Copyright 2021-2022 sidehack
 * Copyright 2021-2023 kano
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "driver-bm1397.h"
#include "crc.h"
#include "compat.h"
#include <unistd.h>
#include <gpiod.h>


#ifdef __GNUC__
#if __GNUC__ >= 7
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif
#endif

// usleep reliability
#if defined(__APPLE__)
#define USLEEPMIN 2000
#define USLEEPPLUS 200
#elif defined (WIN32)
#define USLEEPMIN 250
#define USLEEPPLUS 100
#else
#define USLEEPMIN 200
#define USLEEPPLUS 50
#endif

static bool compac_prepare(struct thr_info *thr);
static pthread_mutex_t static_lock = PTHREAD_MUTEX_INITIALIZER;
static bool last_widescreen;
static uint8_t dev_init_count[0xffff] = {0};
static uint8_t *init_count;
static uint32_t stat_len;
static uint32_t chip_max;

#define MS2US(_n) ((_n) * 1000)

// report averages and how far they overrun the requested time
// linux would appear to be unable to handle less than 55us
// on the RPi4 it would regularly sleep 3 times as long
// thus code in general ignores sleeping anything less than 200us
#define TUNE_CODE 1

static void gekko_usleep(struct COMPAC_INFO *info, int usec)
{
#if TUNE_CODE
	struct timeval stt, fin;
	double td, fac;
#endif

	// error for usleep()
	if (usec >= 1000000)
	{
		cgsleep_ms(usec / 1000);
#if TUNE_CODE
		mutex_lock(&info->slock);
		info->inv++;
		mutex_unlock(&info->slock);
#endif
		return;
	}

#if TUNE_CODE
	cgtime(&stt);
#endif
	usleep(usec);
#if TUNE_CODE
	cgtime(&fin);

	td = us_tdiff(&fin, &stt);
	fac = (td / (double)usec);

	mutex_lock(&info->slock);
	if (td < usec)
		info->num0++;
	if (fac >= 1.5)
	{
		info->req1_5 += usec;
		info->fac1_5 += fac;
		info->num1_5++;
	}
	else
	{
		if (fac >= 1.1)
		{
			info->req1_1 += usec;
			info->fac1_1 += fac;
			info->num1_1++;
		}
		else
		{
			info->req += usec;
			info->fac += fac;
			info->num++;
		}
	}
	mutex_unlock(&info->slock);
#endif
}

static float freqbounding(float value, float lower_bound, float upper_bound)
{
	if (value < lower_bound)
		return lower_bound;
	if (value > upper_bound)
		return upper_bound;
	return value;
}

uint32_t bmcrc(unsigned char *ptr, uint32_t len)
{
	unsigned char c[5] = {1, 1, 1, 1, 1};
	uint32_t i, c1, ptr_idx = 0;

	for (i = 0; i < len; i++) {
		c1 = c[1];
		c[1] = c[0];
		c[0] = c[4] ^ ((ptr[ptr_idx] & (0x80 >> (i % 8))) ? 1 : 0);
		c[4] = c[3];
		c[3] = c[2];
		c[2] = c1 ^ c[0];

		if (((i + 1) % 8) == 0)
			ptr_idx++;
	}
	return (c[4] * 0x10) | (c[3] * 0x08) | (c[2] * 0x04) | (c[1] * 0x02) | (c[0] * 0x01);
}

void dumpbuffer(struct cgpu_info *compac, int LOG_LEVEL, char *note, unsigned char *ptr, uint32_t len)
{
	if (opt_log_output || LOG_LEVEL <= opt_log_level) {
		char str[2048];
		const char * hex = "0123456789ABCDEF";
		char * pout = str;
		unsigned int i = 0;

		for(; i < 768 && i < len - 1; ++i) {
			*pout++ = hex[(*ptr>>4)&0xF];
			*pout++ = hex[(*ptr++)&0xF];
			if (i % 42 == 41) {
				*pout = 0;
				pout = str;
				applog(LOG_LEVEL, "%i: %s %s: %s", compac->cgminer_id, compac->drv->name, note, str);
			} else {
				*pout++ = ':';
			}
		}
		*pout++ = hex[(*ptr>>4)&0xF];
		*pout++ = hex[(*ptr)&0xF];
		*pout = 0;
		applog(LOG_LEVEL, "%d: %s %d - %s: %s", compac->cgminer_id, compac->drv->name, compac->device_id, note, str);
	}
}

static int compac_micro_send(struct cgpu_info *compac, uint8_t cmd, uint8_t channel, uint8_t value)
{
	struct COMPAC_INFO *info = compac->device_data;
	int bytes = 1;
	int read_bytes = 1;
	int micro_temp;
	uint8_t temp;
	unsigned short usb_val;
	__maybe_unused char null[255];

	// synchronous : safe to run in the listen thread.
	if (!info->micro_found) {
		return 0;
	}

	// Baud Rate : 500,000

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF3; // low byte: bitmask - 1111 0011 - CB1(HI), CB0(HI)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	gekko_usleep(info, MS2US(2));
	//usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, 0x06, (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);

	info->cmd[0] = cmd | channel;
	info->cmd[1] = value;

	if (value != 0x00 || cmd == M2_SET_VCORE) {
		bytes = 2;
	}

	usb_read_timeout(compac, (char *)info->rx, 255, &read_bytes, 1, C_GETRESULTS);

	dumpbuffer(compac, LOG_INFO, "(micro) TX", info->cmd, bytes);
	usb_write(compac, (char *)info->cmd, bytes, &read_bytes, C_REQUESTRESULTS);

	memset(info->rx, 0, info->rx_len);
	usb_read_timeout(compac, (char *)info->rx, 1, &read_bytes, 5, C_GETRESULTS);

	if (read_bytes > 0) {
		dumpbuffer(compac, LOG_INFO, "(micro) RX", info->rx, read_bytes);
		switch (cmd) {
			case 0x20:
				temp = info->rx[0];
				micro_temp = 32 + 1.8 * temp;
				if (micro_temp != info->micro_temp) {
					info->micro_temp = micro_temp;
					applog(LOG_WARNING, "%d: %s %d - micro temp changed to %d°C / %.1f°F",
						compac->cgminer_id, compac->drv->name, compac->device_id, temp, info->micro_temp);
				}
				break;
			default:
				break;
		}
	}

	// Restore Baud Rate
	//usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, (info->bauddiv + 1), (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF2; // low byte: bitmask - 1111 0010 - CB1(HI), CB0(LO)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	gekko_usleep(info, MS2US(2));

	return read_bytes;
}

#define compac_send(_c, _r, _b, _crc) compac_send2(_c, _r, _b, _crc, NULL)
static void compac_send2(struct cgpu_info *compac, unsigned char *req_tx, uint32_t bytes, uint32_t crc_bits, __maybe_unused char *msg)
{
	struct COMPAC_INFO *info = compac->device_data;
	int read_bytes = 1;
	unsigned int i, off = 0;

	// leave original buffer intact

	if (info->asic_type == BM1397)
	{
		info->cmd[0] = 0x55;
		info->cmd[1] = 0xAA;
		off = 2;
	}

	for (i = 0; i < bytes; i++)
		info->cmd[i+off] = req_tx[i];

	bytes += off;

	info->cmd[bytes-1] |= bmcrc(req_tx, crc_bits);

#if 0
	if (msg == NULL)
		msg = "null";

applog(LOG_ERR, "%s() %d: %s %d - Send len %3u (%s)", __func__,
	compac->cgminer_id, compac->drv->name, compac->device_id, bytes, msg);
applog(LOG_ERR, "%s()  [%02x %02x %02x %02x %02x %02x %02x %02x]", __func__,
	info->cmd[0], info->cmd[1], info->cmd[2], info->cmd[3], info->cmd[4], info->cmd[5], info->cmd[6], info->cmd[7]);
applog(LOG_ERR, "%s()  [%02x %02x %02x %02x %02x %02x %02x %02x]", __func__,
	info->cmd[8], info->cmd[9], info->cmd[10], info->cmd[11], info->cmd[12], info->cmd[13], info->cmd[14], info->cmd[15]);
#endif

	int log_level = (bytes < info->task_len) ? LOG_INFO : LOG_INFO;

	dumpbuffer(compac, log_level, "TX", info->cmd, bytes);
	usb_write(compac, (char *)(info->cmd), bytes, &read_bytes, C_REQUESTRESULTS);

	//let the usb frame propagate
	if (info->asic_type == BM1397)
		gekko_usleep(info, info->usb_prop);
	else
		gekko_usleep(info, MS2US(1));
}

static float limit_freq(struct COMPAC_INFO *info, float freq, bool zero)
{
	switch(info->ident)
	{
	 case IDENT_BSC:
	 case IDENT_GSC:
	 case IDENT_BSD:
	 case IDENT_GSD:
	 case IDENT_BSE:
	 case IDENT_GSE:
		freq = freqbounding(freq, info->freq_base, 500);
		break;
	 case IDENT_GSH:
	 case IDENT_GSI:
		freq = freqbounding(freq, 50, 900);
		break;
	 case IDENT_GSF:
	 case IDENT_GSFM:
		// allow 0 also if zero is true - coded obviously
		if (zero && freq == 0)
			freq = 0;
		else
			freq = freqbounding(freq, 100, 800);
		break;
	 default:
		// 'should' never happen ...
		freq = freqbounding(freq, 100, 300);
		break;
	}
	return freq;
}

static void ping_freq(struct cgpu_info *compac, int asic)
{
	struct COMPAC_INFO *info = compac->device_data;
	bool ping = false;

	if (info->asic_type == BM1397)
	{
		unsigned char pingall[] = {0x52, 0x05, 0x00, BM1397FREQ, 0x00};
		compac_send2(compac, pingall, sizeof(pingall), 8 * sizeof(pingall) - 8, "pingfreq");
		ping = true;
	}
	else if (info->asic_type == BM1387)
	{
		unsigned char buffer[] = {0x44, 0x05, 0x00, 0x0C, 0x00};  // PLL_PARAMETER
		buffer[2] = (0x100 / info->chips) * asic;
		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
		ping = true;
	}
	else if (info->asic_type == BM1384)
	{
		unsigned char buffer[] = {0x04, 0x00, 0x04, 0x00};
		buffer[1] = (0x100 / info->chips) * asic;
		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		ping = true;
	}

	if (ping)
	{
		cgtime(&info->last_frequency_ping);
		cgtime(&(info->asics[asic].last_frequency_ping));
	}
}

static void gsf_calc_nb2c(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	int c, i, j;
	double fac;

	if (info->chips == 1)
	{
		// default all 0 is correct
		info->nb2c_setup = true;
	}
	else if (info->chips == 6)
	{
		// groups of 4
		fac = CHIPPY1397(info, 1) / 4.0;
		for (i = 0; i < 256; i += 64)
		{
			for (j = 0; j < 64; j++)
			{
				c = (int)((double)j / fac);
				if (c >= (int)(info->chips))
					c = info->chips - 1;
				info->nb2chip[i + j] = c;
			}
		}
		info->nb2c_setup = true;
	}
}

static void gc_wipe(struct GEKKOCHIP *gc, struct timeval *now)
{
	// clear out everything
	gc->zerosec = now->tv_sec;
	gc->offset = 0;
	memset(gc->noncenum, 0, sizeof(gc->noncenum));
	gc->noncesum = 0;
	gc->last = 0;
}

static void gc_wipe_all(struct COMPAC_INFO *info, struct timeval *now, bool locked)
{
	int i;

	if (!locked)
		mutex_lock(&info->ghlock);

	for (i = 0; i < (int)(info->chips); i++)
		gc_wipe(&(info->asics[i].gc), now);

	if (!locked)
		mutex_unlock(&info->ghlock);
}

// update asic->gc offset as at 'now' and correct values
// info must be locked, wipe creates a new data set
static void gc_offset(struct COMPAC_INFO *info, struct ASIC_INFO *asic, struct timeval *now, bool wipe, bool locked)
{
	struct GEKKOCHIP *gc = &(asic->gc);
	time_t delta;

	if (!locked)
		mutex_lock(&info->ghlock);

	// wipe or delta != 0
	if (wipe || !CHCMP(gc->zerosec, now->tv_sec))
	{
		// clear some/all delta data

		delta = CHBASE(now->tv_sec) - CHBASE(gc->zerosec);
		// if time goes back, also reset everything
		//  a forward jump of CHNUM will reset the whole buffer
		if (wipe || delta < 0 || delta >= CHNUM)
			gc_wipe(gc, now);
		else
		{
			// delta is > 0, but usually always 1 unless,
			//  due to asic failure, a 10 minutes had no nonces
			// however the loop will total 1 iteration each
			//  10 minutes elapsed real time e.g. if not called
			//  for 30 minutes, it will loop 3 times
			// there is also a CHNUM-1 limit on that

			gc->zerosec = now->tv_sec;
			// clear out the old values
			do
			{
				gc->offset = CHOFF(gc->offset+1);

				gc->noncesum -= gc->noncenum[CHOFF(gc->offset)];
				gc->noncenum[CHOFF(gc->offset)] = 0;

				if (gc->last < (CHNUM-1))
					gc->last++;
			}
			while (--delta > 0);
		}
	}

	// if there's been no nonces up to now, history must already be all zero
	//  so just remove history
	if (gc->noncesum == 0 && gc->last > 0)
		gc->last = 0;

	if (!locked)
		mutex_unlock(&info->ghlock);
}

// update info->gh offset as at 'now' and correct values
// info must be locked, wipe creates a new data set and also wipes all asic->gc
static void gh_offset(struct COMPAC_INFO *info, struct timeval *now, bool wipe, bool locked)
{
	struct GEKKOHASH *gh = &(info->gh);
	time_t delta;
	int i;

	if (!locked)
		mutex_lock(&info->ghlock);

	// first time in, wipe is ignored (it's already all zero)
	if (gh->zerosec == 0)
	{
		gh->zerosec = now->tv_sec;
		for (i = 0; i < (int)(info->chips); i++)
			info->asics[i].gc.zerosec = now->tv_sec;
	}
	else
	{
		if (wipe)
			gc_wipe_all(info, now, true);

		// wipe or delta != 0
		if (wipe || gh->zerosec != now->tv_sec)
		{
			// clear some/all delta data

			delta = now->tv_sec - gh->zerosec;
			// if time goes back, also reset everything
			//  N.B. a forward time jump between 2 and GHLIMsec
			//  seconds will reduce the hash rate value
			//  but GHLIMsec or more will reset the whole buffer
			if (wipe || delta < 0 || delta >= GHLIMsec)
			{
				// clear out everything
				gh->zerosec = now->tv_sec;
				gh->offset = 0;
				memset(gh->diff, 0, sizeof(gh->diff));
				memset(gh->firstt, 0, sizeof(gh->firstt));
				memset(gh->firstd, 0, sizeof(gh->firstd));
				memset(gh->lastt, 0, sizeof(gh->lastt));
				memset(gh->noncenum, 0, sizeof(gh->noncenum));
				gh->diffsum = 0;
				gh->noncesum = 0;
				gh->last = 0;
			}
			else
			{
				// delta is > 0, but usually always 1 unless,
				//  due to asic failure, a second had no nonces
				// however the loop will total 1 iteration each
				//  second elapsed real time e.g. if not called
				//  for 3 seconds, it will loop 3 times
				// there is also a GHLIMsec-1 limit on that

				gh->zerosec = now->tv_sec;
				// clear out the old values
				do
				{
					gh->offset = GHOFF(gh->offset+1);

					gh->diffsum -= gh->diff[GHOFF(gh->offset)];
					gh->diff[GHOFF(gh->offset)] = 0;
					gh->noncesum -= gh->noncenum[GHOFF(gh->offset)];
					gh->noncenum[GHOFF(gh->offset)] = 0;

					gh->firstt[GHOFF(gh->offset)].tv_sec = 0;
					gh->firstt[GHOFF(gh->offset)].tv_usec = 0;
					gh->firstd[GHOFF(gh->offset)] = 0;
					gh->lastt[GHOFF(gh->offset)].tv_sec = 0;
					gh->lastt[GHOFF(gh->offset)].tv_usec = 0;

					if (gh->last < (GHNUM-1))
						gh->last++;
				}
				while (--delta > 0);
			}
		}
	}

	// if there's been no nonces up to now, history must already be all zero
	//  so just remove history
	if (gh->noncesum == 0 && gh->last > 0)
		gh->last = 0;
	// this also handles the issue of a nonce-less wipe with a high
	//  now->tv_usec and if the first nonce comes in during the next second.
	//  without setting 'last=0' the previous empty full second(s) will
	//   always be included in the elapsed time used to calc the hash rate

	if (!locked)
		mutex_unlock(&info->ghlock);
}

// update info->gh with a new nonce as at 'now' (diff=info->difficulty)
// info must be locked, wipe creates a new data set with the single nonce
static void add_gekko_nonce(struct COMPAC_INFO *info, struct ASIC_INFO *asic, struct timeval *now)
{
	struct GEKKOHASH *gh = &(info->gh);

	mutex_lock(&info->ghlock);

	gh_offset(info, now, false, true);

	if (gh->diff[gh->offset] == 0)
	{
		gh->firstt[gh->offset].tv_sec = now->tv_sec;
		gh->firstt[gh->offset].tv_usec = now->tv_usec;
		gh->firstd[gh->offset] = info->difficulty;
	}
	gh->lastt[gh->offset].tv_sec = now->tv_sec;
	gh->lastt[gh->offset].tv_usec = now->tv_usec;

	gh->diff[gh->offset] += info->difficulty;
	gh->diffsum += info->difficulty;
	(gh->noncenum[gh->offset])++;
	(gh->noncesum)++;

	if (asic != NULL)
	{
		struct GEKKOCHIP *gc = &(asic->gc);

		gc_offset(info, asic, now, false, true);
		(gc->noncenum[gc->offset])++;
		(gc->noncesum)++;
	}

	mutex_unlock(&info->ghlock);
}

// calculate MH/s hashrate, info must be locked
// value is 0.0 if there's no useful data
// caller check info->gh.last for history size used and info->gh.noncesum-1
//  for the amount of data used (i.e. accuracy of the hash rate)
static double gekko_gh_hashrate(struct COMPAC_INFO *info, struct timeval *now, bool locked)
{
	struct GEKKOHASH *gh = &(info->gh);
	struct timeval age, end;
	int zero, last;
	uint64_t delta;
	double ghr, old;

	ghr = 0.0;

	if (!locked)
		mutex_lock(&info->ghlock);

	// can't be calculated with only one nonce
	if (gh->diffsum > 0 && gh->noncesum > 1)
	{
		gh_offset(info, now, false, true);

		if (gh->diffsum > 0 && gh->noncesum > 1)
		{
			// offset of 'now'
			zero = gh->offset;
			// offset of oldest nonce
			last = GHOFF(zero - gh->last);

			if (gh->diff[last] != 0)
			{
				// from the oldest nonce, excluding it's diff
				delta = gh->firstd[last];
				age.tv_sec = gh->firstt[last].tv_sec;
				age.tv_usec = gh->firstt[last].tv_usec;
			}
			else
			{
				// if last is empty, use the start time of last
				delta = 0;
				age.tv_sec = gh->zerosec - (GHNUM - 1);
				age.tv_usec = 0;
			}

			// up to the time of the newest nonce as long as it
			//  was curr or prev second, otherwise use now
			if (gh->diff[zero] != 0)
			{
				// time of the newest nonce found this second
				end.tv_sec = gh->lastt[zero].tv_sec;
				end.tv_usec = gh->lastt[zero].tv_usec;
			}
			else
			{
				// unexpected ... no recent nonces ...
				if (gh->diff[GHOFF(zero-1)] == 0)
				{
					end.tv_sec = now->tv_sec;
					end.tv_usec = now->tv_usec;
				}
				else
				{
					// time of the newest nonce found this second-1
					end.tv_sec = gh->lastt[GHOFF(zero-1)].tv_sec;
					end.tv_usec = gh->lastt[GHOFF(zero-1)].tv_usec;
				}
			}

			old = tdiff(&end, &age);
			if (old > 0.0)
			{
				ghr = (double)(gh->diffsum - delta)
					* (pow(2.0, 32.0) / old) / 1.0e6;
			}
		}
	}

	if (!locked)
		mutex_unlock(&info->ghlock);

	return ghr;
}

static void job_offset(struct COMPAC_INFO *info, struct timeval *now, bool wipe, bool locked)
{
	struct GEKKOJOB *job = &(info->job);
	time_t delta;
	int jobnow;

	jobnow = JOBTIME(now->tv_sec);

	if (!locked)
		mutex_lock(&info->joblock);

	// first time in, wipe is ignored (it's already all zero)
	if (job->zeromin == 0)
		job->zeromin = jobnow;
	else
	{
		// wipe or delta != 0
		if (wipe || job->zeromin != jobnow)
		{
			// clear some/all delta data

			delta = jobnow - job->zeromin;
			// if time goes back, also reset everything
			//  N.B. a forward time jump between 2 and JOBLIMn
			//  seconds will reduce the job rate value
			//  but JOBLIMn or more will reset the whole buffer
			if (wipe || delta < 0 || delta >= JOBLIMn)
			{
				// clear out everything
				job->zeromin = jobnow;
				job->lastjob.tv_sec = 0;
				job->lastjob.tv_usec = 0;
				job->offset = 0;
				memset(job->firstj, 0, sizeof(job->firstj));
				memset(job->lastj, 0, sizeof(job->lastj));
				memset(job->jobnum, 0, sizeof(job->jobnum));
				memset(job->avgms, 0, sizeof(job->avgms));
				memset(job->minms, 0, sizeof(job->minms));
				memset(job->maxms, 0, sizeof(job->maxms));
				job->jobsnum = 0;
				job->last = 0;
			}
			else
			{
				// delta is > 0, but usually always 1 unless,
				//  due to asic or pool failure, a minute had no jobs
				// however the loop will total 1 iteration each
				//  minute elapsed real time e.g. if not called
				//  for 2 minutes, it will loop 2 times
				// there is also a JOBLIMn-1 limit on that

				job->zeromin = jobnow;
				// clear out the old values
				do
				{
					job->offset = JOBOFF(job->offset+1);

					job->firstj[JOBOFF(job->offset)].tv_sec = 0;
					job->firstj[JOBOFF(job->offset)].tv_usec = 0;
					job->lastj[JOBOFF(job->offset)].tv_sec = 0;
					job->lastj[JOBOFF(job->offset)].tv_usec = 0;

					job->jobsnum -= job->jobnum[JOBOFF(job->offset)];
					job->jobnum[JOBOFF(job->offset)] = 0;
					job->avgms[JOBOFF(job->offset)] = 0;
					job->minms[JOBOFF(job->offset)] = 0;
					job->maxms[JOBOFF(job->offset)] = 0;

					if (job->last < (JOBMIN-1))
						job->last++;
				}
				while (--delta > 0);
			}
		}
	}

	// if there's been no jobs up to now, history must already be all zero
	//  so just remove history
	if (job->jobsnum == 0 && job->last > 0)
		job->last = 0;
	// this also handles the issue of a job-less wipe with a high
	//  now->tv_usec and if the first job comes in during the next minute.
	//  without setting 'last=0' the previous empty full minute will
	//   always be included in the elapsed time used to calc the job rate

	if (!locked)
		mutex_unlock(&info->joblock);
}

// update info->job with a job as at 'now'
// info must be locked, wipe creates a new empty data set
static void add_gekko_job(struct COMPAC_INFO *info, struct timeval *now, bool wipe)
{
	struct GEKKOJOB *job = &(info->job);
	bool firstjob;
	double avg;
	double ms;

	mutex_lock(&info->joblock);

	job_offset(info, now, wipe, true);

	if (!wipe)
	{
		if (job->jobnum[job->offset] == 0)
		{
			job->firstj[job->offset].tv_sec = now->tv_sec;
			job->firstj[job->offset].tv_usec = now->tv_usec;
			firstjob = true;
		}
		else
			firstjob = false;

		job->lastj[job->offset].tv_sec = now->tv_sec;
		job->lastj[job->offset].tv_usec = now->tv_usec;

		// first job time in each offset gets ignored
		// this is only necessary for the very first job,
		//  but easier to do it for every offset group
		if (firstjob)
		{
			// already true
			// job->avgms[job->offset] = 0.0;
			// job->minms[job->offset] = 0.0;
			// job->maxms[job->offset] = 0.0;
		}
		else
		{
			avg = job->avgms[job->offset] * (double)(job->jobnum[job->offset] - 1);

			ms = (double)(now->tv_sec - job->lastjob.tv_sec) * 1000.0;
			ms += (double)(now->tv_usec - job->lastjob.tv_usec) / 1000.0;

			// jobnum[] must be > 0
			job->avgms[job->offset] = (avg + ms) / (double)(job->jobnum[job->offset]);

			if (job->minms[job->offset] == 0.0)
			{
				job->minms[job->offset] = ms;
				job->maxms[job->offset] = ms;
			}
			else
			{
				if (ms < job->minms[job->offset])
					job->minms[job->offset] = ms;
				if (job->maxms[job->offset] < ms)
					job->maxms[job->offset] = ms;
			}
		}

		(job->jobnum[job->offset])++;
		(job->jobsnum)++;

		job->lastjob.tv_sec = now->tv_sec;
		job->lastjob.tv_usec = now->tv_usec;
	}

	mutex_unlock(&info->joblock);
}

// ignore nonces for this many work items after the ticket change
#define TICKET_DELAY 8

// allow this many nonces below the ticket value in case of work swap delays
// N.B. if the chip mask is half the wanted value,
//	roughly 85% of shares will be low since CDF 190% = 0.850
// with the lowest nonce_count of 150 below for diff 2,
//  TICKET_BLOW_LIM 4 will always be exceeded if incorrectly set to diff 1
#define TICKET_BELOW_LIM 4

struct TICKET_INFO {
	uint32_t diff;		// work diff value
	uint32_t ticket_mask;	// ticket mask to ensure work diff
	int nonce_count;	// CDF[Erl] nonces must have 1 below low_limit
	double low_limit;	// must be a diff below this or ticket is too hi
	double hi_limit;	// a diff below this means ticket is too low
				//  set to .1 below diff to avoid any rounding
	uint32_t cclimit;	// chips x cores limit i.e. required to go above 16
};

// ticket restart checks allowed before forced to diff=1
#define MAX_TICKET_CHECK 3

// ticket values, diff descending. List values rather than calc them
// end comments are how long at given task/sec (15 ~= 60 1diff nonce/sec = ~260GH/s)
//  testing should take and chance of failure
//   though it will retry MAX_TICKET_CHECK times so shouldn't give up in the
//   exceedingly rare occasion where it fails once due to bad luck
// limit to max diff of 16 unless the chips x cores is a bit better than a GSF/GSFM
//  to ensure enough nonces are coming back to identify status changes/issues
// the luck calculation is the chance all nonce diff values will be above low_limit
//  after nonce_count nonces i.e. after nonce_count nonces there should be a nonce
//  below low_limit, or the ticket mask is actually higher than it was set to
//  the gsl function is cdf_gamma_Q(nonces, nonces, low_limit/diff)
static struct TICKET_INFO ticket_1397[] =
{
	{ 64,	0xfc,  20000,	65.9,	63.9, 2600 }, // 90 59.3m Erlang=1.6x10-5 <- 64+ nonces
	{ 32,	0xf8,  10000,	33.3,	31.9, 1300 }, // 45 29.6m Erlang=3.0x10-5 <- 32+ nonces
	{ 16,	0xf0,	5000,	16.9,	15.9,	 0 }, // 15 22.2m Erlang=4.6x10-5 <- 16+ nonces
	{  8,	0xe0,	1250,	 8.9,	 7.9,	 0 }, // 15  166s Erlang=6.0x10-5
	{  4,	0xc0,	 450,	 4.9,	 3.9,	 0 }, // 15   30s Erlang=3.9x10-6
	{  2,	0x80,	 150,	 2.9,	 1.9,	 0 }, // 15    5s Erlang=5.4x10-7
	{  1,	0x00,	  50,	 1.9,	 0.0,	 0 }, // 15  0.8s Erlang=1.5x10-7 <- all nonces
	{  0  }
};

// force=true to allow setting it if it may not have taken before
//  force also delays longer after sending the ticket mask
// diff=0.0 mean set the highest valid
static void set_ticket(struct cgpu_info *compac, float diff, bool force, bool locked)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct timeval now;
	bool got = false;
	uint32_t udiff, new_diff = 0, new_mask = 0, cc;
	int i;

	if (diff == 0.0)
	{
		// above max will get the highest valid for cc
		diff = 128;
	}

//	if (!force && info->last_work_diff == diff)
//		return;

	// closest uint diff equal or below
	udiff = (uint32_t)floor(diff);
	cc = info->chips * info->cores;

	for (i = 0; ticket_1397[i].diff > 0; i++)
	{
		if (udiff >= ticket_1397[i].diff && cc > ticket_1397[i].cclimit)
		{
			// if ticket is already the same
			if (!force && info->difficulty == ticket_1397[i].diff)
				return;

			if (!locked)
				mutex_lock(&info->lock);
			new_diff = info->difficulty = ticket_1397[i].diff;
			new_mask = info->ticket_mask = ticket_1397[i].ticket_mask;
			info->last_work_diff = diff;
			cgtime(&info->last_ticket_attempt);
			info->ticket_number = i;
			info->ticket_work = 0;
			info->ticket_nonces = 0;
			info->below_nonces = 0;
			info->ticket_ok = false;
			info->ticket_got_low = false;
			if (!locked)
				mutex_unlock(&info->lock);
			got = true;
			break;
		}
	}

	// code failure
	if (!got)
		return;

	// set them all the same 0x51 .... 0x00
	unsigned char ticket[] = {0x51, 0x09, 0x00, BM1397TICKET, 0x00, 0x00, 0x00, 0xC0, 0x00};
	ticket[7] = info->ticket_mask;

	compac_send2(compac, ticket, sizeof(ticket), 8 * sizeof(ticket) - 8, "ticket");
	if (!force)
		gekko_usleep(info, MS2US(10));
	else
		gekko_usleep(info, MS2US(20));

	applog(LOG_ERR, "%d: %s %d - set ticket to 0x%02x/%u work %u/%.1f",
		compac->cgminer_id, compac->drv->name, compac->device_id,
		new_mask, new_diff, udiff, diff);

	// wipe info->gh/asic->gc
	cgtime(&now);
	gh_offset(info, &now, true, false);
	job_offset(info, &now, true, false);
	// reset P:
	info->frequency_computed = 0;
}

// expected nonces for GEKKOCHIP - MUST already be locked AND gc_offset()
// full 50 mins + current offset in 10 mins - N.B. uses CLOCK_MONOTONIC
// it will grow from 0% to ~100% between 50 & 60 mins if the chip
//  is performing at 100% - random variance of course also applies
static double noncepercent(struct COMPAC_INFO *info, int chip, struct timeval *now)
{
	double sec, hashpersec, noncepersec, nonceexpect;

	if (info->asic_type != BM1397)
		return 0.0;

	sec = CHTIME * (CHNUM-1) + (now->tv_sec % CHTIME) + ((double)now->tv_usec / 1000000.0);

	hashpersec = info->asics[chip].frequency * info->cores * info->hr_scale * 1000000.0;

	noncepersec = (hashpersec / (double)0xffffffffull)
			/ (double)(ticket_1397[info->ticket_number].diff);

	nonceexpect = noncepersec * sec;

	return 100.0 * (double)(info->asics[chip].gc.noncesum) / nonceexpect;
}

// GSF/GSFM any chip count
static void calc_gsf_freq(struct cgpu_info *compac, float frequency, int chip)
{
	struct COMPAC_INFO *info = compac->device_data;
	char chipn[8];
	bool doall;

	if (info->asic_type != BM1397)
		return;

	if (chip == -1)
		doall = true;
	else
	{
		if (chip < 0 || chip >= (int)(info->chips))
		{
			applog(LOG_ERR, "%d: %s %d - invalid set chip [%d] -> freq %.2fMHz",
				compac->cgminer_id, compac->drv->name, compac->device_id, chip, frequency);
			return;
		}
		doall = false;
	}

	// if attempting the same frequency that previously failed ...
	if (frequency != 0 && frequency == info->freq_fail)
		return;

	unsigned char prefreqall[] = {0x51, 0x09, 0x00, 0x70, 0x0F, 0x0F, 0x0F, 0x00, 0x00};
	unsigned char prefreqch[] = {0x41, 0x09, 0x00, 0x70, 0x0F, 0x0F, 0x0F, 0x00, 0x00};

	// default 200Mhz if it fails
	unsigned char freqbufall[] = {0x51, 0x09, 0x00, BM1397FREQ, 0x40, 0xF0, 0x02, 0x35, 0x00};
	unsigned char freqbufch[] = {0x41, 0x09, 0x00, BM1397FREQ, 0x40, 0xF0, 0x02, 0x35, 0x00};

	float deffreq = 200.0;

	float fa, fb, fc1, fc2, newf;
	float f1, basef, famax = 0xf0, famin = 0x10;
	uint16_t c;
	int i;

        // allow a frequency 'power down'
	if (frequency == 0)
	{
		doall = true;
		basef = fa = 0;
		fb = fc1 = fc2 = 1;
	}
	else
	{
		f1 = limit_freq(info, frequency, false);
		fb = 2; fc1 = 1; fc2 = 5; // initial multiplier of 10
		if (f1 >= 500)
		{
			// halv down to '250-400'
			fb = 1;
		}
		else if (f1 <= 150)
		{
			// tiple up to '300-450'
			fc1 = 3;
		}
		else if (f1 <= 250)
		{
			// double up to '300-500'
			fc1 = 2;
		}
		// else f1 is 250-500

		// f1 * fb * fc1 * fc2 is between 2500 and 5000
		// - so round up to the next 25 (freq_mult)
		basef = info->freq_mult * ceil(f1 * fb * fc1 * fc2 / info->freq_mult);

		// fa should be between 100 (0x64) and 200 (0xC8)
		fa = basef / info->freq_mult;
	}

	// code failure ... basef isn't 400 to 6000
	if (frequency != 0 && (fa < famin || fa > famax))
	{
		info->freq_fail = frequency;
		newf = deffreq;
	}
	else
	{
		if (doall)
		{
			freqbufall[5] = (int)fa;
			freqbufall[6] = (int)fb;
			// fc1, fc2 'should' already be 1..15
			freqbufall[7] = (((int)fc1 & 0xf) << 4) + ((int)fc2 & 0xf);
		}
		else
		{
			freqbufch[5] = (int)fa;
			freqbufch[6] = (int)fb;
			// fc1, fc2 'should' already be 1..15
			freqbufch[7] = (((int)fc1 & 0xf) << 4) + ((int)fc2 & 0xf);
		}

		newf = basef / ((float)fb * (float)fc1 * (float)fc2);
	}

	if (doall)
	{
		// i.e. -1 means no reply since last set
		for (c = 0; c < info->chips; c++)
			info->asics[c].frequency_reply = -1;

		for (i = 0; i < 2; i++)
		{
			gekko_usleep(info, MS2US(10));
			compac_send2(compac, prefreqall, sizeof(prefreqall), 8 * sizeof(prefreqall) - 8, "prefreq");
		}
		for (i = 0; i < 2; i++)
		{
			gekko_usleep(info, MS2US(10));
			compac_send2(compac, freqbufall, sizeof(freqbufall), 8 * sizeof(freqbufall) - 8, "freq");
		}

		// the freq wanted, which 'should' be the same
		for (c = 0; c < info->chips; c++)
			info->asics[c].frequency = frequency;
	}
	else
	{
		// just setting 1 chip
		prefreqch[2] = freqbufch[2] = CHIPPY1397(info, chip);

		// i.e. -1 means no reply since last set
		info->asics[chip].frequency_reply = -1;

		for (i = 0; i < 2; i++)
		{
			gekko_usleep(info, MS2US(10));
			compac_send2(compac, prefreqch, sizeof(prefreqch), 8 * sizeof(prefreqch) - 8, "prefreq");
		}
		for (i = 0; i < 2; i++)
		{
			gekko_usleep(info, MS2US(10));
			compac_send2(compac, freqbufch, sizeof(freqbufch), 8 * sizeof(freqbufch) - 8, "freq");
		}

		// the freq wanted, which 'should' be the same
		info->asics[chip].frequency = frequency;
	}

	if (doall)
		info->frequency = frequency;

	gekko_usleep(info, MS2US(10));

	if (doall)
		snprintf(chipn, sizeof(chipn), "all");
	else
		snprintf(chipn, sizeof(chipn), "%d", chip);

//	applog(LOG_INFO, "%d: %s %d - setting frequency to %.2fMHz (%.2f)" " (%.0f/%.0f/%.0f/%.0f)",
	applog(LOG_ERR, "%d: %s %d - setting [%s] frequency to %.2fMHz (%.2f)" " (%.0f/%.0f/%.0f/%.0f)",
		compac->cgminer_id, compac->drv->name, compac->device_id, chipn, frequency, newf, fa, fb, fc1, fc2);

	ping_freq(compac, 0);
}

static void compac_send_chain_inactive(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	unsigned int i, j;

	applog(LOG_INFO,"%d: %s %d - sending chain inactive for %d chip(s)",
		compac->cgminer_id, compac->drv->name, compac->device_id, info->chips);

	if (info->asic_type == BM1397)
	{
		// chain inactive
		unsigned char chainin[5] = {0x53, 0x05, 0x00, 0x00, 0x00};
		for (i = 0; i < 3; i++)
		{
			compac_send2(compac, chainin, sizeof(chainin), 8 * sizeof(chainin) - 8, "chin");
			gekko_usleep(info, MS2US(100));
		}

		unsigned char chippy[] = {0x40, 0x05, 0x00, 0x00, 0x00};
		for (i = 0; i < info->chips; i++)
		{
			chippy[2] = CHIPPY1397(info, i);
			compac_send2(compac, chippy, sizeof(chippy), 8 * sizeof(chippy) - 8, "chippy");
			gekko_usleep(info, MS2US(10));
		}

		unsigned char init1[] = {0x51, 0x09, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};
		unsigned char init2[] = {0x51, 0x09, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00};
		unsigned char init3[] = {0x51, 0x09, 0x00, 0x20, 0x00, 0x00, 0x00, 0x01, 0x00};
		unsigned char init4[] = {0x51, 0x09, 0x00, 0x3C, 0x80, 0x00, 0x80, 0x74, 0x00};

		compac_send2(compac, init1, sizeof(init1), 8 * sizeof(init1) - 8, "init1");
		gekko_usleep(info, MS2US(10));
		compac_send2(compac, init2, sizeof(init2), 8 * sizeof(init2) - 8, "init2");
		gekko_usleep(info, MS2US(100));
		compac_send2(compac, init3, sizeof(init3), 8 * sizeof(init3) - 8, "init3");
		gekko_usleep(info, MS2US(50));
		compac_send2(compac, init4, sizeof(init4), 8 * sizeof(init4) - 8, "init4");
		gekko_usleep(info, MS2US(100));

		// set ticket based on chips, pool will be above this anyway
		set_ticket(compac, 0.0, true, false);

		unsigned char init5[] = {0x51, 0x09, 0x00, 0x68, 0xC0, 0x70, 0x01, 0x11, 0x00};
		unsigned char init6[] = {0x51, 0x09, 0x00, 0x28, 0x06, 0x00, 0x00, 0x0F, 0x00};

		for (j = 0; j < 2; j++)
		{
			compac_send2(compac, init5, sizeof(init5), 8 * sizeof(init5) - 8, "init5");
			gekko_usleep(info, MS2US(50));
		}
		compac_send2(compac, init6, sizeof(init6), 8 * sizeof(init6) - 8, "init6");
		gekko_usleep(info, MS2US(100));

		unsigned char baudrate[] = { 0x51, 0x09, 0x00, 0x18, 0x00, 0x00, 0x61, 0x31, 0x00 }; // lo 1.51M
		info->bauddiv = 1; // 1.5M

#ifdef WIN32___fixme_zzz
		if (info->midstates == 4)
		{
			// 4 mid = slow it down on windows 116K
			baudrate[5] = 0x00;
			baudrate[6] = 0x7A;
			// 3.125M/27
			info->bauddiv = 26;
		}
#endif

		applog(LOG_ERR, "%d: %s %d - setting bauddiv : %02x %02x (ftdi/%d)",
			compac->cgminer_id, compac->drv->name, compac->device_id, baudrate[5], baudrate[6], info->bauddiv + 1);
		compac_send2(compac, baudrate, sizeof(baudrate), 8 * sizeof(baudrate) - 8, "baud");
		gekko_usleep(info, MS2US(10));

		usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, info->bauddiv + 1,
				(FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);
		gekko_usleep(info, MS2US(10));

		calc_gsf_freq(compac, info->frequency, -1);

		gekko_usleep(info, MS2US(20));
	} else if (info->asic_type == BM1387) {
		unsigned char buffer[5] = {0x55, 0x05, 0x00, 0x00, 0x00};
		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8); // chain inactive
		gekko_usleep(info, MS2US(5));
		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8); // chain inactive
		gekko_usleep(info, MS2US(5));
		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8); // chain inactive
		for (i = 0; i < info->chips; i++) {
			buffer[0] = 0x41;
			buffer[1] = 0x05;
			buffer[2] = (0x100 / info->chips) * i;
			gekko_usleep(info, MS2US(5));
			compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
		}

		gekko_usleep(info, MS2US(10));
		unsigned char baudrate[] = { 0x58, 0x09, 0x00, 0x1C, 0x00, 0x20, 0x07, 0x00, 0x19 };
		if (opt_gekko_bauddiv) {
			info->bauddiv = opt_gekko_bauddiv;
		} else {
			info->bauddiv = 0x01; // 1.5Mbps baud.
#ifdef WIN32
			if (info->midstates == 4)
				info->bauddiv = 0x0D; // 214Kbps baud.
#endif
		}
		applog(LOG_INFO, "%d: %s %d - setting bauddiv : %02x",
			compac->cgminer_id, compac->drv->name, compac->device_id, info->bauddiv);
		baudrate[6] = info->bauddiv;
		compac_send(compac, baudrate, sizeof(baudrate), 8 * sizeof(baudrate) - 8);
		gekko_usleep(info, MS2US(10));
		usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, (info->bauddiv + 1),
				(FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);
		gekko_usleep(info, MS2US(10));

		unsigned char gateblk[9] = {0x58, 0x09, 0x00, 0x1C, 0x40, 0x20, 0x99, 0x80, 0x01};
		gateblk[6] = 0x80 | info->bauddiv;
		compac_send(compac, gateblk, sizeof(gateblk), 8 * sizeof(gateblk) - 8); // chain inactive
	} else if (info->asic_type == BM1384) {
		unsigned char buffer[] = {0x85, 0x00, 0x00, 0x00};
		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5); // chain inactive
		for (i = 0; i < info->chips; i++) {
			buffer[0] = 0x01;
			buffer[1] = (0x100 / info->chips) * i;
			compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		}
		buffer[0] = 0x86; // GATEBLK
		buffer[1] = 0x00;
		buffer[2] = 0x9a; // 0x80 | 0x1a;
		//compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
	}

	if (info->mining_state == MINER_CHIP_COUNT_OK) {
		applog(LOG_INFO, "%d: %s %d - open cores",
			compac->cgminer_id, compac->drv->name, compac->device_id);
		info->zero_check = 0;
		info->task_hcn = 0;
		info->mining_state = MINER_OPEN_CORE;
	}
}

static void compac_update_rates(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct ASIC_INFO *asic;
	float average_frequency = 0, est;
	unsigned int i;

	cgtime(&(info->last_update_rates));

	if (info->chips == 0 || info->frequency == 0)
		return;

	for (i = 0; i < info->chips; i++)
	{
		if (info->asics[i].frequency == 0)
			return;
	}

	info->frequency_asic = 0;
	for (i = 0; i < info->chips; i++) {
		asic = &info->asics[i];
		asic->hashrate = asic->frequency * info->cores * 1000000 * info->hr_scale;
		asic->fullscan_ms = 1000.0 * info->hr_scale * 0xffffffffull / asic->hashrate;
		asic->fullscan_us = 1000.0 * info->hr_scale * 1000.0 * 0xffffffffull / asic->hashrate;
		average_frequency += asic->frequency;
		info->frequency_asic = (asic->frequency > info->frequency_asic ) ? asic->frequency : info->frequency_asic;
	}

	average_frequency = average_frequency / info->chips;
	if (average_frequency != info->frequency) {
		applog(LOG_INFO,"%d: %s %d - frequency updated %.2fMHz -> %.2fMHz",
			compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency, average_frequency);
		info->frequency = average_frequency;
		info->wu_max = 0;
	}

	info->wu = (info->chips * info->frequency * info->cores / 71.6) * info->hr_scale;
	info->hashrate = info->chips * info->frequency * info->cores * 1000000 * info->hr_scale;
	info->fullscan_ms = 1000.0 * info->hr_scale * 0xffffffffull / info->hashrate;
	info->fullscan_us = 1000.0 * info->hr_scale * 1000.0 * 0xffffffffull / info->hashrate;
	if (info->asic_type != BM1397)
	{
		info->ticket_mask = bound(pow(2, ceil(log(info->hashrate / (2.0 * 0xffffffffull)) / log(2))) - 1, 0, 4000);
		info->ticket_mask = (info->asic_type == BM1387) ? 0 : info->ticket_mask;
		info->difficulty = info->ticket_mask + 1;
	}

	info->wait_factor = info->wait_factor0;
	if (!opt_gekko_noboost && info->vmask && (info->asic_type == BM1387 || info->asic_type == BM1397))
		info->wait_factor *= info->midstates;
	est = info->wait_factor * (float)(info->fullscan_us);
	info->max_task_wait = bound((uint64_t)est, 1, 3 * info->fullscan_us);

	if (info->asic_type == BM1387)
	{
		if (opt_gekko_tune_up > 95)
			info->tune_up = 100.0 * ((info->frequency - info->freq_base * (600 / info->frequency)) / info->frequency);
		else
			info->tune_up = opt_gekko_tune_up;
	}
	else if (info->asic_type == BM1397)
	{
		// 90% will always allow at least 2 freq steps
		if (opt_gekko_tune_up > 90)
			opt_gekko_tune_up = 90;
		else
			info->tune_up = opt_gekko_tune_up;
	}
	else
		info->tune_up = 99;

	// shouldn't happen, but next call should fix it
	if (info->difficulty == 0)
		info->nonce_expect = 0;
	else
	{
		// expected ms per nonce for PT_NONONCE
		info->nonce_expect = info->fullscan_ms * info->difficulty;
		// BM1397 check is per miner, not per chip, fullscan_ms is sum of chips
		if (info->asic_type != BM1397 && info->chips > 1)
			info->nonce_expect *= (float)info->chips;

		// CDF >2000% is avg once in 485165205.1 nonces
		info->nonce_limit = info->nonce_expect * 20.0;

		// N.B. this ignores info->ghrequire since
		//  that should be an independent test
	}

	applog(LOG_INFO, "%d: %s %d - Rates: ms %.2f tu %.2f td %.2f",
		compac->cgminer_id, compac->drv->name, compac->device_id,
		info->fullscan_ms, info->tune_up, info->tune_down);
}

static void compac_set_frequency_single(struct cgpu_info *compac, float frequency, int asic_id)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct ASIC_INFO *asic = &info->asics[asic_id];
	struct timeval now;

	if (info->asic_type == BM1387) {
		unsigned char buffer[] = {0x48, 0x09, 0x00, 0x0C, 0x00, 0x50, 0x02, 0x41, 0x00};   //250MHz -- osc of 25MHz
		frequency = bound(frequency, 50, 1200);
		frequency = FREQ_BASE(frequency);

		if (frequency < 400) {
			buffer[7] = 0x41;
			buffer[5] = (frequency * 8) / info->freq_mult;
		} else {
			buffer[7] = 0x21;
			buffer[5] = (frequency * 4) / info->freq_mult;
		}
		buffer[2] = (0x100 / info->chips) * asic_id;

		//asic->frequency = frequency;
		asic->frequency_set = frequency;
		asic->frequency_attempt++;

		applog(LOG_INFO, "%d: %s %d - setting chip[%d] frequency (%d) %.2fMHz -> %.2fMHz",
			compac->cgminer_id, compac->drv->name, compac->device_id,
			asic_id, asic->frequency_attempt, asic->frequency, frequency);

		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);

		//unsigned char gateblk[9] = {0x48, 0x09, 0x00, 0x1C, 0x40, 0x20, 0x99, 0x80, 0x01};
		//gateblk[6] = 0x80 | info->bauddiv;
		//gateblk[2] = (0x100 / info->chips) * id;
		//compac_send(compac, gateblk, sizeof(gateblk), 8 * sizeof(gateblk) - 8); // chain inactive

		// wipe info->gh/asic->gc
		cgtime(&now);
		gh_offset(info, &now, true, false);
		// reset P:
		info->frequency_computed = 0;
	}
}

static void compac_set_frequency(struct cgpu_info *compac, float frequency)
{
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t i, r, r1, r2, r3, p1, p2, pll;
	struct timeval now;

	if (info->asic_type == BM1397) {
		calc_gsf_freq(compac, frequency, -1);
	} else if (info->asic_type == BM1387) {
		unsigned char buffer[] = {0x58, 0x09, 0x00, 0x0C, 0x00, 0x50, 0x02, 0x41, 0x00};   //250MHz -- osc of 25MHz
		frequency = bound(frequency, 50, 1200);
		frequency = FREQ_BASE(frequency);

		if (frequency < 400) {
			buffer[7] = 0x41;
			buffer[5] = (frequency * 8) / info->freq_mult;
		} else {
			buffer[7] = 0x21;
			buffer[5] = (frequency * 4) / info->freq_mult;
		}
/*
		} else {
			buffer[7] = 0x11;
			buffer[5] = (frequency * 2) / info->freq_mult;
		}
*/

		applog(LOG_INFO, "%d: %s %d - setting frequency to %.2fMHz",
			compac->cgminer_id, compac->drv->name, compac->device_id, frequency);

		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);

		info->frequency = frequency;
		for (i = 0; i < info->chips; i++)
			info->asics[i].frequency = frequency;
	} else if (info->asic_type == BM1384) {
		unsigned char buffer[] = {0x82, 0x0b, 0x83, 0x00};

		frequency = bound(frequency, 6, 500);
		frequency = FREQ_BASE(frequency);

		info->frequency = frequency;

		r = floor(log(info->frequency/info->freq_mult) / log(2));

		r1 = 0x0785 - r;
		r2 = 0x200 / pow(2, r);
		r3 = info->freq_mult * pow(2, r);

		p1 = r1 + r2 * (info->frequency - r3) / info->freq_base;
		p2 = p1 * 2 + (0x7f + r);

		pll = (((uint32_t)(info->frequency) % (uint32_t)(info->freq_mult)) == 0 ? p1 : p2);

		if (info->frequency < 100) {
			pll = 0x0783 - 0x80 * (100 - info->frequency) / info->freq_base;
		}

		buffer[1] = (pll >> 8) & 0xff;
		buffer[2] = (pll) & 0xff;

		applog(LOG_INFO, "%d: %s %d - setting frequency to %.2fMHz",
			compac->cgminer_id, compac->drv->name, compac->device_id, frequency);

		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		buffer[0] = 0x84;
		buffer[1] = 0x00;
		buffer[2] = 0x00;
//		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		buffer[2] = 0x04;
		compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);

		for (i = 0; i < info->chips; i++)
			info->asics[i].frequency = frequency;
	}
	compac_update_rates(compac);

	// wipe info->gh/asic->gc
	cgtime(&now);
	gh_offset(info, &now, true, false);
	// reset P:
	info->frequency_computed = 0;
}

static void compac_update_work(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	int i;

	for (i = 0; i < JOB_MAX; i++) {
		info->active_work[i] = false;
	}
	info->update_work = 1;
}

static void compac_flush_buffer(struct cgpu_info *compac)
{
	int read_bytes = 1;
	unsigned char resp[32];

	while (read_bytes) {
		usb_read_timeout(compac, (char *)resp, 32, &read_bytes, 1, C_REQUESTRESULTS);
	}
}

static void compac_flush_work(struct cgpu_info *compac)
{
	compac_flush_buffer(compac);
	compac_update_work(compac);
}



struct gpiod_chip *gpio_chip;
struct gpiod_line *gpio_line;

#define GPIO_CHIP_NAME "gpiochip1"
#define GPIO_LINE_OFFSET 28

int init_gpio()
{
    gpio_chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if (!gpio_chip)
    {
        perror("Failed to open GPIO chip");
        return -1;
    }
    // Get the GPIO line
    gpio_line = gpiod_chip_get_line(gpio_chip, GPIO_LINE_OFFSET);
    if (!gpio_line)
    {
        perror("Failed to get GPIO line");
        gpiod_chip_close(gpio_chip);
        return -1;
    }

    // Request the GPIO line
    int ret = gpiod_line_request_output(gpio_line, "gpio-reset", 0);
    if (ret < 0)
    {
        perror("Failed to request GPIO line");
        gpiod_line_release(gpio_line);
        gpiod_chip_close(gpio_chip);
        return -1;
    }
    return 0;

}

void toggle_gpio()
{
    // Toggle the GPIO line
    gpiod_line_set_value(gpio_line, 0);
    usleep(5000000); // Delay for 100 milliseconds
    gpiod_line_set_value(gpio_line, 1);
}
static void compac_toggle_reset(struct cgpu_info *compac)
{
	
	struct COMPAC_INFO *info = compac->device_data;
	unsigned short usb_val;

	applog(info->log_wide,"%d: %s %d - Toggling ASIC nRST to reset",
		compac->cgminer_id, compac->drv->name, compac->device_id);

	// toggle gpio reset pin to reset ASIC


	int fd;

	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_RESET, info->interface, C_RESET);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_DATA, FTDI_VALUE_DATA_BTS, info->interface, C_SETDATA);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, FTDI_VALUE_BAUD_BTS, (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_FLOW, FTDI_VALUE_FLOW, info->interface, C_SETFLOW);

	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_PURGE_TX, info->interface, C_PURGETX);
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_PURGE_RX, info->interface, C_PURGERX);

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF2; // low byte: bitmask - 1111 0010 - CB1(HI)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	gekko_usleep(info, MS2US(30));

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF0; // low byte: bitmask - 1111 0000 - CB1(LO)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	if (info->asic_type == BM1397)
		gekko_usleep(info, MS2US(1000));
	else
		gekko_usleep(info, MS2US(30));

	usb_val = (FTDI_BITMODE_CBUS << 8) | 0xF2; // low byte: bitmask - 1111 0010 - CB1(HI)
	usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BITMODE, usb_val, info->interface, C_SETMODEM);
	gekko_usleep(info, MS2US(200));

	cgtime(&info->last_reset);
}

static void compac_gsf_nonce(struct cgpu_info *compac, K_ITEM *item)
{
	struct COMPAC_INFO *info = compac->device_data;
	unsigned char *rx = DATA_NONCE(item)->rx;
	int hwe = compac->hw_errors;
	struct work *work = NULL;
	bool active_work = false;
	uint32_t job_id = 0;
	uint32_t nonce = 0;
	int domid, midnum = 0;
	double diff = 0.0;
	bool boost, ok;
	int asic_id, i;

	if (info->asic_type != BM1397)
		return;

	job_id = rx[7] & 0xff;
	nonce = (rx[5] << 0) | (rx[4] << 8) | (rx[3] << 16) | (rx[2] << 24);

	// N.B. busy work (0xff) never returns a nonce

	mutex_lock(&info->lock);
	info->nonces++;
	info->nonceless = 0;
	info->noncebyte[rx[3]]++;
	mutex_unlock(&info->lock);

	if (info->nb2c_setup)
		asic_id = info->nb2chip[rx[3]];
	else
		asic_id = floor((double)(rx[4]) / ((double)0x100 / (double)(info->chips)));

	if (asic_id >= (int)(info->chips))
        {
		applog(LOG_ERR, "%d: %s %d - nonce %08x @ %02x rx[4] %02x invalid asic_id (0..%d)",
			compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id,
			rx[4], (int)(info->chips)-1);
		asic_id = (info->chips - 1);
        }
#if 0
else
{
		applog(LOG_ERR, "%d: %s %d - gotnonce %08x @ %02x rx[2] %02x id %d(%u)",
			compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id,
			rx[2], asic_id, info->chips);
		applog(LOG_ERR, " N:[%02x %02x %02x %02x %02x %02x %02x]",
			rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);
}
#endif

	struct ASIC_INFO *asic = &info->asics[asic_id];

	if (nonce == asic->prev_nonce)
	{
		applog(LOG_INFO, "%d: %s %d - Duplicate Nonce : %08x @ %02x [%02x %02x %02x %02x %02x %02x %02x]",
			compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id,
			rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);

		mutex_lock(&info->lock);
		info->dups++;
		info->dupsall++;
		info->dupsreset++;
		asic->dups++;
		asic->dupsall++;
		cgtime(&info->last_dup_time);
		if (info->dups == 1)
			info->mining_state = MINER_MINING_DUPS;
		mutex_unlock(&info->lock);

		return;
	}

	mutex_lock(&info->lock);
	info->prev_nonce = nonce;
	asic->prev_nonce = nonce;

	applog(LOG_INFO, "%d: %s %d - Device reported nonce: %08x @ %02x (%d)",
		compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id, info->tracker);

	if (!opt_gekko_noboost && info->vmask)
	{
		domid = info->midstates;
		boost = true;
	}
	else
	{
		domid = 1;
		boost = false;
	}

	ok = false;

	// test the exact jobid/midnum
	uint32_t w_job_id = job_id & 0xfc;
	if (w_job_id <= info->max_job_id)
	{
		work = info->work[w_job_id];
		active_work = info->active_work[w_job_id];
		if (work && active_work)
		{
			if (boost)
			{
				midnum = job_id & 0x03;
				work->micro_job_id = pow(2, midnum);
				memcpy(work->data, &(work->pool->vmask_001[work->micro_job_id]), 4);
			}

			if ((diff = test_nonce_value(work, nonce)) != 0.0)
			{
				ok = true;
				if (midnum > 0)
					info->boosted = true;
			}
		}
	}

	if (!ok)
	{
		// not found, try each cur_attempt
		for (i = 0; !ok && i < (int)CUR_ATTEMPT; i++)
		{
			w_job_id = JOB_ID_ROLL(info->job_id, cur_attempt[i], info) & 0xfc;
			work = info->work[w_job_id];
			active_work = info->active_work[w_job_id];
			if (work && active_work)
			{
				for (midnum = 0; !ok && midnum < domid; midnum++)
				{
					// failed original job_id already tested
					if ((w_job_id | midnum) == job_id)
						continue;

					if (boost)
					{
						work->micro_job_id = pow(2, midnum);
						memcpy(work->data, &(work->pool->vmask_001[work->micro_job_id]), 4);
					}
					if ((diff = test_nonce_value(work, nonce)) != 0)
					{
						if (midnum > 0)
							info->boosted = true;
						info->cur_off[i]++;
						ok = true;

						applog(LOG_INFO, "%d: %s %d - Nonce Recovered : %08x @ job[%02x]->fix[%02x] len %u prelen %u",
							compac->cgminer_id, compac->drv->name, compac->device_id,
							nonce, job_id, w_job_id, (uint32_t)(DATA_NONCE(item)->len),
							(uint32_t)(DATA_NONCE(item)->prelen));
					}
				}
			}
		}

		if (!ok)
		{
			applog(LOG_INFO, "%d: %s %d - Nonce Dumped : %08x @ job[%02x] cur[%02x] diff %u",
				compac->cgminer_id, compac->drv->name, compac->device_id,
				nonce, job_id, info->job_id, info->difficulty);

			inc_hw_errors_n(info->thr, info->difficulty);
			cgtime(&info->last_hwerror);
			mutex_unlock(&info->lock);
			return;
		}
	}

	// verify the ticket mask is correct
	if (!info->ticket_ok && diff > 0)
	{
		do // so we can break out
		{
			if (++(info->ticket_work) > TICKET_DELAY)
			{
				int i = info->ticket_number;
				info->ticket_nonces++;
				// nonce below ticket setting - redo ticket - chip must be too low
				// check this for all nonces until ticket_ok
				if (diff < ticket_1397[i].hi_limit)
				{
					if (++(info->below_nonces) < TICKET_BELOW_LIM)
						break;

					// redo ticket to return fewer nonces

					if (info->ticket_failures > MAX_TICKET_CHECK)
					{
						// give up - just set it to max

						applog(LOG_ERR, "%d: %s %d - ticket %u failed too many times setting to max",
							compac->cgminer_id, compac->drv->name, compac->device_id, ticket_1397[i].diff);
						//set_ticket(compac, 1.0, true, true);
						set_ticket(compac, 0.0, true, true);
						info->ticket_ok = true;
						break;
					}

					applog(LOG_ERR, "%d: %s %d - ticket %u failure (%"PRId64") diff %.1f below lim %.1f - retry %d",
						compac->cgminer_id, compac->drv->name, compac->device_id,
						ticket_1397[i].diff, info->below_nonces, diff, ticket_1397[i].hi_limit, info->ticket_failures+1);

					// try again ...
					//set_ticket(compac, ticket_1397[i].diff, true, true);
					set_ticket(compac, 0.0, true, true);
					info->ticket_failures++;
					break;
				}
				// after nonce_count, CDF[Erlang] chance of NOT being below
				if (diff < ticket_1397[i].low_limit)
					info->ticket_got_low = true;

				if (info->ticket_work >= (ticket_1397[i].nonce_count + TICKET_DELAY))
				{
					// we should have got a 'low' by now
					if (info->ticket_got_low)
					{
						info->ticket_ok = true;
						info->ticket_failures = 0;

						applog(LOG_ERR, "%d: %s %d - ticket value confirmed 0x%02x/%u after %"PRId64" nonces",
							compac->cgminer_id, compac->drv->name, compac->device_id,
							info->ticket_mask, info->difficulty, info->ticket_nonces);
						break;
					}

					// chip ticket must be too high means lost shares

					if (info->ticket_failures > MAX_TICKET_CHECK)
					{
						// give up - just set it to 1.0

						applog(LOG_ERR, "%d: %s %d - ticket %u failed too many times setting to max",
							compac->cgminer_id, compac->drv->name, compac->device_id, ticket_1397[i].diff);

						//set_ticket(compac, 1.0, true, true);
						set_ticket(compac, 0.0, true, true);
						info->ticket_ok = true;
						break;
					}

					applog(LOG_ERR, "%d: %s %d - ticket %u failure no low < %.1f after %d - retry %d",
						compac->cgminer_id, compac->drv->name, compac->device_id,
						ticket_1397[i].diff, ticket_1397[i].low_limit, info->ticket_work, info->ticket_failures+1);

					// try again ...
					//set_ticket(compac, ticket_1397[i].diff, true, true);
					set_ticket(compac, 0.0, true, true);
					info->ticket_failures++;
					break;
				}
			}
		}
		while (0);
	}

	if (active_work && work)
		work->device_diff = info->difficulty;

	mutex_unlock(&info->lock);

	if (active_work && work && submit_nonce(info->thr, work, nonce))
	{
		mutex_lock(&info->lock);

		cgtime(&info->last_nonce);
		cgtime(&asic->last_nonce);

		// count of valid nonces
		asic->nonces++; // info only

		if (midnum > 0)
		{
			applog(LOG_INFO, "%d: %s %d - AsicBoost nonce found : midstate %d",
				compac->cgminer_id, compac->drv->name, compac->device_id, midnum);
		}

		// if work diff < info->dificulty, 'accept' hash rate will be low
		info->hashes += info->difficulty * 0xffffffffull;
		info->xhashes += info->difficulty;

		info->accepted++;
		info->failing = false;
		info->dups = 0;
		asic->dups = 0;
		mutex_unlock(&info->lock);

		if (info->nb2c_setup)
			add_gekko_nonce(info, asic, &(DATA_NONCE(item)->when));
		else
			add_gekko_nonce(info, NULL, &(DATA_NONCE(item)->when));
	}
	else
	{
		// shouldn't be possible since diff has already been checked
		if (hwe != compac->hw_errors)
		{
			mutex_lock(&info->lock);
			cgtime(&info->last_hwerror);
			mutex_unlock(&info->lock);
		}
	}
}

static uint64_t compac_check_nonce(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	uint32_t nonce = 0;
	int hwe = compac->hw_errors;
	uint32_t job_id = 0;
	uint64_t hashes = 0;
	struct timeval now;
	int i;

	if (info->asic_type == BM1387) {
		job_id = info->rx[5] & 0xff;
		nonce = (info->rx[3] << 0) | (info->rx[2] << 8) | (info->rx[1] << 16) | (info->rx[0] << 24);
	} else if (info->asic_type == BM1384) {
		job_id = info->rx[4] ^ 0x80;
		nonce = (info->rx[3] << 0) | (info->rx[2] << 8) | (info->rx[1] << 16) | (info->rx[0] << 24);
	} else if (info->asic_type == BM1397) {
		// should never call here
		return hashes;
	}

	if ((info->rx[0] == 0x72 && info->rx[1] == 0x03 && info->rx[2] == 0xEA && info->rx[3] == 0x83)
	||  (info->rx[0] == 0xE1 && info->rx[1] == 0x6B && info->rx[2] == 0xF8 && info->rx[3] == 0x09))
	{
		//busy work nonces
		return hashes;
	}

	if (job_id > info->max_job_id
	||  (abs((int)(info->job_id) - (int)job_id) > 3 && abs((int)(info->max_job_id) - (int)job_id + (int)(info->job_id)) > 3))
	{
		return hashes;
	}

	if (!info->active_work[job_id]
	&&  !(job_id > 0 && info->active_work[job_id - 1])
	&&  !(job_id > 1 && info->active_work[job_id - 2])
	&&  !(job_id > 2 && info->active_work[job_id - 3]))
	{
		return hashes;
	}

	cgtime(&now);

	info->nonces++;
	info->nonceless = 0;

	int asic_id = (int)floor((double)(info->rx[0]) / ((double)0x100 / (double)(info->chips)));
	struct ASIC_INFO *asic = &info->asics[asic_id];

	if (nonce == asic->prev_nonce) {
		applog(LOG_INFO, "%d: %s %d - Duplicate Nonce : %08x @ %02x [%02x %02x %02x %02x %02x %02x]",
			compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id,
			info->rx[0], info->rx[1], info->rx[2], info->rx[3], info->rx[4], info->rx[5]);

		info->dups++;
		info->dupsall++;
		info->dupsreset++;
		asic->dups++;
		asic->dupsall++;
		cgtime(&info->last_dup_time);
		if (info->dups == 1) {
			info->mining_state = MINER_MINING_DUPS;
		}

		return hashes;
	}

	info->prev_nonce = nonce;
	asic->prev_nonce = nonce;

	applog(LOG_INFO, "%d: %s %d - Device reported nonce: %08x @ %02x (%d)",
		compac->cgminer_id, compac->drv->name, compac->device_id, nonce, job_id, info->tracker);

	struct work *work = info->work[job_id];
	bool active_work = info->active_work[job_id];
	int midnum = 0;
	if (!opt_gekko_noboost && info->vmask)
	{
		// force check last few nonces by [job_id - 1]
		// doesn't handle job_id roll over
		if (info->asic_type == BM1387)
		{
			for (i = 0; i < info->midstates; i++)
			{
				if ((int)job_id >= i)
				{
					if (info->active_work[job_id - i])
					{
						work = info->work[job_id - i];
						active_work = info->active_work[job_id - i];
						if (active_work && work)
						{
							work->micro_job_id = pow(2, i);
							memcpy(work->data, &(work->pool->vmask_001[work->micro_job_id]), 4);
							if (test_nonce(work, nonce))
							{
								midnum = i;
								if (i > 0)
									info->boosted = true;

								break;
							}
						}
					}
				}
			}
		}
	}

	if (!active_work || !work) {
		return hashes;
	}

	work->device_diff = info->difficulty;

	if (submit_nonce(info->thr, work, nonce)) {
		cgtime(&info->last_nonce);
		cgtime(&asic->last_nonce);

		// count of valid nonces
		asic->nonces++; // info only

		if (midnum > 0) {
			applog(LOG_INFO, "%d: %s %d - AsicBoost nonce found : midstate%d",
				compac->cgminer_id, compac->drv->name, compac->device_id, midnum);
		}

		hashes = info->difficulty * 0xffffffffull;
		info->xhashes += info->difficulty;

		info->accepted++;
		info->failing = false;
		info->dups = 0;
		asic->dups = 0;

		add_gekko_nonce(info, asic, &now);
	} else {
		if (hwe != compac->hw_errors) {
			cgtime(&info->last_hwerror);
		}
	}

	return hashes;
}

static void busy_work(struct COMPAC_INFO *info)
{
	memset(info->task, 0, info->task_len);

	if (info->asic_type == BM1387 || info->asic_type == BM1397) {
		info->task[0] = 0x21;
		info->task[1] = info->task_len;
		info->task[2] = info->job_id & 0xff;
		info->task[3] = ((!opt_gekko_noboost && info->vmask) ? 0x04 : 0x01);
		memset(info->task + 8, 0xff, 12);

		unsigned short crc = crc16_false(info->task, info->task_len - 2);
		info->task[info->task_len - 2] = (crc >> 8) & 0xff;
		info->task[info->task_len - 1] = crc & 0xff;
	} else if (info->asic_type == BM1384) {
		if (info->mining_state == MINER_MINING) {
			info->task[39] = info->ticket_mask & 0xff;
			stuff_msb(info->task + 40, info->task_hcn);
		}
		info->task[51] = info->job_id & 0xff;
	}
}

static void init_task(struct COMPAC_INFO *info)
{
	struct work *work = info->work[info->job_id];

	memset(info->task, 0, info->task_len);

	if (info->asic_type == BM1387 || info->asic_type == BM1397) {
		info->task[0] = 0x21;
		info->task[1] = info->task_len;
		info->task[2] = info->job_id & 0xff;
		info->task[3] = ((!opt_gekko_noboost && info->vmask) ? info->midstates : 0x01);

		if (info->mining_state == MINER_MINING) {
			stuff_reverse(info->task + 8, work->data + 64, 12);
			stuff_reverse(info->task + 20, work->midstate, 32);
			if (!opt_gekko_noboost && info->vmask) {
				if (info->midstates > 1)
					stuff_reverse(info->task + 20 + 32, work->midstate1, 32);
				if (info->midstates > 2)
					stuff_reverse(info->task + 20 + 32 + 32, work->midstate2, 32);
				if (info->midstates > 3)
					stuff_reverse(info->task + 20 + 32 + 32 + 32, work->midstate3, 32);
			}
		} else {
			memset(info->task + 8, 0xff, 12);
		}
		unsigned short crc = crc16_false(info->task, info->task_len - 2);
		info->task[info->task_len - 2] = (crc >> 8) & 0xff;
		info->task[info->task_len - 1] = crc & 0xff;
	} else if (info->asic_type == BM1384) {
		if (info->mining_state == MINER_MINING) {
			stuff_reverse(info->task, work->midstate, 32);
			stuff_reverse(info->task + 52, work->data + 64, 12);
			info->task[39] = info->ticket_mask & 0xff;
			stuff_msb(info->task + 40, info->task_hcn);
		}
		info->task[51] = info->job_id & 0xff;
	}
}

static void change_freq_any(struct cgpu_info *compac, float new_freq)
{
	struct COMPAC_INFO *info = compac->device_data;
	unsigned int i;
	float old_freq;

	old_freq = info->frequency;
	for (i = 0; i < info->chips; i++)
	{
		struct ASIC_INFO *asic = &info->asics[i];
		cgtime(&info->last_frequency_adjust);
		cgtime(&asic->last_frequency_adjust);
		cgtime(&info->monitor_time);
		asic->frequency_updated = 1;
		if (info->asic_type == BM1397)
		{
			if (i == 0)
				compac_set_frequency(compac, new_freq);
		}
		else if (info->asic_type == BM1387)
		{
			compac_set_frequency_single(compac, new_freq, i);
			info->frequency = new_freq;
		}
		else if (info->asic_type == BM1384)
		{
			if (i == 0)
			{
				compac_set_frequency(compac, new_freq);
				compac_send_chain_inactive(compac);
				info->frequency = new_freq;
			}
		}
	}

	applog(LOG_WARNING,"%d: %s %d - new frequency %.2fMHz -> %.2fMHz",
		compac->cgminer_id, compac->drv->name, compac->device_id,
		old_freq, new_freq);
}

// compac_mine() with long term adjustments
static void *compac_mine2(void *object)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;
	struct work *work = NULL;
	struct work *old_work = NULL;

	struct timeval now;
	struct timeval last_rolling = (struct timeval){0};
	struct timeval last_movement = (struct timeval){0};
	struct timeval last_plateau_check = (struct timeval){0};
	struct timeval last_frequency_check = (struct timeval){0};

	struct sched_param param;
	int sent_bytes, sleep_us, use_us, policy, ret_nice;
	double diff_us, left_us;
	unsigned int i, j;
	uint32_t err = 0;

	uint64_t hashrate_gs;
	double dev_runtime, wu;
	float frequency_computed;
	bool frequency_updated;
	bool has_freq;
	bool job_added;
	bool last_was_busy = false;

	int plateau_type = 0;

#ifndef WIN32
	ret_nice = nice(-15);
#else /* WIN32 */
	pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy);
	pthread_setschedparam(pthread_self(), policy, &param);
	ret_nice = param.sched_priority;
#endif /* WIN32 */
	applog(LOG_INFO, "%d: %s %d - work thread niceness (%d)",
		compac->cgminer_id, compac->drv->name, compac->device_id, ret_nice);

	sleep_us = 100;

	cgtime(&last_plateau_check);

	while (info->mining_state != MINER_SHUTDOWN)
	{
		if (old_work)
		{
			mutex_lock(&info->lock);
			work_completed(compac, old_work);
			mutex_unlock(&info->lock);
			old_work = NULL;
		}

		if (info->chips == 0
		||  compac->deven == DEV_DISABLED
		||  compac->usbinfo.nodev
		||  (info->mining_state != MINER_MINING && info->mining_state != MINER_MINING_DUPS))
		{
			gekko_usleep(info, MS2US(10));
			continue;
		}

		cgtime(&now);

		if (!info->update_work)
		{
			diff_us = us_tdiff(&now, &info->last_task);
			left_us = info->max_task_wait - diff_us;
			if (left_us > 0)
			{
				// allow 300us from here to next sleep
				left_us -= 300;
				use_us = sleep_us;
				if (use_us > left_us)
					use_us = left_us;
				// 1ms is more than enough
				if (use_us > 1000)
					use_us = 1000;
				if (use_us >= USLEEPMIN)
				{
					gekko_usleep(info, use_us);
					continue;
				}
			}
		}

		frequency_updated = 0;
		info->update_work = 0;

		sleep_us = bound(ceil(info->max_task_wait / 20), 1, 100 * 1000); // 1us .. 100ms

		dev_runtime = cgpu_runtime(compac);
		wu = compac->diff1 / dev_runtime * 60;

		if (wu > info->wu_max)
		{
			info->wu_max = wu;
			cgtime(&info->last_wu_increase);
		}

		// don't change anything until we have 10s of data since a reset
		if (ms_tdiff(&now, &info->last_reset) > MS_SECOND_10)
		{
			if (ms_tdiff(&now, &last_rolling) >= MS_SECOND_1)
			{
				mutex_lock(&info->ghlock);
				if (info->gh.noncesum > (GHNONCENEEDED+1))
				{
					cgtime(&last_rolling);
					info->rolling = gekko_gh_hashrate(info, &last_rolling, true);
				}
				mutex_unlock(&info->ghlock);
			}

			hashrate_gs = (double)info->rolling * 1000000ull;
			info->eff_gs = 100.0 * (1.0 * hashrate_gs / info->hashrate);
			info->eff_wu = 100.0 * (1.0 * wu / info->wu);
			if (info->eff_gs > 100)
				info->eff_gs = 100;
			if (info->eff_wu > 100)
				info->eff_wu = 100;

			frequency_computed = ((hashrate_gs / 1.0e6) / info->cores) / info->chips;
			frequency_computed = limit_freq(info, frequency_computed, true);
			if (frequency_computed > info->frequency_computed
			&&  frequency_computed <= info->frequency)
			{
				info->frequency_computed = frequency_computed;
				cgtime(&info->last_computed_increase);

				applog(LOG_INFO, "%d: %s %d - new comp=%.2f (gs=%.2f)",
					compac->cgminer_id, compac->drv->name, compac->device_id, frequency_computed,
					((double)hashrate_gs)/1.0e6);
			}

			plateau_type = 0;
			// search for plateau
			if (ms_tdiff(&now, &last_plateau_check) > MS_SECOND_5)
			{
				has_freq = false;
				for (i = 0; i < info->chips; i++)
				{
					if (info->asics[i].frequency != 0)
					{
						has_freq = true;
						break;
					}
				}

				cgtime(&last_plateau_check);
				for (i = 0; i < info->chips; i++)
				{
					struct ASIC_INFO *asic = &info->asics[i];

					// missing nonces
					if (info->asic_type == BM1397)
					{
						if (has_freq && i == 0 && info->nonce_limit > 0.0
						&&  ms_tdiff(&now, &info->last_nonce) > info->nonce_limit)
						{
							plateau_type = PT_NONONCE;
							applog(LOG_ERR, "%d: %s %d - plateau_type PT_NONONCE [%u] %d > %.2f (lock=%d)",
								compac->cgminer_id, compac->drv->name, compac->device_id, i,
								ms_tdiff(&now, &info->last_nonce), info->nonce_limit, info->lock_freq);
							if (info->lock_freq)
								info->lock_freq = false;
						}
					}
					else
					{
						if (has_freq && info->nonce_limit > 0.0
						&&  ms_tdiff(&now, &asic->last_nonce) > info->nonce_limit)
						{
							plateau_type = PT_NONONCE;
							applog(LOG_ERR, "%d: %s %d - plateau_type PT_NONONCE [%u] %d > %.2f (lock=%d)",
								compac->cgminer_id, compac->drv->name, compac->device_id, i,
								ms_tdiff(&now, &asic->last_nonce), info->nonce_limit, info->lock_freq);
							if (info->lock_freq)
								info->lock_freq = false;
						}
					}

					// asic check-in failed
					if (!info->lock_freq
					&&  info->asic_type != BM1397)
					{
						if (ms_tdiff(&asic->last_frequency_ping, &asic->last_frequency_reply) > MS_SECOND_30
						&&  ms_tdiff(&now, &asic->last_frequency_reply) > MS_SECOND_30)
						{
							plateau_type = PT_FREQNR;
							applog(LOG_INFO, "%d: %s %d - plateau_type PT_FREQNR [%u] %d > %d",
								compac->cgminer_id, compac->drv->name, compac->device_id, i,
								ms_tdiff(&now, &asic->last_frequency_reply), MS_SECOND_30);
						}
					}

					// set frequency requests not honored
					if (!info->lock_freq
					&&  asic->frequency_attempt > 3)
					{
						plateau_type = PT_FREQSET;
						applog(LOG_INFO, "%d: %s %d - plateau_type PT_FREQSET [%u] %u > 3",
							compac->cgminer_id, compac->drv->name, compac->device_id, i, asic->frequency_attempt);
					}

					if (plateau_type)
					{
						float old_frequency, new_frequency;
						new_frequency = info->frequency_requested;
						bool didmsg, doreset;
						char *reason;

						char freq_buf[512];
						char freq_chip_buf[15];

						memset(freq_buf, 0, sizeof(freq_buf));
						memset(freq_chip_buf, 0, sizeof(freq_chip_buf));
						for (j = 0; j < info->chips; j++)
						{
							struct ASIC_INFO *asjc = &info->asics[j];
							sprintf(freq_chip_buf, "[%d:%.2f]", j, asjc->frequency);
							strcat(freq_buf, freq_chip_buf);
						}
						applog(LOG_INFO,"%d: %s %d - %s",
							compac->cgminer_id, compac->drv->name, compac->device_id, freq_buf);

						if (info->plateau_reset < 3)
						{
							// Capture failure high frequency using first three resets
							if ((info->frequency - info->freq_base) > info->frequency_fail_high)
								info->frequency_fail_high = (info->frequency - info->freq_base);

							applog(LOG_WARNING,"%d: %s %d - asic plateau: [%u] (%d/3) %.2fMHz",
								compac->cgminer_id, compac->drv->name, compac->device_id,
								i, info->plateau_reset + 1, info->frequency_fail_high);
						}

						if (info->plateau_reset >= 2) {
							if (ms_tdiff(&now, &info->last_frequency_adjust) > MS_MINUTE_30) {
								// Been running for 30 minutes, possible plateau
								// Overlook the incident
							} else {
								// Step back frequency
								info->frequency_fail_high -= info->freq_base;
							}
							new_frequency = limit_freq(info, FREQ_BASE(info->frequency_fail_high), true);
						}
						info->plateau_reset++;
						asic->last_state = asic->state;
						asic->state = ASIC_HALFDEAD;
						cgtime(&asic->state_change_time);
						cgtime(&info->monitor_time);

						switch (plateau_type)
						{
						 case PT_FREQNR:
							applog(info->log_wide,"%d: %s %d - no frequency reply from chip[%u] - %.2fMHz",
								compac->cgminer_id, compac->drv->name, compac->device_id, i, info->frequency);
							asic->frequency_attempt = 0;
							reason = " FREQNR";
							break;
						 case PT_FREQSET:
							applog(info->log_wide,"%d: %s %d - frequency set fail to chip[%u] - %.2fMHz",
								compac->cgminer_id, compac->drv->name, compac->device_id, i, info->frequency);
							asic->frequency_attempt = 0;
							reason = " FREQSET";
							break;
						 case PT_NONONCE:
							applog(info->log_wide,"%d: %s %d - missing nonces from chip[%u] - %.2fMHz",
								compac->cgminer_id, compac->drv->name, compac->device_id, i, info->frequency);
							reason = " NONONCE";
							break;
						 default:
							reason = NULL;
							break;
						}

						if (plateau_type == PT_NONONCE || info->asic_type == BM1387 || info->asic_type == BM1397)
						{
							// BM1384 is less tolerant to sudden drops in frequency.
							// Ignore other indicators except no nonce.
							doreset = true;
						}
						else
							doreset = false;

						didmsg = false;
						old_frequency = info->frequency_requested;
						if (new_frequency != old_frequency)
						{
							info->frequency_requested = new_frequency;
							applog(LOG_WARNING,"%d: %s %d - plateau%s [%u] adjust:%s target frequency %.2fMHz -> %.2fMHz",
								compac->cgminer_id, compac->drv->name, compac->device_id,
								reason ? : "", i, doreset ? " RESET" : "", old_frequency, new_frequency);
							didmsg = true;
						}

						if (doreset)
						{
							if (didmsg == false)
							{
								applog(LOG_WARNING,"%d: %s %d - plateau%s [%u] RESET at %.2fMHz",
									compac->cgminer_id, compac->drv->name, compac->device_id,
									reason ? : "", i, old_frequency);
							}
							info->mining_state = MINER_RESET;
						}
						// any plateau on 1 chip means no need to check the rest
						break;
					}
				}
			}

			if (!info->lock_freq
			&&  ms_tdiff(&now, &info->last_reset) < info->ramp_time)
			{
				// move running frequency towards target every second
				//  initially or for up to ramp_time after a reset
				if ((plateau_type == 0)
				&&  (ms_tdiff(&now, &last_movement) > MS_SECOND_1)
				&&  (info->frequency < info->frequency_requested)
				&&  (info->gh.noncesum > GHNONCENEEDED))
				{
					float new_frequency = info->frequency + info->step_freq;

					if (new_frequency > info->frequency_requested)
						new_frequency = info->frequency_requested;

					frequency_updated = 1;
					change_freq_any(compac, new_frequency);
					cgtime(&last_movement);
				}
			}
			else
			{
				// after ramp_time regularly check the frequency vs hashrate

				// when we have enough nonces or gh is full
				if (!info->lock_freq
				&&  (info->gh.last == (GHNUM-1) || info->gh.noncesum > GHNONCES)
				&&  (ms_tdiff(&now, &info->tune_limit) >= MS_MINUTE_2))
				{
					float new_freq, prev_freq;
					double hash_for_freq, curr_hr;
					int nonces, last;

					prev_freq = info->frequency;

					mutex_lock(&info->ghlock);
					curr_hr = gekko_gh_hashrate(info, &now, true);
					nonces = info->gh.noncesum;
					last = info->gh.last;
					mutex_unlock(&info->ghlock);

					// verify with locked values
					if ((last == (GHNUM-1)) || (nonces > GHNONCES))
					{
						hash_for_freq = info->frequency * (double)(info->cores * info->chips);
						// a low hash rate means frequency may be too high
						if (curr_hr < (hash_for_freq * info->ghrequire))
						{
							new_freq = FREQ_BASE(info->frequency - (info->freq_base * 2));
							if (new_freq < info->min_freq)
								new_freq = FREQ_BASE(info->min_freq);
							if (info->frequency_requested > new_freq)
								info->frequency_requested = new_freq;
							if (info->frequency_start > new_freq)
								info->frequency_start = new_freq;

							applog(LOG_WARNING,"%d: %s %d - %.2fGH/s low [%.2f/%.2f/%d] reset limit %.2fMHz -> %.2fMHz",
								compac->cgminer_id, compac->drv->name, compac->device_id,
								curr_hr/1.0e3, hash_for_freq/1.0e3, hash_for_freq*info->ghrequire/1.0e3,
								nonces, prev_freq, new_freq);

							mutex_lock(&info->lock);
							frequency_updated = 1;
							change_freq_any(compac, info->frequency_start);

							// reset from start
							info->mining_state = MINER_RESET;
							mutex_unlock(&info->lock);

							continue;
						}

						cgtime(&info->tune_limit);

						// step the freq up one - environment may have changed to allow it
						if (opt_gekko_tune2 != 0
						&&  (info->frequency_requested < info->frequency_selected)
						&&  (tdiff(&now, &info->last_reset) >= ((double)opt_gekko_tune2 * 60.0))
						&&  (tdiff(&now, &info->last_tune_up) >= ((double)opt_gekko_tune2 * 60.0)))
						{
							new_freq = FREQ_BASE(info->frequency + info->freq_base);
							if (new_freq <= info->frequency_selected)
							{
								if (info->frequency_requested < new_freq)
									info->frequency_requested = new_freq;

								applog(LOG_WARNING,"%d: %s %d - tune up attempt (%.1fGH/s) %.2fMHz -> %.2fMHz",
									compac->cgminer_id, compac->drv->name, compac->device_id,
									curr_hr/1.0e3, prev_freq, new_freq);

								// will reset if it doesn't improve,
								//  as soon as it has enough nonces
								frequency_updated = 1;
								change_freq_any(compac, new_freq);
							}
							cgtime(&info->last_tune_up);
						}
					}
				}
			}

			if (!info->lock_freq
			&&  !frequency_updated
			&&  ms_tdiff(&now, &last_frequency_check) > 20)
			{
				cgtime(&last_frequency_check);
				for (i = 0; i < info->chips; i++)
				{
					struct ASIC_INFO *asic = &info->asics[i];
					if (asic->frequency_updated)
					{
						asic->frequency_updated = 0;
						info->frequency_of = i;
						if (info->asic_type != BM1397)
							ping_freq(compac, i);
						break;
					}
				}
			}
		}

		has_freq = false;
		for (i = 0; i < info->chips; i++)
		{
			if (info->asics[i].frequency != 0)
			{
				has_freq = true;
				break;
			}
		}

		// don't bother with work if it's not mining
		if (!has_freq)
		{
			gekko_usleep(info, MS2US(10));
			continue;
		}

		struct timeval stt, fin;
		double wd;

		// don't delay work updates when doing busy work
		if (info->work_usec_num > 1 && !last_was_busy)
		{
			// check if we got here early
			// and sleep almost the entire delay required
			cgtime(&now);
			diff_us = us_tdiff(&now, &info->last_task);
			left_us = info->max_task_wait - diff_us;
			if (left_us > 0)
			{
				// allow time for get_queued() + a bit
				left_us -= (info->work_usec_avg + USLEEPPLUS);
				if (left_us >= USLEEPMIN)
					gekko_usleep(info, left_us);
			}
			else
			{
#if TUNE_CODE
				// ran over by 10us or more
				if (left_us <= -10)
				{
					info->over1num++;
					info->over1amt -= left_us;
				}
#endif
			}
		}

		cgtime(&stt);
		work = get_queued(compac);

		if (work)
		{
			cgtime(&fin);
			wd = us_tdiff(&fin, &stt);
			if (info->work_usec_num == 0)
				info->work_usec_avg = wd;
			else
			{
				// fast work times should have a higher effect
				if (wd < (info->work_usec_avg / 2.0))
					info->work_usec_avg = (info->work_usec_avg + wd) / 2.0;
				else
				{
					// ignore extra long work times after we get a few
					if (info->work_usec_num > 5
					&&  (wd / 3.0) < info->work_usec_avg)
					{
						info->work_usec_avg =
							(info->work_usec_avg * 9.0 + wd) / 10.0;
					}
				}
			}

			info->work_usec_num++;

			if (last_was_busy)
				last_was_busy = false;

#if TUNE_CODE
			diff_us = us_tdiff(&fin, &info->last_task);
			// stats if we got here too fast ...
			left_us = info->max_task_wait - diff_us;
			if (left_us < 0)
			{
				// ran over by 10us or more
				if (left_us <= -10)
				{
					info->over2num++;
					info->over2amt -= left_us;
				}
			}
#endif

			if (opt_gekko_noboost)
				work->pool->vmask = 0;

			info->job_id += info->add_job_id;
			if (info->job_id > info->max_job_id)
				info->job_id = info->min_job_id;
			old_work = info->work[info->job_id];
			info->work[info->job_id] = work;
			info->active_work[info->job_id] = 1;
			info->vmask = work->pool->vmask;
			if (info->asic_type == BM1387 || info->asic_type == BM1397)
			{
				if (!opt_gekko_noboost && info->vmask)
					info->task_len = 54 + 32 * (info->midstates - 1);
				else
					info->task_len = 54;
			}
			init_task(info);
		}
		else
		{
			struct pool *cp;
			cp = current_pool();
			if (!cp->stratum_active)
				cgtime(&info->last_pool_lost);

			if (cp->stratum_active
			&&  (info->asic_type == BM1387 || info->asic_type == BM1397))
			{
				// get Dups instead of sending busy work

				// sleep 1ms then fast loop back
				gekko_usleep(info, MS2US(1));
				last_was_busy = true;
				continue;
			}

			busy_work(info);
			info->busy_work++;
			last_was_busy = true;

			cgtime(&info->monitor_time);
			applog(LOG_INFO, "%d: %s %d - Busy",
				compac->cgminer_id, compac->drv->name, compac->device_id);
		}

		int task_len = info->task_len;
#if 0
		unsigned char jid = info->task[2];
#endif

		if (info->asic_type == BM1397)
		{ 
			int k;
			for (k = (info->task_len - 1); k >= 0; k--)
			{
				info->task[k+2] = info->task[k];
			}
			info->task[0] = 0x55;
			info->task[1] = 0xaa;
			task_len += 2;
		}

#if 0
applog(LOG_ERR, "%s() %d: %s %d - Task [%02x] len %3u", __func__,
	compac->cgminer_id, compac->drv->name, compac->device_id, jid, task_len);
applog(LOG_ERR, " [%02x %02x %02x %02x %02x %02x %02x %02x]",
	info->task[0], info->task[1], info->task[2], info->task[3], info->task[4], info->task[5], info->task[6], info->task[7]);
applog(LOG_ERR, " [%02x %02x %02x %02x %02x %02x %02x %02x]",
	info->task[8], info->task[9], info->task[10], info->task[11], info->task[12], info->task[13], info->task[14], info->task[15]);
applog(LOG_ERR, " [%02x %02x %02x %02x %02x %02x %02x %02x]",
	info->task[16], info->task[17], info->task[18], info->task[19], info->task[20], info->task[21], info->task[22], info->task[23]);
applog(LOG_ERR, " [%02x %02x %02x %02x %02x %02x %02x %02x]",
	info->task[24], info->task[25], info->task[26], info->task[27], info->task[28], info->task[29], info->task[30], info->task[31]);
#endif

		cgtime(&now); // set the time we actually sent it

		err = usb_write(compac, (char *)info->task, task_len, &sent_bytes, C_SENDWORK);
		//dumpbuffer(compac, LOG_WARNING, "TASK.TX", info->task, task_len);
		if (err != LIBUSB_SUCCESS)
		{
			applog(LOG_WARNING,"%d: %s %d - usb failure (%d)", compac->cgminer_id, compac->drv->name, compac->device_id, err);
			info->mining_state = MINER_RESET;
			continue;
		}
		if (sent_bytes != task_len)
		{
			if (ms_tdiff(&now, &info->last_write_error) > (5 * 1000)) {
				applog(LOG_WARNING,"%d: %s %d - usb write error [%d:%d]",
					compac->cgminer_id, compac->drv->name, compac->device_id, sent_bytes, task_len);
				cgtime(&info->last_write_error);
			}
			job_added = false;
		}
		else
		{
			// successfully sent work
			add_gekko_job(info, &now, false);
			job_added = true;
		}

		//let the usb frame propagate
		if (info->asic_type == BM1397 && info->usb_prop != 1000)
			gekko_usleep(info, info->usb_prop);
		else
			gekko_usleep(info, MS2US(1));

		info->task_ms = (info->task_ms * 9 + ms_tdiff(&now, &info->last_task)) / 10;

		info->last_task.tv_sec = now.tv_sec;
		info->last_task.tv_usec = now.tv_usec;
		if (info->first_task.tv_sec == 0L)
		{
			info->first_task.tv_sec = now.tv_sec;
			info->first_task.tv_usec = now.tv_usec;
		}
		info->tasks++;

		// work source changes can affect this e.g. 1 vs 4 midstates
		if (work && job_added
		&&  ms_tdiff(&now, &info->last_update_rates) > MS_SECOND_5)
		{
			compac_update_rates(compac);
		}
	}
	return NULL;
}

// not called by BM1397
static void *compac_handle_rx(void *object, int read_bytes, int path)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;
	struct ASIC_INFO *asic;
	int cmd_resp;
	unsigned int i;
	struct timeval now;

	cgtime(&now);

	cmd_resp = 0;
	if (info->rx[read_bytes-1] <= 0x1f)
	{
		if (bmcrc(info->rx, 8 * read_bytes - 5) == info->rx[read_bytes-1])
			cmd_resp = 1;
	}

	int log_level = (cmd_resp) ? LOG_INFO : LOG_INFO;
	if (path) {
		dumpbuffer(compac, log_level, "RX1", info->rx, read_bytes);
	} else {
		dumpbuffer(compac, log_level, "RX0", info->rx, read_bytes);
	}

	if (cmd_resp && info->rx[0] == 0x80 && info->frequency_of != (int)(info->chips)) {
		float frequency = 0.0;
		int frequency_of = info->frequency_of;
		info->frequency_of = info->chips;
		cgtime(&info->last_frequency_report);

		if (info->asic_type == BM1387 && (info->rx[2] == 0 || (info->rx[3] >> 4) == 0 || (info->rx[3] & 0x0f) != 1 || (info->rx[4]) != 0 || (info->rx[5]) != 0)) {
			cgtime(&info->last_frequency_invalid);
			applog(LOG_INFO,"%d: %s %d - invalid frequency report", compac->cgminer_id, compac->drv->name, compac->device_id);
		} else {
			if (info->asic_type == BM1387) {
				frequency = info->freq_mult * info->rx[1] / (info->rx[2] * (info->rx[3] >> 4) * (info->rx[3] & 0x0f));
			} else if (info->asic_type == BM1384) {
				frequency = (info->rx[1] + 1) * info->freq_base / ((1 + info->rx[2]) & 0x0f) * pow(2, (3 - info->rx[3])) + ((info->rx[2] >> 4) * info->freq_base);
			}

			if (frequency_of != (int)(info->chips)) {
				asic = &info->asics[frequency_of];
				cgtime(&asic->last_frequency_reply);
				if (frequency != asic->frequency) {
					bool syncd = 1;
					if (frequency < asic->frequency && frequency != info->frequency_requested) {
						applog(LOG_INFO,"%d: %s %d - chip[%d] reported frequency at %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_of, frequency);
					} else {
						applog(LOG_INFO,"%d: %s %d - chip[%d] reported new frequency of %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_of, frequency);
					}
					asic->frequency = frequency;
					if (asic->frequency == asic->frequency_set) {
						asic->frequency_attempt = 0;
					}
					for (i = 1; i < info->chips; i++) {
						if (info->asics[i].frequency != info->asics[i - 1].frequency) {
							syncd = 0;
						}
					}
					if (info->frequency_syncd != syncd) {
						info->frequency_syncd = syncd;
						applog(LOG_INFO,"%d: %s %d - syncd [%d]", compac->cgminer_id, compac->drv->name, compac->device_id, syncd);
					}
				} else {
					applog(LOG_INFO,"%d: %s %d - chip[%d] reported frequency of %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, info->frequency_of, frequency);
				}
			} else {
				//applog(LOG_INFO,"%d: %s %d - [-1] reported frequency of %.2fMHz", compac->cgminer_id, compac->drv->name, compac->device_id, frequency);
				//if (frequency != info->frequency) {
				//	info->frequency = frequency;
				//}
			}
			compac_update_rates(compac);
		}
	}

	switch (info->mining_state) {
		case MINER_CHIP_COUNT:
		case MINER_CHIP_COUNT_XX:
			if (cmd_resp && (info->rx[0] == 0x13 ||
			    (info->rx[0] == 0xaa && info->rx[1] == 0x55 && info->rx[2] == 0x13))) { // BM1397
				struct ASIC_INFO *asic = &info->asics[info->chips];
				memset(asic, 0, sizeof(struct ASIC_INFO));
				asic->frequency = info->frequency_default;
				asic->frequency_attempt = 0;
				asic->last_frequency_ping = (struct timeval){0};
				asic->last_frequency_reply = (struct timeval){0};
				cgtime(&asic->last_nonce);
				info->chips++;
				info->mining_state = MINER_CHIP_COUNT_XX;
				compac_update_rates(compac);
			}
			break;
		case MINER_OPEN_CORE:
			if ((info->rx[0] == 0x72 && info->rx[1] == 0x03 && info->rx[2] == 0xEA && info->rx[3] == 0x83) ||
				(info->rx[0] == 0xE1 && info->rx[1] == 0x6B && info->rx[2] == 0xF8 && info->rx[3] == 0x09)) {
				//open core nonces = healthy chips.
				info->zero_check++;
			}
			break;
		case MINER_MINING:
			if (!cmd_resp) {
#ifdef __APPLE__
				if (opt_mac_yield)
					sched_yield();
#else
				selective_yield();
#endif
				mutex_lock(&info->lock);
				info->hashes += compac_check_nonce(compac);
				mutex_unlock(&info->lock);
			}
			break;
		default:
			break;
	}
	return NULL;
}

static void *compac_gsf_nonce_que(void *object)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;
	struct timespec abstime, addtime;
	K_ITEM *item;
	int rc;

	if (info->asic_type != BM1397)
		return NULL;

	// wait at most 42ms for a nonce
	ms_to_timespec(&addtime, 42);

	while (info->mining_state != MINER_SHUTDOWN)
	{
		K_WLOCK(info->nlist);
		item = k_unlink_head(info->nstore);
		K_WUNLOCK(info->nlist);

		if (item)
		{
			compac_gsf_nonce(compac, item);
			K_WLOCK(info->nlist);
			k_add_head(info->nlist, item);
			K_WUNLOCK(info->nlist);
		}
		else
		{
			cgcond_time(&abstime);
			timeraddspec(&abstime, &addtime);
			mutex_lock(&info->nlock);
			rc = pthread_cond_timedwait(&info->ncond, &info->nlock, &abstime);
			mutex_unlock(&info->nlock);
			if (rc == ETIMEDOUT)
				info->ntimeout++;
			else
				info->ntrigger++;
		}
	}
	return NULL;
}

static bool gsf_reply(struct COMPAC_INFO *info, unsigned char *rx, int len, struct timeval *now)
{
	unsigned char fa, fb, fc1, fc2;
	bool used = false;

	if (len == (int)(info->rx_len) && rx[7] == BM1397FREQ)
	{
		int chip = TOCHIPPY1397(info, rx[6]);
		if (chip >= 0 && chip < (int)(info->chips))
		{
			struct ASIC_INFO *asic = &info->asics[chip];

			fa = rx[3];
			fb = rx[4];
			fc1 = (rx[5] & 0xf0) >> 4;
			fc2 = rx[5] & 0x0f;

			// only allow a valid reply
			if (fa >= 0 && fb > 0 && fc1 > 0 && fc2 > 0)
			{
				asic->frequency_reply = info->freq_mult * fa / fb / fc1 / fc2;
				asic->last_frequency_reply.tv_sec = now->tv_sec;
				asic->last_frequency_reply.tv_usec = now->tv_usec;
				used = true;
			}
		}
	}

	return used;
}

static void *compac_listen2(struct cgpu_info *compac, struct COMPAC_INFO *info)
{
	unsigned char rx[BUFFER_MAX];
	struct timeval now;
	int read_bytes, tmo, pos = 0, len, i, prelen;
	bool okcrc, used, chipped;
	K_ITEM *item;

	memset(rx, 0, sizeof(rx));

	while (info->mining_state != MINER_SHUTDOWN)
	{
		tmo = 20;

		if (info->mining_state == MINER_CHIP_COUNT)
		{
			unsigned char chippy[] = {0x52, 0x05, 0x00, 0x00, 0x0A};
			compac_send2(compac, chippy, sizeof(chippy), 8 * sizeof(chippy) - 8, "CHIPPY");
			info->mining_state = MINER_CHIP_COUNT_XX;
			// initial config reply allow much longer
			tmo = 1000;
		}

		usb_read_timeout(compac, ((char *)rx)+pos, BUFFER_MAX-pos, &read_bytes, tmo, C_GETRESULTS);
		pos += read_bytes;

		cgtime(&now);

		// all replies should be info->rx_len
		while (read_bytes > 0 && pos >= (int)(info->rx_len))
		{
#if 0
applog(LOG_ERR, "%d: %s %d - READ %3d pos %3d state %2d first 16: [%02x %02x %02x %02x %02x %02x %02x %02x]",
	compac->cgminer_id, compac->drv->name, compac->device_id, read_bytes, pos, info->mining_state,
	rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
applog(LOG_ERR, " [%02x %02x %02x %02x %02x %02x %02x %02x]",
	rx[8], rx[9], rx[10], rx[11], rx[12], rx[13], rx[14], rx[15]);
#endif

			// rubbish - skip over it to next 0xaa
			if (rx[0] != 0xaa || rx[1] != 0x55)
			{
				for (i = 1; i < pos; i++)
				{
					if (rx[i] == 0xaa)
					{
						// next read could be 0x55 or i+1=0x55
						if (i == (pos - 1) || rx[i+1] == 0x55)
							break;
					}
				}
				// no 0xaa dump it and wait for more data
				if (i >= pos)
				{
#if 0
applog(LOG_ERR, " %s %d no 0xaa = dump all (%d) [%02x %02x %02x %02x ...]",
	compac->drv->name, compac->device_id, pos, rx[0], rx[1], rx[2], rx[3]);
#endif
					pos = 0;
					continue;
				}
#if 0
applog(LOG_ERR, " %s %d dump before %d=0xaa [%02x %02x %02x %02x ...]",
	compac->drv->name, compac->device_id, i, rx[0], rx[1], rx[2], rx[3]);
#endif

				// i=0xaa dump up to i-1
				memmove(rx, rx+i, pos-i);
				pos -= i;

				if (pos < (int)(info->rx_len))
					continue;
			}

			// find next 0xaa 0x55
			for (len = info->rx_len; len < pos; len++)
			{
				if (rx[len] == 0xaa
				&&  (len == (pos-1) || rx[len+1] == 0x55))
					break;
			}

			prelen = len;
			// a reply followed by only 0xaa but no 0x55 yet
			if (len == pos && (len == 8 || len == 10) && rx[pos-1] == 0xaa)
				len--;

			// try it as a nonce
			if (len != (int)(info->rx_len))
				len = info->rx_len;

#if 0
			if (info->asic_type == BM1397 &&
			    bmcrc(&rx[i+2], 8 * (info->rx_len-2) - 5) == (rx[i + info->rx_len - 1] & 0x1f)) {
				// crc checksum is good
				crc_match = true;
				rx_okay = true;
			}
			if (info->asic_type == BM1397 && rx[0] >= 0xaa && rx[1] <= 0x55) {
				// bm1397 response
				rx_okay = true;
			}
#endif

			if (rx[len-1] <= 0x1f
			&&  bmcrc(rx+2, 8 * (len-2) - 5) == rx[len-1])
				okcrc = true;
			else
				okcrc = false;

			switch (info->mining_state)
			{
			 case MINER_CHIP_COUNT:
			 case MINER_CHIP_COUNT_XX:
				// BM1397
				chipped = false;
				if (rx[2] == 0x13 && rx[3] == 0x97)
				{
					struct ASIC_INFO *asic = &info->asics[info->chips];
					memset(asic, 0, sizeof(struct ASIC_INFO));
					asic->frequency = info->frequency_default;
					asic->frequency_attempt = 0;
					asic->last_frequency_ping = (struct timeval){0};
					asic->frequency_reply = -1;
					asic->last_frequency_reply = (struct timeval){0};
					cgtime(&asic->last_nonce);
					info->chips++;
					info->mining_state = MINER_CHIP_COUNT_XX;
					compac_update_rates(compac);
					chipped = true;
				}
				// ignore all data until we get at least 1 chip reply
			 	if (!chipped && info->mining_state == MINER_CHIP_COUNT_XX)
				{
					// we found some chips then it replied with other data ...
					if (info->chips > 0)
					{
						info->mining_state = MINER_CHIP_COUNT_OK;
						mutex_lock(&static_lock);
						(*init_count) = 0;
						info->init_count = 0;
						mutex_unlock(&static_lock);

						// don't discard the data
						if (len == (int)(info->rx_len) && okcrc)
							gsf_reply(info, rx, info->rx_len, &now);
					}
					else
						info->mining_state = MINER_RESET;
				}
				break;
			 case MINER_MINING:
				used = false;
				if (len == (int)(info->rx_len) && okcrc)
				{
					used = gsf_reply(info, rx, info->rx_len, &now);
#if 0
if (!used)
{
applog(LOG_ERR, "%d: %s %d - ? len %3d state %2d first 12:",
	compac->cgminer_id, compac->drv->name, compac->device_id, len, info->mining_state);
applog(LOG_ERR, " [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]",
	rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8], rx[9], rx[10], rx[11]);
}
#endif
				}

				// also try unidentifed crc's as a nonce
				if (!used)
				{
					K_WLOCK(info->nlist);
					item = k_unlink_head(info->nlist);
					K_WUNLOCK(info->nlist);
					DATA_NONCE(item)->asic = 0;
					// should never be true ...
					if (len > (int)sizeof(DATA_NONCE(item)->rx))
						len = (int)sizeof(DATA_NONCE(item)->rx);
					memcpy(DATA_NONCE(item)->rx, rx, len);
					DATA_NONCE(item)->len = len;
					DATA_NONCE(item)->prelen = prelen;
					DATA_NONCE(item)->when.tv_sec = now.tv_sec;
					DATA_NONCE(item)->when.tv_usec = now.tv_usec;
					K_WLOCK(info->nlist);
					k_add_tail(info->nstore, item);
					K_WUNLOCK(info->nlist);
					mutex_lock(&info->nlock);
					pthread_cond_signal(&info->ncond);
					mutex_unlock(&info->nlock);
				}
				break;
			 default:
				used = false;
				if (len == (int)(info->rx_len) && okcrc)
					used = gsf_reply(info, rx, info->rx_len, &now);
#if 0
if (!used)
{
applog(LOG_ERR, "%d: %s %d - unhandled, len %3d state %2d first 12:",
	compac->cgminer_id, compac->drv->name, compac->device_id, len, info->mining_state);
applog(LOG_ERR, " [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]",
	rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8], rx[9], rx[10], rx[11]);
}
#endif
				break;
			}
			// we've used up 0..len-1
			if (pos > len)
				memmove(rx, rx+len, pos-len);
			pos -= len;
		}

		if (read_bytes == 0 || pos < 6)
		{
			if (info->mining_state == MINER_CHIP_COUNT_XX)
			{
				if (info->chips < info->expected_chips)
					info->mining_state = MINER_RESET;
				else
				{
					if (info->chips > 0)
					{
						info->mining_state = MINER_CHIP_COUNT_OK;
						mutex_lock(&static_lock);
						(*init_count) = 0;
						info->init_count = 0;
						mutex_unlock(&static_lock);
					}
					else
						info->mining_state = MINER_RESET;
				}
			}
		}
	}
	return NULL;
}

static void *compac_listen(void *object)
{
	struct cgpu_info *compac = (struct cgpu_info *)object;
	struct COMPAC_INFO *info = compac->device_data;

	if (info->asic_type == BM1397)
		return compac_listen2(compac, info);

	struct timeval now;
	unsigned char rx[BUFFER_MAX];
	unsigned char *prx = rx;
	int read_bytes, cmd_resp, pos;
	unsigned int i, rx_bytes;

	memset(rx, 0, sizeof(rx));
	memset(info->rx, 0, sizeof(info->rx));

	pos = 0;
	rx_bytes = 0;

	while (info->mining_state != MINER_SHUTDOWN) {

		cgtime(&now);

		if (info->mining_state == MINER_CHIP_COUNT) {
			if (info->asic_type == BM1387) {
				unsigned char buffer[] = {0x54, 0x05, 0x00, 0x00, 0x00};
				compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
			} else if (info->asic_type == BM1384) {
				unsigned char buffer[] = {0x84, 0x00, 0x00, 0x00};
				compac_send(compac, buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
			}
			usb_read_timeout(compac, (char *)rx, BUFFER_MAX, &read_bytes, 1000, C_GETRESULTS);
			dumpbuffer(compac, LOG_INFO, "CMD.RX", rx, read_bytes);

			rx_bytes = (unsigned int)read_bytes;
			info->mining_state = MINER_CHIP_COUNT_XX;
		} else {

			if (rx_bytes >= (BUFFER_MAX - info->rx_len)) {
				applog(LOG_INFO, "%d: %s %d - Buffer not useful, dumping (b) %d bytes", compac->cgminer_id, compac->drv->name, compac->device_id, rx_bytes);
				dumpbuffer(compac, LOG_INFO, "NO-CRC-RX", rx, rx_bytes);
				pos = 0;
				rx_bytes = 0;
			}

			usb_read_timeout(compac, (char *)(&rx[pos]), info->rx_len, &read_bytes, 20, C_GETRESULTS);
			rx_bytes += read_bytes;
			pos = rx_bytes;
		}

		if (read_bytes > 0) {
			if (rx_bytes < info->rx_len) {
				applog(LOG_INFO, "%d: %s %d - Buffered %d bytes", compac->cgminer_id, compac->drv->name, compac->device_id, rx_bytes);
				dumpbuffer(compac, LOG_INFO, "Partial-RX", rx, rx_bytes);
				continue;
			}

			if (rx_bytes >= info->rx_len)
				cmd_resp = (rx[read_bytes - 1] <= 0x1f && bmcrc(prx, 8 * read_bytes - 5) == rx[read_bytes - 1]) ? 1 : 0;

			if (info->mining_state == MINER_CHIP_COUNT_XX || cmd_resp) {
				if (rx_bytes % info->rx_len == 2) {
					// fix up trial 1
					int shift = 0;
					int extra = 0;
					for (i = 0; i < (rx_bytes - shift); i++) {
						if (rx[i] == 0x01) {
							shift = 2;
							extra = rx[i + 1];
						}
						rx[i] = rx[i + shift];
					}
					rx_bytes -= shift;
					pos = rx_bytes;
					applog(LOG_INFO, "%d: %s %d - Extra Data - 0x01 0x%02x", compac->cgminer_id, compac->drv->name, compac->device_id, extra & 0xff);
				}
			}

			if (rx_bytes >= info->rx_len) {
				bool crc_match = false;
				unsigned int new_pos = 0;
				for (i = 0; i <= (rx_bytes - info->rx_len); i++) {
					bool rx_okay = false;

					if (bmcrc(&rx[i], 8 * info->rx_len - 5) == (rx[i + info->rx_len - 1] & 0x1f)) {
						// crc checksum is good
						crc_match = true;
						rx_okay = true;
					}
					if (info->asic_type == BM1384 && rx[i + info->rx_len - 1] >= 0x80 && rx[i + info->rx_len - 1] <= 0x9f) {
						// bm1384 nonce
						rx_okay = true;
					}
					if (rx_okay) {
						memcpy(info->rx, &rx[i], info->rx_len);
						compac_handle_rx(compac, info->rx_len, 0);
						new_pos = i + info->rx_len;
					}
				}
				if (new_pos > rx_bytes) {
					rx_bytes = 0;
					pos = 0;
				} else if (new_pos > 0) {
					for (i = new_pos; i < rx_bytes; i++) {
						rx[i - new_pos] = rx[i];
					}
					rx_bytes -= new_pos;
					pos = rx_bytes;
				}
			}

/*
			if (bmcrc(&rx[rx_bytes - info->rx_len], 8 * info->rx_len - 5) != info->rx[rx_bytes - 1]) {
				int n_read_bytes;
				pos = rx_bytes;
				usb_read_timeout(compac, &rx[pos], BUFFER_MAX - pos, &n_read_bytes, 5, C_GETRESULTS);
				rx_bytes += n_read_bytes;

				if (rx_bytes % info->rx_len != 0 && rx_bytes >= info->rx_len) {
					int extra_bytes = rx_bytes % info->rx_len;
					for (i = extra_bytes; i < rx_bytes; i++) {
						rx[i - extra_bytes] = rx[i];
					}
					rx_bytes -= extra_bytes;
					applog(LOG_INFO, "%d: %s %d - Fixing buffer alignment, dumping initial %d bytes", compac->cgminer_id, compac->drv->name, compac->device_id, extra_bytes);
				}
			}
			if (rx_bytes % info->rx_len == 0) {
				for (i = 0; i < rx_bytes; i += info->rx_len) {
					memcpy(info->rx, &rx[i], info->rx_len);
					compac_handle_rx(compac, info->rx_len, 1);
				}
				pos = 0;
				rx_bytes = 0;
			}
*/
		} else {

			if (rx_bytes > 0) {
				applog(LOG_INFO, "%d: %s %d - Second read, no data dumping (c) %d bytes", compac->cgminer_id, compac->drv->name, compac->device_id, rx_bytes);
				dumpbuffer(compac, LOG_INFO, "EXTRA-RX", rx, rx_bytes);
			}

			pos = 0;
			rx_bytes = 0;

			// RX line is idle, let's squeeze in a command to the micro if needed.
			if (info->asic_type == BM1387) {
				if (ms_tdiff(&now, &info->last_micro_ping) > MS_SECOND_5 && ms_tdiff(&now, &info->last_task) > 1 && ms_tdiff(&now, &info->last_task) < 3) {
					compac_micro_send(compac, M1_GET_TEMP, 0x00, 0x00);
					cgtime(&info->last_micro_ping);
				}
			}

			switch (info->mining_state) {
				case MINER_CHIP_COUNT_XX:
					if (info->chips < info->expected_chips) {
						applog(LOG_INFO, "%d: %s %d - found %d/%d chip(s)", compac->cgminer_id, compac->drv->name, compac->device_id, info->chips, info->expected_chips);
						info->mining_state = MINER_RESET;
					} else {
						applog(LOG_INFO, "%d: %s %d - found %d chip(s)", compac->cgminer_id, compac->drv->name, compac->device_id, info->chips);
						if (info->chips > 0) {
							info->mining_state = MINER_CHIP_COUNT_OK;
							mutex_lock(&static_lock);
							(*init_count) = 0;
							info->init_count = 0;
							mutex_unlock(&static_lock);
						} else {
							info->mining_state = MINER_RESET;
						}
					}
					break;
				default:
					break;
			}
		}
	}
	return NULL;
}


static bool compac_init(struct thr_info *thr)
{
	int i;
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	float step_freq;

	// all are currently this value
	info->freq_mult = 25.0;
	// most are this value (6.25)
	info->freq_base = info->freq_mult / 4.0;

	// correct some options
	if (opt_gekko_tune_down > 100)
		opt_gekko_tune_down = 100;
	if (opt_gekko_tune_up > 99)
		opt_gekko_tune_up = 99;
	opt_gekko_wait_factor = freqbounding(opt_gekko_wait_factor, 0.01, 2.0);
	info->wait_factor0 = opt_gekko_wait_factor;
	if (opt_gekko_start_freq < 25)
		opt_gekko_start_freq = 25;
	if (opt_gekko_step_freq < 1)
		opt_gekko_step_freq = 1;
	if (opt_gekko_step_delay < 1)
		opt_gekko_step_delay = 1;

	// must limit it to allow tune downs before trying again
	if (opt_gekko_tune2 < 30 && opt_gekko_tune2 != 0)
		opt_gekko_tune2 = 30;

	info->boosted = false;
	info->prev_nonce = 0;
	info->fail_count = 0;
	info->busy_work = 0;
	info->log_wide = (opt_widescreen) ? LOG_WARNING : LOG_INFO;
	info->plateau_reset = 0;
	info->low_eff_resets = 0;
	info->frequency_fail_high = 0;
	info->frequency_fail_low = 999;
	info->frequency_fo = info->chips;

	info->first_task.tv_sec = 0L;
	info->first_task.tv_usec = 0L;
	info->tasks = 0;
	info->tune_limit.tv_sec = 0L;
	info->tune_limit.tv_usec = 0L;
	info->last_tune_up.tv_sec = 0L;
	info->last_tune_up.tv_usec = 0L;

	memset(info->rx, 0, sizeof(info->rx));
	memset(info->tx, 0, sizeof(info->tx));
	memset(info->cmd, 0, sizeof(info->cmd));
	memset(info->end, 0, sizeof(info->end));
	memset(info->task, 0, sizeof(info->task));

	for (i = 0; i < JOB_MAX; i++) {
		info->active_work[i] = false;
		info->work[i] = NULL;
	}

	memset(&(info->gh), 0, sizeof(info->gh));

	cgtime(&info->last_write_error);
	cgtime(&info->last_frequency_adjust);
	cgtime(&info->last_computed_increase);
	cgtime(&info->last_low_eff_reset);
	info->last_frequency_invalid = (struct timeval){0};
	cgtime(&info->last_micro_ping);
	cgtime(&info->last_scanhash);
	cgtime(&info->last_reset);
	cgtime(&info->last_task);
	cgtime(&info->start_time);
	cgtime(&info->monitor_time);

	info->step_freq = FREQ_BASE(opt_gekko_step_freq);

	// if it can't mine at this freq the hardware has failed
	info->min_freq = info->freq_mult;
	// most should only take this long
	info->ramp_time = MS_MINUTE_3;

	info->ghrequire = GHREQUIRE;
	info->freq_fail = 0.0;
	info->hr_scale = 1.0;
	info->usb_prop = 1000;

	switch (info->ident)
	{
	 case IDENT_BSC:
	 case IDENT_GSC:
		info->frequency_requested = limit_freq(info, opt_gekko_gsc_freq, false);
		info->frequency_start = limit_freq(info, opt_gekko_start_freq, false);
		break;
	 case IDENT_BSD:
	 case IDENT_GSD:
		info->frequency_requested = limit_freq(info, opt_gekko_gsd_freq, false);
		info->frequency_start = limit_freq(info, opt_gekko_start_freq, false);
		break;
	 case IDENT_BSE:
	 case IDENT_GSE:
		info->frequency_requested = limit_freq(info, opt_gekko_gse_freq, false);
		info->frequency_start = limit_freq(info, opt_gekko_start_freq, false);
		break;
	 case IDENT_GSH:
		info->frequency_requested = limit_freq(info, opt_gekko_gsh_freq, false);
		info->frequency_start = limit_freq(info, opt_gekko_start_freq, false);
		break;
	 case IDENT_GSI:
		info->frequency_requested = limit_freq(info, opt_gekko_gsi_freq, false);
		info->frequency_start = limit_freq(info, opt_gekko_start_freq, false);
		// default to 550
		if (info->frequency_start == 100)
			info->frequency_start = 550;
		if (info->frequency_start < 100)
			info->frequency_start = 100;
		// due to higher freq allow longer
		info->ramp_time = MS_MINUTE_4;
		break;
	 case IDENT_GSF:
	 case IDENT_GSFM:
		if (info->ident == IDENT_GSF)
			info->frequency_requested = limit_freq(info, opt_gekko_gsf_freq, false);
		else
			info->frequency_requested = limit_freq(info, opt_gekko_r909_freq, false);

		info->frequency_start = limit_freq(info, opt_gekko_start_freq, false);
		if (info->frequency_start < 100)
			info->frequency_start = 100;
		if (info->frequency_start == 100)
		{
			if (info->ident == IDENT_GSF)
			{
				// default to 200
				info->frequency_start = 200;
			}
			else // (info->ident == IDENT_GSFM)
			{
				// default to 400
				info->frequency_start = 400;
			}
		}
		// ensure request is >= start
		if (info->frequency_requested < info->frequency_start)
			info->frequency_requested = info->frequency_start;
		// correct the defaults:
		info->freq_base = info->freq_mult / 5.0;
		step_freq = opt_gekko_step_freq;
		if (step_freq == 6.25)
			step_freq = 5.0;
		info->step_freq = FREQ_BASE(step_freq);
		// IDENT_GSFM runs at the more reliable higher frequencies
		if (info->ident == IDENT_GSF)
		{
			// chips can get lower than the calculated 67.2 at lower freq
			info->hr_scale = 52.5 / 67.2;
		}
		// due to ticket mask allow longer
		info->ramp_time = MS_MINUTE_5;
		break;
	 default:
		info->frequency_requested = 200;
		info->frequency_start = info->frequency_requested;
		break;
	}
	if (info->frequency_start > info->frequency_requested) {
		info->frequency_start = info->frequency_requested;
	}
	info->frequency_requested = FREQ_BASE(info->frequency_requested);
	info->frequency_selected = info->frequency_requested;
	info->frequency_start = FREQ_BASE(info->frequency_start);

	info->frequency = info->frequency_start;
	info->frequency_default = info->frequency_start;

	if (!info->rthr.pth) {
		pthread_mutex_init(&info->lock, NULL);
		pthread_mutex_init(&info->wlock, NULL);
		pthread_mutex_init(&info->rlock, NULL);

		pthread_mutex_init(&info->ghlock, NULL);
		pthread_mutex_init(&info->joblock, NULL);

		if (info->ident == IDENT_GSF || info->ident == IDENT_GSFM)
		{
			info->nlist = k_new_list("GekkoNonces", sizeof(struct COMPAC_NONCE),
						ALLOC_NLIST_ITEMS, LIMIT_NLIST_ITEMS, true);
			info->nstore = k_new_store(info->nlist);
		}

		if (thr_info_create(&(info->rthr), NULL, compac_listen, (void *)compac)) {
			applog(LOG_ERR, "%d: %s %d - read thread create failed", compac->cgminer_id, compac->drv->name, compac->device_id);
			return false;
		} else {
			applog(LOG_INFO, "%d: %s %d - read thread created", compac->cgminer_id, compac->drv->name, compac->device_id);
		}
		pthread_detach(info->rthr.pth);

		gekko_usleep(info, MS2US(100));

		if (thr_info_create(&(info->wthr), NULL, compac_mine2, (void *)compac)) {
			applog(LOG_ERR, "%d: %s %d - write thread create failed", compac->cgminer_id, compac->drv->name, compac->device_id);
			return false;
		} else {
			applog(LOG_INFO, "%d: %s %d - write thread created", compac->cgminer_id, compac->drv->name, compac->device_id);
		}

		pthread_detach(info->wthr.pth);

		if (info->ident == IDENT_GSF || info->ident == IDENT_GSFM)
		{
			gekko_usleep(info, MS2US(10));

			if (pthread_mutex_init(&info->nlock, NULL))
			{
				applog(LOG_ERR, "%d: %s %d - nonce mutex create failed",
					compac->cgminer_id, compac->drv->name, compac->device_id);
				return false;
			}
			if (pthread_cond_init(&info->ncond, NULL))
			{
				applog(LOG_ERR, "%d: %s %d - nonce cond create failed",
					compac->cgminer_id, compac->drv->name, compac->device_id);
				return false;
			}
			if (thr_info_create(&(info->nthr), NULL, compac_gsf_nonce_que, (void *)compac))
			{
				applog(LOG_ERR, "%d: %s %d - nonce thread create failed",
					compac->cgminer_id, compac->drv->name, compac->device_id);
				return false;
			}
			else
			{
				applog(LOG_INFO, "%d: %s %d - nonce thread created",
					compac->cgminer_id, compac->drv->name, compac->device_id);
			}

			pthread_detach(info->nthr.pth);
		}
	}

	return true;
}

static int64_t compac_scanwork(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;

	struct timeval now;
	int read_bytes;
	// uint64_t hashes = 0;
	uint64_t xhashes = 0;

	if (info->chips == 0)
		gekko_usleep(info, MS2US(10));

	if (compac->usbinfo.nodev)
		return -1;

#ifdef __APPLE__
	if (opt_mac_yield)
		sched_yield();
#else
	selective_yield();
#endif
	cgtime(&now);

	switch (info->mining_state) {
		case MINER_INIT:
			gekko_usleep(info, MS2US(50));
			compac_flush_buffer(compac);
			info->chips = 0;
			info->ramping = 0;
			info->frequency_syncd = 1;
			if (info->frequency_start > info->frequency_requested) {
				info->frequency_start = info->frequency_requested;
			}
			info->mining_state = MINER_CHIP_COUNT;
			return 0;
			break;
		case MINER_CHIP_COUNT:
			if (ms_tdiff(&now, &info->last_reset) > MS_SECOND_5) {
				applog(LOG_INFO, "%d: %s %d - found 0 chip(s)", compac->cgminer_id, compac->drv->name, compac->device_id);
				info->mining_state = MINER_RESET;
				return 0;
			}
			gekko_usleep(info, MS2US(10));
			break;
		case MINER_CHIP_COUNT_OK:
			gekko_usleep(info, MS2US(50));
			//compac_set_frequency(compac, info->frequency_start);
			compac_send_chain_inactive(compac);

			if (info->asic_type == BM1397)
				info->mining_state = MINER_OPEN_CORE_OK;

			return 0;
			break;
		case MINER_OPEN_CORE:
			info->job_id = info->ramping % (info->max_job_id + 1);

			//info->task_hcn = (0xffffffff / info->chips) * (1 + info->ramping) / info->cores;
			init_task(info);
			dumpbuffer(compac, LOG_DEBUG, "RAMP", info->task, info->task_len);

			usb_write(compac, (char *)info->task, info->task_len, &read_bytes, C_SENDWORK);
			if (info->ramping > (info->cores * info->add_job_id)) {
				//info->job_id = 0;
				info->mining_state = MINER_OPEN_CORE_OK;
				info->task_hcn = (0xffffffff / info->chips);
				return 0;
			}

			info->ramping += info->add_job_id;
			info->task_ms = (info->task_ms * 9 + ms_tdiff(&now, &info->last_task)) / 10;
			cgtime(&info->last_task);
			gekko_usleep(info, MS2US(10));
			return 0;
			break;
		case MINER_OPEN_CORE_OK:
			applog(LOG_INFO, "%d: %s %d - start work", compac->cgminer_id, compac->drv->name, compac->device_id);
			if (info->asic_type == BM1397)
				gsf_calc_nb2c(compac);
			cgtime(&info->start_time);
			cgtime(&info->monitor_time);
			cgtime(&info->last_frequency_adjust);
			info->last_dup_time = (struct timeval){0};
			cgtime(&info->last_frequency_report);
			cgtime(&info->last_micro_ping);
			cgtime(&info->last_nonce);
			compac_flush_buffer(compac);
			compac_update_rates(compac);
			info->update_work = 1;
			info->mining_state = MINER_MINING;
			return 0;
			break;
		case MINER_MINING:
			break;
		case MINER_RESET:
			compac_flush_work(compac);
			if (info->asic_type == BM1387 || info->asic_type == BM1397) {
				compac_toggle_reset(compac);
			} else if (info->asic_type == BM1384) {
				compac_set_frequency(compac, info->frequency_default);
				//compac_send_chain_inactive(compac);
			}
			compac_prepare(thr);

			info->fail_count++;
			info->dupsreset = 0;
			info->mining_state = MINER_INIT;
			cgtime(&info->last_reset);
			// in case clock jumps back ...
			cgtime(&info->tune_limit);

			// wipe info->gh/asic->gc
			gh_offset(info, &now, true, false);
			// wipe info->job
			job_offset(info, &now, true, false);
			// reset P:
			info->frequency_computed = 0;
			return 0;
			break;
		case MINER_MINING_DUPS:
			info->mining_state = MINER_MINING;
			break;
		default:
			break;
	}

	mutex_lock(&info->lock);
	// hashes = info->hashes;
	xhashes = info->xhashes;
	info->hashes = 0;
	info->xhashes = 0;
	mutex_unlock(&info->lock);

	gekko_usleep(info, MS2US(1));
	return xhashes * 0xffffffffull;
	//return hashes;
}

static struct cgpu_info *compac_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *compac;
	struct COMPAC_INFO *info;
	int i;
	bool exclude_me = 0;
	uint32_t baudrate = CP210X_DATA_BAUD;
	unsigned int bits = CP210X_BITS_DATA_8 | CP210X_BITS_PARITY_MARK;

	compac = usb_alloc_cgpu(&bm1397_drv, 1);

	if (!usb_init(compac, dev, found))
	{
		applog(LOG_INFO, "failed usb_init");
		compac = usb_free_cgpu(compac);
		return NULL;
	}

	// all zero
	info = cgcalloc(1, sizeof(struct COMPAC_INFO));

#if TUNE_CODE
	pthread_mutex_init(&info->slock, NULL);
#endif

	compac->device_data = (void *)info;

	info->ident = usb_ident(compac);

	if (opt_gekko_gsc_detect || opt_gekko_gsd_detect || opt_gekko_gse_detect
	||  opt_gekko_gsh_detect || opt_gekko_gsi_detect || opt_gekko_gsf_detect
	||  opt_gekko_r909_detect)
	{
		exclude_me  = (info->ident == IDENT_BSC && !opt_gekko_gsc_detect);
		exclude_me |= (info->ident == IDENT_GSC && !opt_gekko_gsc_detect);
		exclude_me |= (info->ident == IDENT_BSD && !opt_gekko_gsd_detect);
		exclude_me |= (info->ident == IDENT_GSD && !opt_gekko_gsd_detect);
		exclude_me |= (info->ident == IDENT_BSE && !opt_gekko_gse_detect);
		exclude_me |= (info->ident == IDENT_GSE && !opt_gekko_gse_detect);
		exclude_me |= (info->ident == IDENT_GSH && !opt_gekko_gsh_detect);
		exclude_me |= (info->ident == IDENT_GSI && !opt_gekko_gsi_detect);
		exclude_me |= (info->ident == IDENT_GSF && !opt_gekko_gsf_detect);
		exclude_me |= (info->ident == IDENT_GSFM && !opt_gekko_r909_detect);
	}

	if (opt_gekko_serial != NULL
	&&  (strstr(opt_gekko_serial, compac->usbdev->serial_string) == NULL))
	{
		exclude_me = true;
	}

	if (exclude_me)
	{
		usb_uninit(compac);
		free(info);
		compac->device_data = NULL;
		return NULL;
	}

	switch (info->ident)
	{
		case IDENT_BSC:
		case IDENT_GSC:
		case IDENT_BSD:
		case IDENT_GSD:
		case IDENT_BSE:
		case IDENT_GSE:
			info->asic_type = BM1384;

			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_IFC_ENABLE,
						CP210X_VALUE_UART_ENABLE, info->interface, NULL, 0, C_ENABLE_UART);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_DATA,
						CP210X_VALUE_DATA, info->interface, NULL, 0, C_SETDATA);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD,
						0, info->interface, &baudrate, sizeof (baudrate), C_SETBAUD);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_SET_LINE_CTL,
						bits, info->interface, NULL, 0, C_SETPARITY);
			break;
		case IDENT_GSH:
			info->asic_type = BM1387;
			info->expected_chips = 2;
			break;
		case IDENT_GSI:
			info->asic_type = BM1387;
			info->expected_chips = 12;
			break;
		case IDENT_GSF:
		case IDENT_GSFM:
			info->asic_type = BM1397;
			// at least 1
			info->expected_chips = 1;
			break;
		default:
			quit(1, "%d: %s compac_detect_one() invalid %s ident=%d",
				compac->cgminer_id, compac->drv->dname, compac->drv->dname, info->ident);
	}

	info->min_job_id = 0x10;
	switch (info->asic_type)
	{
		case BM1384:
			info->rx_len = 5;
			info->task_len = 64;
			info->cores = 55;
			info->add_job_id = 1;
			info->max_job_id = 0x1f;
			info->midstates = 1;
			info->can_boost = false;
			break;
		case BM1387:
			info->rx_len = 7;
			info->task_len = 54;
			info->cores = 114;
			info->add_job_id = 1;
			info->max_job_id = 0x7f;
			info->midstates = (opt_gekko_lowboost) ? 2 : 4;
			info->can_boost = true;
			compac_toggle_reset(compac);
			break;
		case BM1397:
			info->rx_len = 9;
			info->task_len = 54;
			info->cores = 672;
			info->add_job_id = 4;
			info->max_job_id = 0x7f;
			// ignore lowboost
			info->midstates = 4;
			info->can_boost = true;
			compac_toggle_reset(compac);
			break;
		default:
			break;
	}

	info->interface = usb_interface(compac);
	info->mining_state = MINER_INIT;

	applog(LOG_DEBUG, "Using interface %d", info->interface);

	if (!add_cgpu(compac))
		quit(1, "Failed to add_cgpu in compac_detect_one");

	update_usb_stats(compac);

	for (i = 0; i < 8; i++)
		compac->unique_id[i] = compac->unique_id[i+3];
	compac->unique_id[8] = 0;

	info->wait_factor = info->wait_factor0;
	if (!opt_gekko_noboost && info->vmask && (info->asic_type == BM1387 || info->asic_type == BM1397))
		info->wait_factor *= info->midstates;

	return compac;
}

static void compac_detect(bool __maybe_unused hotplug)
{
	usb_detect(&bm1397_drv, compac_detect_one);
}

static bool compac_prepare(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	int device = (compac->usbinfo.bus_number * 0xff + compac->usbinfo.device_address) % 0xffff;

	mutex_lock(&static_lock);
	init_count = &dev_init_count[device];
	(*init_count)++;
	info->init_count = (*init_count);
	mutex_unlock(&static_lock);

	if (info->init_count == 1) {
		applog(LOG_WARNING, "%d: %s %d - %s (%s)",
			compac->cgminer_id, compac->drv->name, compac->device_id,
			compac->usbdev->prod_string, compac->unique_id);
	} else {
		applog(LOG_INFO, "%d: %s %d - init_count %d",
			compac->cgminer_id, compac->drv->name, compac->device_id,
			info->init_count);
	}

	info->thr = thr;
	info->bauddiv = 0x19; // 115200
	//info->bauddiv = 0x0D; // 214286
	//info->bauddiv = 0x07; // 375000

	//Sanity check and abort to prevent miner thread from being created.
	if (info->asic_type == BM1387) {
		// Ping Micro
		info->micro_found = 0;
/*
		if (info->asic_type == BM1387) {
			info->vcore = bound(opt_gekko_gsh_vcore, 300, 810);
			info->micro_found = 1;
			if (!compac_micro_send(compac, M1_GET_TEMP, 0x00, 0x00)) {
				info->micro_found = 0;
				applog(LOG_INFO, "%d: %s %d - micro not found : dummy mode", compac->cgminer_id, compac->drv->name, compac->device_id);
			} else {
				uint8_t vcc = (info->vcore / 1000.0 - 0.3) / 0.002;
				applog(LOG_INFO, "%d: %s %d - requesting vcore of %dmV (%x)", compac->cgminer_id, compac->drv->name, compac->device_id, info->vcore, vcc);
				compac_micro_send(compac, M2_SET_VCORE, 0x00, vcc);   // Default 400mV
			}
		}
*/

	}

	if (info->init_count != 0 && info->init_count % 5 == 0) {
		applog(LOG_INFO, "%d: %s %d - forcing usb_nodev()", compac->cgminer_id, compac->drv->name, compac->device_id);
		usb_nodev(compac);
	} else if (info->init_count > 1) {
		if (info->init_count > 10) {
			compac->deven = DEV_DISABLED;
		} else {
			cgsleep_ms(MS_SECOND_5);
		}
	}

	return true;
}

static void compac_statline(char *buf, size_t bufsiz, struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct timeval now;
	unsigned int i;

	char ab[2];
	char asic_stat[64];
	char asic_statline[512];
	char ms_stat[64];
	char eff_stat[64];
	uint32_t len = 0;

	memset(asic_statline, 0, sizeof(asic_statline));
	memset(asic_stat, 0, sizeof(asic_stat));
	memset(ms_stat, 0, sizeof(ms_stat));
	memset(eff_stat, 0, sizeof(eff_stat));

	if (info->chips == 0) {
		if (info->init_count > 1) {
			sprintf(asic_statline, "found 0 chip(s)");
		}

		for (i = strlen(asic_statline); i < stat_len + 15; i++)
			asic_statline[i] = ' ';

		tailsprintf(buf, bufsiz, "%s", asic_statline);
		return;
	}

	ab[0] = (info->boosted) ? '+' : 0;
	ab[1] = 0;

	if (info->chips > chip_max)
		chip_max = info->chips;

	cgtime(&now);

	if (opt_widescreen) {
		asic_stat[0] = '[';

		for (i = 1; i <= info->chips; i++) {
			struct ASIC_INFO *asic = &info->asics[i - 1];

			switch (asic->state) {
				case ASIC_HEALTHY:
					asic_stat[i] = 'o';
					break;
				case ASIC_HALFDEAD:
					asic_stat[i] = '-';
					break;
				case ASIC_ALMOST_DEAD:
					asic_stat[i] = '!';
					break;
				case ASIC_DEAD:
					asic_stat[i] = 'x';
					break;
			}

		}
		asic_stat[info->chips + 1] = ']';
		for (i = 1; i <= (chip_max - info->chips) + 1; i++)
			asic_stat[info->chips + 1 + i] = ' ';
	}

	sprintf(ms_stat, "(%.0f:%.0f)", info->task_ms, info->fullscan_ms);

	uint8_t wuc = (ms_tdiff(&now, &info->last_wu_increase) > MS_MINUTE_1) ? 32 : 94;

	if (info->eff_gs >= 99.9 && info->eff_wu >= 98.9) {
		sprintf(eff_stat, "|  100%% WU:100%%");
	} else if (info->eff_gs >= 99.9) {
		sprintf(eff_stat, "|  100%% WU:%c%2.0f%%", wuc, info->eff_wu);
	} else if (info->eff_wu >= 98.9) {
		sprintf(eff_stat, "| %4.1f%% WU:100%%", info->eff_gs);
	} else {
		sprintf(eff_stat, "| %4.1f%% WU:%c%2.0f%%", info->eff_gs, wuc, info->eff_wu);
	}

	if (info->asic_type == BM1387 || info->asic_type == BM1397) {
		char *chipnam = ((info->asic_type == BM1387) ? "BM1387" : "BM1397");

		if (info->micro_found) {
			sprintf(asic_statline, "%s:%02d%-1s %.2fMHz T:%.0f P:%.0f %s %.0fF",
				chipnam, info->chips, ab, info->frequency, info->frequency_requested,
				info->frequency_computed, ms_stat, info->micro_temp);
		} else {
			if (opt_widescreen) {
				sprintf(asic_statline, "%s:%02d%-1s %.0f/%.0f/%3.0f %s %s",
					chipnam, info->chips, ab, info->frequency, info->frequency_requested,
					info->frequency_computed, ms_stat, asic_stat);
			} else {
				sprintf(asic_statline, "%s:%02d%-1s %.2fMHz T:%-3.0f P:%-3.0f %s %s",
					chipnam, info->chips, ab, info->frequency, info->frequency_requested,
					info->frequency_computed, ms_stat, asic_stat);
			}
		}
	} else {
		if (opt_widescreen) {
			sprintf(asic_statline, "BM1384:%02d  %.0f/%.0f/%.0f %s %s",
				info->chips, info->frequency, info->frequency_requested,
				info->frequency_computed, ms_stat, asic_stat);
		} else {
			sprintf(asic_statline, "BM1384:%02d  %.2fMHz T:%-3.0f P:%-3.0f %s %s",
				info->chips, info->frequency, info->frequency_requested,
				info->frequency_computed, ms_stat, asic_stat);
		}
	}

	len = strlen(asic_statline);
	if (len > stat_len || opt_widescreen != last_widescreen) {
		mutex_lock(&static_lock);
		stat_len = len;
		last_widescreen = opt_widescreen;
		mutex_unlock(&static_lock);
	}

	for (i = len; i < stat_len; i++)
		asic_statline[i] = ' ';

	strcat(asic_statline, eff_stat);
	asic_statline[63] = 0;

	tailsprintf(buf, bufsiz, "%s", asic_statline);
}

static struct api_data *compac_api_stats(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct api_data *root = NULL;
	struct timeval now;
	char nambuf[64], buf256[256];
	double taskdiff, tps, ghs, off;
	time_t secs;
	size_t len;
	int i, j, k;

	cgtime(&now);
	taskdiff = tdiff(&now, &(info->first_task));
	if (taskdiff == 0 || info->tasks < 2)
		tps = 0;
	else
		tps = (double)(info->tasks - 1) / taskdiff;

	root = api_add_string(root, "Serial", compac->usbdev->serial_string, false);
	root = api_add_int(root, "Nonces", &info->nonces, false);
	root = api_add_int(root, "Accepted", &info->accepted, false);
	root = api_add_double(root, "TasksPerSec", &tps, true);
	root = api_add_uint64(root, "Tasks", &info->tasks, false);
	root = api_add_uint64(root, "BusyWork", &info->busy_work, false);
	root = api_add_int(root, "Midstates", &info->midstates, false);
	root = api_add_uint64(root, "MaxTaskWait", &info->max_task_wait, false);
	root = api_add_float(root, "WaitFactor0", &info->wait_factor0, false);
	root = api_add_float(root, "WaitFactor", &info->wait_factor, false);
	root = api_add_float(root, "FreqBase", &info->freq_base, false);
	root = api_add_float(root, "FreqFail", &info->freq_fail, false);
	root = api_add_uint32(root, "TicketDiff", &info->difficulty, false);
	root = api_add_hex32(root, "TicketMask", &info->ticket_mask, false);
	root = api_add_int64(root, "TicketNonces", &info->ticket_nonces, false);
	root = api_add_int64(root, "TicketBelow", &info->below_nonces, false);
	root = api_add_bool(root, "TicketOK", &info->ticket_ok, false);
	root = api_add_float(root, "NonceExpect", &info->nonce_expect, false);
	root = api_add_float(root, "NonceLimit", &info->nonce_limit, false);

	// info->gh access must be under lock
	mutex_lock(&info->ghlock);
	ghs = gekko_gh_hashrate(info, &now, true) / 1.0e3;
	secs = now.tv_sec - info->gh.zerosec;
	root = api_add_time(root, "GHZeroDelta", &secs, true);
	root = api_add_int(root, "GHLast", &info->gh.last, true);
	root = api_add_int(root, "GHNonces", &info->gh.noncesum, true);
	root = api_add_int64(root, "GHDiff", &info->gh.diffsum, true);
	root = api_add_double(root, "GHGHs", &ghs, true);
	mutex_unlock(&info->ghlock);
	root = api_add_float(root, "Require", &info->ghrequire, true);
	ghs = info->frequency * (double)(info->cores * info->chips) * info->ghrequire / 1.0e3;
	root = api_add_double(root, "RequireGH", &ghs, true);

	// info->job access must be under lock
	// N.B. this is as at the last job sent, not 'now'
	mutex_lock(&info->joblock);
	off = tdiff(&now, &(info->job.lastjob));
	root = api_add_double(root, "JobDataAge", &off, true);

	buf256[0] = '\0';
	for (i = 0; i < JOBMIN; i++)
	{
		j = JOBOFF(info->job.offset - i);
		len = strlen(buf256);
		// /, digit, null = 3
		if ((len - sizeof(buf256)) < 3)
			break;
		snprintf(buf256+len, sizeof(buf256)-len, "/%d", info->job.jobnum[j]);
	}
	root = api_add_string(root, "Jobs", buf256+1, true);

	buf256[0] = '\0';
	for (i = 0; i < JOBMIN; i++)
	{
		double elap;
		j = JOBOFF(info->job.offset - i);
		elap = tdiff(&(info->job.lastj[j]), &(info->job.firstj[j]));
		len = strlen(buf256);
		// /, digit, null = 3
		if ((len - sizeof(buf256)) < 3)
			break;
		snprintf(buf256+len, sizeof(buf256)-len, "/%.2f", elap);
	}
	root = api_add_string(root, "JobElapsed", buf256+1, true);

	buf256[0] = '\0';
	for (i = 0; i < JOBMIN; i++)
	{
		double jps, elap;
		j = JOBOFF(info->job.offset - i);
		elap = tdiff(&(info->job.lastj[j]), &(info->job.firstj[j]));
		if (elap == 0)
			jps = 0;
		else
			jps = (double)(info->job.jobnum[j] - 1) / elap;
		len = strlen(buf256);
		// /, digit, null = 3
		if ((len - sizeof(buf256)) < 3)
			break;
		snprintf(buf256+len, sizeof(buf256)-len, "/%.2f", jps);
	}
	root = api_add_string(root, "JobsPerSec", buf256+1, true);

	buf256[0] = '\0';
	for (i = 0; i < JOBMIN; i++)
	{
		j = JOBOFF(info->job.offset - i);
		len = strlen(buf256);
		// /, digit, null = 3
		if ((len - sizeof(buf256)) < 3)
			break;
		snprintf(buf256+len, sizeof(buf256)-len, "/%.2f", info->job.avgms[j]);
	}
	root = api_add_string(root, "JobsAvgms", buf256+1, true);

	buf256[0] = '\0';
	for (i = 0; i < JOBMIN; i++)
	{
		j = JOBOFF(info->job.offset - i);
		len = strlen(buf256);
		// /, digit, null = 3
		if ((len - sizeof(buf256)) < 3)
			break;
		snprintf(buf256+len, sizeof(buf256)-len, "/%.2f:%.2f",
				info->job.minms[j], info->job.maxms[j]);
	}
	root = api_add_string(root, "JobsMinMaxms", buf256+1, true);

	mutex_unlock(&info->joblock);

	for (i = 0; i < (int)CUR_ATTEMPT; i++)
	{
		snprintf(nambuf, sizeof(nambuf), "cur_off_%d_%d", i, cur_attempt[i]);
		root = api_add_uint64(root, nambuf, &info->cur_off[i], true);
	}
	root = api_add_double(root, "Rolling", &info->rolling, false);
	root = api_add_int(root, "Resets", &info->fail_count, false);
	root = api_add_float(root, "Frequency", &info->frequency, false);
	root = api_add_float(root, "FreqComp", &info->frequency_computed, false);
	root = api_add_float(root, "FreqReq", &info->frequency_requested, false);
	root = api_add_float(root, "FreqStart", &info->frequency_start, false);
	root = api_add_float(root, "FreqSel", &info->frequency_selected, false);
	//root = api_add_temp(root, "Temp", &info->micro_temp, false);
	root = api_add_int(root, "Dups", &info->dupsall, true);
	root = api_add_int(root, "DupsReset", &info->dupsreset, true);
	root = api_add_uint(root, "Chips", &info->chips, false);
	root = api_add_bool(root, "FreqLocked", &info->lock_freq, false);
	if (info->asic_type == BM1397)
		root = api_add_int(root, "USBProp", &info->usb_prop, false);
	mutex_lock(&info->ghlock);
	for (i = 0; i < (int)info->chips; i++)
	{
		struct ASIC_INFO *asic = &info->asics[i];
		snprintf(nambuf, sizeof(nambuf), "Chip%dNonces", i);
		root = api_add_int(root, nambuf, &asic->nonces, true);
		snprintf(nambuf, sizeof(nambuf), "Chip%dDups", i);
		root = api_add_uint(root, nambuf, &asic->dupsall, true);

		gc_offset(info, asic, &now, false, true);
		snprintf(nambuf, sizeof(nambuf), "Chip%dRanges", i);
		buf256[0] = '\0';
		for (j = 0; j < CHNUM; j++)
		{
			len = strlen(buf256);
			// slash, digit, null = 3
			if ((len - sizeof(buf256)) < 3)
				break;
			k = CHOFF(asic->gc.offset - j);
			snprintf(buf256+len, sizeof(buf256)-len, "/%d", asic->gc.noncenum[k]);
		}
		len = strlen(buf256);
		if ((len - sizeof(buf256)) >= 3)
			snprintf(buf256+len, sizeof(buf256)-len, "/%d", asic->gc.noncesum);
		len = strlen(buf256);
		if ((len - sizeof(buf256)) >= 3)
			snprintf(buf256+len, sizeof(buf256)-len, "/%.2f%%", noncepercent(info, i, &now));
		root = api_add_string(root, nambuf, buf256+1, true);

		snprintf(nambuf, sizeof(nambuf), "Chip%dFreqSend", i);
		root = api_add_float(root, nambuf, &asic->frequency, true);
		snprintf(nambuf, sizeof(nambuf), "Chip%dFreqReply", i);
		root = api_add_float(root, nambuf, &asic->frequency_reply, true);
	}
	mutex_unlock(&info->ghlock);

	for (i = 0; i < 16; i++)
	{
		snprintf(nambuf, sizeof(nambuf), "NonceByte-%1X0", i);
		buf256[0] = '\0';
		for (j = 0; j < 16; j++)
		{
			len = strlen(buf256);
			// dot, digit, null = 3
			if ((len - sizeof(buf256)) < 3)
				break;
			snprintf(buf256+len, sizeof(buf256)-len, ".%"PRId64, info->noncebyte[i*16+j]);
		}
		root = api_add_string(root, nambuf, buf256+1, true);
	}

	for (i = 0; i < 16; i++)
	{
		snprintf(nambuf, sizeof(nambuf), "nb2c-%1X0", i);
		buf256[0] = '\0';
		for (j = 0; j < 16; j++)
		{
			len = strlen(buf256);
			// dot, digit, null = 3
			if ((len - sizeof(buf256)) < 3)
				break;
			snprintf(buf256+len, sizeof(buf256)-len, ".%u", info->nb2chip[i*16+j]);
		}
		root = api_add_string(root, nambuf, buf256+1, true);
	}

	root = api_add_uint64(root, "NTimeout", &info->ntimeout, false);
	root = api_add_uint64(root, "NTrigger", &info->ntrigger, false);

#if TUNE_CODE
	mutex_lock(&info->slock);
	uint64_t num = info->num;
	double req = info->req;
	double fac = info->fac;
	uint64_t num1_1 = info->num1_1;
	double req1_1 = info->req1_1;
	double fac1_1 = info->fac1_1;
	uint64_t num1_5 = info->num1_5;
	double req1_5 = info->req1_5;
	double fac1_5 = info->fac1_5;
	uint64_t inv = info->inv;
	mutex_unlock(&info->slock);

	double avg, res, avg1_1, res1_1, avg1_5, res1_5;

	if (num == 0)
		avg = res = 0.0;
	else
	{
		avg = req / (double)num;
		res = fac / (double)num;
	}
	if (num1_1 == 0)
		avg1_1 = res1_1 = 0.0;
	else
	{
		avg1_1 = req1_1 / (double)num1_1;
		res1_1 = fac1_1 / (double)num1_1;
	}
	if (num1_5 == 0)
		avg1_5 = res1_5 = 0.0;
	else
	{
		avg1_5 = req1_5 / (double)num1_5;
		res1_5 = fac1_5 / (double)num1_5;
	}

	root = api_add_uint64(root, "SleepN", &num, true);
	root = api_add_double(root, "SleepAvgReq", &avg, true);
	root = api_add_double(root, "SleepAvgRes", &res, true);
	root = api_add_uint64(root, "SleepN1_1", &num1_1, true);
	root = api_add_double(root, "SleepAvgReq1_1", &avg1_1, true);
	root = api_add_double(root, "SleepAvgRes1_1", &res1_1, true);
	root = api_add_uint64(root, "SleepN1_5", &num1_5, true);
	root = api_add_double(root, "SleepAvgReq1_5", &avg1_5, true);
	root = api_add_double(root, "SleepAvgRes1_5", &res1_5, true);
	root = api_add_uint64(root, "SleepInv", &inv, true);

	root = api_add_uint64(root, "WorkGenNum", &info->work_usec_num, true);
	root = api_add_double(root, "WorkGenAvg", &info->work_usec_avg, true);

	if (info->over1num == 0)
		avg = 0.0;
	else
		avg = info->over1amt / (double)(info->over1num);
	root = api_add_int64(root, "Over1N", &info->over1num, true);
	root = api_add_double(root, "Over1Avg", &avg, true);

	if (info->over2num == 0)
		avg = 0.0;
	else
		avg = info->over2amt / (double)(info->over2num);
	root = api_add_int64(root, "Over2N", &info->over2num, true);
	root = api_add_double(root, "Over2Avg", &avg, true);
#endif

#if STRATUM_WORK_TIMING
	cg_rlock(&swt_lock);
	uint64_t swc = stratum_work_count;
	uint64_t swt = stratum_work_time;
	uint64_t swmin = stratum_work_min;
	uint64_t swmax = stratum_work_max;
	uint64_t swt0 = stratum_work_time0;
	uint64_t swt10 = stratum_work_time10;
	uint64_t swt100 = stratum_work_time100;
	cg_runlock(&swt_lock);

	double sw_avg;

	if (swc == 0)
		sw_avg = 0.0;
	else
		sw_avg = (double)swt / (double)swc;

	root = api_add_uint64(root, "SWCount", &swc, true);
	root = api_add_double(root, "SWAvg", &sw_avg, true);
	root = api_add_uint64(root, "SWMin", &swmin, true);
	root = api_add_uint64(root, "SWMax", &swmax, true);
	root = api_add_uint64(root, "SW0Count", &swt0, true);
	root = api_add_uint64(root, "SW10Count", &swt10, true);
	root = api_add_uint64(root, "SW100Count", &swt100, true);
#endif

	return root;
}

static void compac_shutdown(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	applog(LOG_INFO, "%d: %s %d - shutting down", compac->cgminer_id, compac->drv->name, compac->device_id);
	if (!compac->usbinfo.nodev) {
		if (info->asic_type == BM1387) {
			compac_micro_send(compac, M2_SET_VCORE, 0x00, 0x00);   // 300mV
			compac_toggle_reset(compac);
		} else if (info->asic_type == BM1397) {
			calc_gsf_freq(compac, 0, -1);
			compac_toggle_reset(compac);
		} else if (info->asic_type == BM1384 && info->frequency != info->frequency_default) {
			float frequency = info->frequency - info->freq_mult;
			while (frequency > info->frequency_default) {
				compac_set_frequency(compac, frequency);
				frequency -= info->freq_mult;
				cgsleep_ms(100);
			}
			compac_set_frequency(compac, info->frequency_default);
			compac_send_chain_inactive(compac);
		}
	}
	info->mining_state = MINER_SHUTDOWN;
	pthread_join(info->rthr.pth, NULL); // Let thread close.
	pthread_join(info->wthr.pth, NULL); // Let thread close.
	if (info->asic_type == BM1397)
		pthread_join(info->nthr.pth, NULL); // Let thread close.
	PTH(thr) = 0L;
}

uint64_t bound(uint64_t value, uint64_t lower_bound, uint64_t upper_bound)
{
	if (value < lower_bound)
		return lower_bound;
	if (value > upper_bound)
		return upper_bound;
	return value;
}

void stuff_reverse(unsigned char *dst, unsigned char *src, uint32_t len)
{
	uint32_t i;
	for (i = 0; i < len; i++) {
		dst[i] = src[len - i - 1];
	}
}

void stuff_lsb(unsigned char *dst, uint32_t x)
{
	dst[0] = (x >>  0) & 0xff;
	dst[1] = (x >>  8) & 0xff;
	dst[2] = (x >> 16) & 0xff;
	dst[3] = (x >> 24) & 0xff;
}

void stuff_msb(unsigned char *dst, uint32_t x)
{
	dst[0] = (x >> 24) & 0xff;
	dst[1] = (x >> 16) & 0xff;
	dst[2] = (x >>  8) & 0xff;
	dst[3] = (x >>  0) & 0xff;
}

static char *compac_api_set(struct cgpu_info *compac, char *option, char *setting, char *replybuf, size_t siz)
{
	struct COMPAC_INFO *info = compac->device_data;
	float freq;

	if (strcasecmp(option, "help") == 0)
	{
		// freq: all of the drivers automatically fix the value
		//	BM1397 0 is a special case, since it 'works'
		if (info->asic_type == BM1397)
		{
			snprintf(replybuf, siz, "reset freq: 0-1200 chip: N:0-800 target: 0-1200"
						" lockfreq unlockfreq waitfactor: 0.01-2.0"
						" usbprop: %d-1000 require: 0.0-0.8", USLEEPMIN);
		}
		else
		{
			snprintf(replybuf, siz, "reset freq: 0-1200 chip: N:0-800 target: 0-1200"
						" lockfreq unlockfreq waitfactor: 0.01-2.0"
						" require: 0.0-0.8");
		}
                return replybuf;
	}

	if (strcasecmp(option, "reset") == 0)
	{
		// will cause various problems ...
		info->mining_state = MINER_RESET;
		return NULL;
	}

	// set all chips to freq
	if (strcasecmp(option, "freq") == 0)
	{
		if (!setting || !*setting)
		{
			snprintf(replybuf, siz, "missing freq");
			return replybuf;
		}

		freq = limit_freq(info, atof(setting), true);
		freq = FREQ_BASE(freq);

		change_freq_any(compac, freq);

		return NULL;
	}

	// set chip:freq
	if (strcasecmp(option, "chip") == 0)
	{
		char *fpos;
		int chip;

		if (!setting || !*setting)
		{
			snprintf(replybuf, siz, "missing chip:freq");
			return replybuf;
		}

		if (info->asic_type != BM1397)
		{
			snprintf(replybuf, siz, "chip:freq only valid for BM1397");
			return replybuf;
		}

		fpos = strchr(setting, ':');
		if (!fpos || fpos == setting || *(fpos+1) == '\0')
		{
			snprintf(replybuf, siz, "not chip:freq");
			return replybuf;
		}

		// atoi will stop at the ':'
		chip = atoi(setting);
		if (chip < 0 || chip >= (int)(info->chips))
		{
			snprintf(replybuf, siz, "invalid chip %d", chip);
			return replybuf;
		}

		freq = limit_freq(info, atof(fpos+1), true);
		freq = FREQ_BASE(freq);

		calc_gsf_freq(compac, freq, chip);

		return NULL;
	}

	if (strcasecmp(option, "target") == 0)
	{
		if (!setting || !*setting)
		{
			snprintf(replybuf, siz, "missing freq");
			return replybuf;
		}

		freq = limit_freq(info, atof(setting), true);
		freq = FREQ_BASE(freq);

		info->frequency_requested = freq;

		return NULL;
	}

	if (strcasecmp(option, "lockfreq") == 0)
	{
		info->lock_freq = true;

		return NULL;
	}

	if (strcasecmp(option, "unlockfreq") == 0)
	{
		info->lock_freq = false;

		return NULL;
	}

	// set wait-factor
	if (strcasecmp(option, "waitfactor") == 0)
	{
		if (!setting || !*setting)
		{
			snprintf(replybuf, siz, "missing value");
			return replybuf;
		}

		info->wait_factor0 = freqbounding(atof(setting), 0.01, 2.0);
		compac_update_rates(compac);

		return NULL;
	}

	// set work propagation time
	if (strcasecmp(option, "usbprop") == 0)
	{
		if (info->asic_type != BM1397)
		{
			snprintf(replybuf, siz, "usbprop only for BM1397");
			return replybuf;
		}

		if (!setting || !*setting)
		{
			snprintf(replybuf, siz, "missing usec");
			return replybuf;
		}

		info->usb_prop = (int)bound(atoi(setting), USLEEPMIN, 1000);

		return NULL;
	}

	// set ghrequire
	if (strcasecmp(option, "require") == 0)
	{
		if (!setting || !*setting)
		{
			snprintf(replybuf, siz, "missing value");
			return replybuf;
		}

		info->ghrequire = freqbounding(atof(setting), 0.0, 0.8);
		compac_update_rates(compac);

		return NULL;
	}

	snprintf(replybuf, siz, "Unknown option: %s", option);
	return replybuf;
}

struct device_drv bm1397_drv = {
	.drv_id              = DRIVER_bm1397,
	.dname               = "bm1397",
	.name                = "H97",
	.hash_work           = hash_queued_work,
	.get_api_stats       = compac_api_stats,
	.get_statline_before = compac_statline,
	.set_device	     	 = compac_api_set,
	.drv_detect          = compac_detect,
	.scanwork            = compac_scanwork,
	.flush_work          = compac_flush_work,
	.update_work         = compac_update_work,
	.thread_prepare      = compac_prepare,
	.thread_init         = compac_init,
	.thread_shutdown     = compac_shutdown,
};
