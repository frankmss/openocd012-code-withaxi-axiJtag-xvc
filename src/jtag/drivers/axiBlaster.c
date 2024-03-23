/***************************************************************************
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/**
 * @file
这个驱动只在 zynq
linux上使用，zynq4205矿机中没有usb接口，所以不能使用ft芯片，为了改善sysgpio的效率
所以在zynq的pl端实现jtag
port，这部分代码与usb-blaster的驱动一致，因此只改变usb-blaster驱动的地层读写部分就可以了
zynq的硬件结构
---------------
|             |    |---------------|   |-------|    |-----------------|
|    ps       |--->|  axi lite bus |-->|  fifo |--->|   usb-blaster   |
|             |    |---------------|   |-------|    |-----------------|
|--------------
因此ps端的驱动程序地层操作，只是读写register即可
author:cahill
date：2021.11.2
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/replacements.h>
#include <helper/time_support.h>
//#include <jtag/commands.h>
#include <jtag/interface.h>

/* system includes */
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
/* Size of USB endpoint max packet size, ie. 64 bytes */
#define MAX_PACKET_SIZE 64
/*
 * Size of data buffer that holds bytes in byte-shift mode.
 * This buffer can hold multiple USB packets aligned to
 * MAX_PACKET_SIZE bytes boundaries.
 * BUF_LEN must be grater than or equal MAX_PACKET_SIZE.
 */
#define BUF_LEN 4096

/* USB-Blaster II specific command */
#define CMD_COPY_TDO_BUFFER 0x5F

enum gpio_steer {
  FIXED_0 = 0,
  FIXED_1,
  SRST,
  TRST,
};
/* Low level flags */
#define COPY_TDO_BUFFER (1 << 0)
struct axiblast_lowlevel {
  int fd;
  unsigned int *map_base;
  unsigned int map_size;

  unsigned int axiblaster_phyreg_base;
  unsigned int axiblaster_phyFifo_full_addr;
  unsigned int axiblaster_phyFifo_empty_addr;
  unsigned int axiblaster_phyFifo_read_addr;
  unsigned int axiblaster_phyFifo_write_addr;
  unsigned int axiblaster_phyClk_reg_addr;

  unsigned int *vBase_addr;
  unsigned int *vfull_addr;
  unsigned int *vempty_addr;
  unsigned int *vwrite_addr;
  unsigned int *vread_addr;
  unsigned int *vclk_addr;

  unsigned int axiblaster_jtagclk;
  int (*write)(struct axiblast_lowlevel *low, uint8_t *buf, int size,
               uint32_t *bytes_written);
  int (*read)(struct axiblast_lowlevel *low, uint8_t *buf, unsigned size,
              uint32_t *bytes_read);
  int (*axiSpeed)(struct axiblast_lowlevel *low, int speed);
  // int (*open)(struct axiblast_lowlevel *low);
  // int (*close)(struct axiblast_lowlevel *low);
  // int (*speed)(struct axiblast_lowlevel *low, int speed);
  void *priv;
};
struct axiblaster_info {
  enum gpio_steer pin6;
  enum gpio_steer pin8;
  int tms;
  int tdi;
  bool trst_asserted;
  bool srst_asserted;
  uint8_t buf[BUF_LEN];
  int bufidx;

  char *lowlevel_name;
  struct axiblast_lowlevel *drv;
  char *axiblast_device_desc;
  uint16_t axiblast_vid, axiblast_pid;
  uint16_t axiblast_vid_uninit, axiblast_pid_uninit;
  int flags;
  char *firmware_path;
};






#if 0
static unsigned int tmp_delay=0;
static int axiblaster_delay(int delay){
	for (int i = 0; i < delay; i++){
		for(int k=0; k<1; k++){
			asm volatile ("");
			tmp_delay++;
			tmp_delay = ~tmp_delay;
			tmp_delay = tmp_delay + k;
		}
	}

	return 0;
}
#endif

#define DEBUG_TYPE_READ     0
#define DEBUG_TYPE_WRITE    1
#define DEBUG_TYPE_OCD_READ 2
#define DEBUG_TYPE_BUFFER   3
#define LINE_LEN  16
#if 0
static void axiblast_debug_buffer(uint8_t *buffer, int length, uint8_t type)
{
	char line[128];
	char s[4];
	int i;
	int j;

	switch (type) {
		case DEBUG_TYPE_READ:
			sprintf(line, "USB READ %d bytes", length);
			break;
		case DEBUG_TYPE_WRITE:
			sprintf(line, "USB WRITE %d bytes", length);
			break;
		case DEBUG_TYPE_OCD_READ:
			sprintf(line, "TO OpenOCD %d bytes", length);
			break;
		case DEBUG_TYPE_BUFFER:
			sprintf(line, "Buffer %d bytes", length);
			break;
	}

	//LOG_DEBUG("%s", line);
	printf("%s \n", line);
	for (i = 0; i < length; i += LINE_LEN) {
		switch (type) {
			case DEBUG_TYPE_READ:
				sprintf(line, "USB READ: %04x", i);
				break;
			case DEBUG_TYPE_WRITE:
				sprintf(line, "USB WRITE: %04x", i);
				break;
			case DEBUG_TYPE_OCD_READ:
				sprintf(line, "TO OpenOCD: %04x", i);
				break;
			case DEBUG_TYPE_BUFFER:
				sprintf(line, "BUFFER: %04x", i);
				break;
		}

		for (j = i; j < i + LINE_LEN && j < length; j++) {
			sprintf(s, " %02x", buffer[j]);
			strcat(line, s);
		}
		 //LOG_DEBUG("%s", line);
		printf("%s \n", line);
	}
}
#endif //if 0

static int read_zynq_axiBus(struct axiblast_lowlevel *dlow, uint8_t *buf,
                            unsigned size, uint32_t *bytes_read) {
 int timeout = 10000;
  unsigned int tmpVal;
  unsigned int ava_num = 0;
  unsigned int emptySign=0;
	struct axiblast_lowlevel *low = dlow;
  // unsigned int *empSign = low->vempty_addr;
  // unsigned int *readAddr = low->vread_addr;
  //LOG_INFO(" read_zynq_axiBus: 1");
  *bytes_read = 0;
  while ((*bytes_read < size) && timeout--) {
    //axiblaster_delay(10000);
		emptySign = *(low->vempty_addr);
    if(emptySign == 0){
      ava_num++;
    }
    // printf("emptySign=0x%x  ", emptySign);
    if(emptySign==0){
    //if(1){
			
      tmpVal = (*(low->vread_addr));
      *(buf + *bytes_read) = (uint8_t)tmpVal;
      //printf(":0x%x", *(buf + *bytes_read));
      (*bytes_read)++;  
    }else{
			jtag_sleep(1);
      //printf("read_zynq_axiBus waiting \n");
      //break;
    }
    
  }
  //printf("\n size: %d == %d\n", ava_num, size);
  //LOG_INFO(" read_zynq_axiBus: 2");
	//axiblast_debug_buffer(buf, *bytes_read, DEBUG_TYPE_READ);
  return ERROR_OK;
}
static int write_zynq_axiBus(struct axiblast_lowlevel *dlow, uint8_t *buf,
                             int size, uint32_t *bytes_written) {
 	uint8_t tmpVal = 0;
	
	struct axiblast_lowlevel *low = dlow;
  *bytes_written = 0;

  //axiblast_debug_buffer(buf, size, DEBUG_TYPE_WRITE);
	for (int i = 0; i < size; ) {
		//axiblaster_delay(10000);
    if (*(low->vfull_addr) == 0) {
			
      tmpVal = *(buf + *bytes_written);
      //printf("-0x%x ", tmpVal);
      *(low->vwrite_addr) = (unsigned int)tmpVal;
      (*bytes_written)++;
      i++;
    } else {
			// axiopenjtag_delay(10000);
      //break;
      jtag_sleep(1);
      //printf("write_zynq_axiBus wating.. \n");
    }
  }

  //LOG_INFO(" write_zynq_axiBus: writeByte_num:%d", *(bytes_written));
  return ERROR_OK;
}

/*
 * Global device control
 */
static struct axiblast_lowlevel _drv = {
    .read = read_zynq_axiBus,
    .write = write_zynq_axiBus,
    
};
static struct axiblaster_info info = {
    .axiblast_vid = 0x09fb, /* Altera */
    .axiblast_pid = 0x6001, /* USB-Blaster */
    .lowlevel_name = NULL,
    .srst_asserted = false,
    .trst_asserted = false,
    .pin6 = FIXED_1,
    .pin8 = FIXED_1,
    .drv = &(_drv),
};

// operate zynq hardware (axi lite bus)


static int axiblast_setSpeed(int speed){
  int clockcmd;
  clockcmd = (int) (6250/speed);
  //info.drv->axiSpeed(info.drv, clockcmd);
  *(_drv.vclk_addr) = clockcmd;
  return ERROR_OK;
}
static int axiblast_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}
static int axiblast_khz(int khz, int *jtag_speed)
{

	if (khz >= 48000)
		*jtag_speed = 48000;
	else if (khz >= 24000)
		*jtag_speed = 24000;
	else if (khz >= 12000)
		*jtag_speed = 12000;
	else if (khz >= 6000)
		*jtag_speed = 6000;
	else if (khz >= 3000)
		*jtag_speed = 3000;
	else if (khz >= 1500)
		*jtag_speed = 1500;
	else if (khz >= 750)
		*jtag_speed = 750;
	else
		*jtag_speed = 375;

	return ERROR_OK;
}
// low layer function
/*
 * Access functions to lowlevel driver, agnostic of libftdi/libftdxx
 */
static char *hexdump(uint8_t *buf, unsigned int size) {
  unsigned int i;
  char *str = calloc(size * 2 + 1, 1);

  for (i = 0; i < size; i++) sprintf(str + 2 * i, "%02x", buf[i]);
  return str;
}



static int axiblast_buf_read(uint8_t *buf, unsigned size,
                             uint32_t *bytes_read) {
  int ret = info.drv->read(info.drv, buf, size, bytes_read);
  char *str = hexdump(buf, *bytes_read);

  LOG_DEBUG_IO("(size=%d, buf=[%s]) -> %" PRIu32, size, str, *bytes_read);
  free(str);
  return ret;
}

static int axiblast_buf_write(uint8_t *buf, int size, uint32_t *bytes_written) {
  int ret = info.drv->write(info.drv, buf, size, bytes_written);
  char *str = hexdump(buf, *bytes_written);

  LOG_DEBUG_IO("(size=%d, buf=[%s]) -> %" PRIu32, size, str, *bytes_written);
  free(str);
  return ret;
}

static int nb_buf_remaining(void) { return BUF_LEN - info.bufidx; }

static void axiblast_flush_buffer(void) {
  uint32_t retlen;
  int nb = info.bufidx, ret = ERROR_OK;

  while (ret == ERROR_OK && nb > 0) {
    ret = axiblast_buf_write(info.buf, nb, &retlen);
    nb -= retlen;
  }
  info.bufidx = 0;
}
// low layer function over

/* top lay api to openocd
 */
/* Simple bit banging mode:
 *
 *   Bit 7 (0x80): Must be zero (see byte-shift mode above)
 *   Bit 6 (0x40): If set, you will receive a byte indicating the state of
 TDO
 *                 in return.
 *   Bit 5 (0x20): Output Enable/LED.
 *   Bit 4 (0x10): TDI Output.
 *   Bit 3 (0x08): nCS Output (not used in JTAG mode).
 *   Bit 2 (0x04): nCE Output (not used in JTAG mode).
 *   Bit 1 (0x02): TMS Output.
 *   Bit 0 (0x01): TCK Output.
 *
 * For transmitting a single data bit, you need to write two bytes (one for
 * setting up TDI/TMS/TCK=0, and one to trigger TCK high with same TDI/TMS
 * held). Up to 64 bytes can be combined in a single USB packet.
 * It isn't possible to read a data without transmitting data.
 */

#define TCK (1 << 0)
#define TMS (1 << 1)
#define NCE (1 << 2)
#define NCS (1 << 3)
#define TDI (1 << 4)
#define LED (1 << 5)
#define READ (1 << 6)
#define SHMODE (1 << 7)
#define READ_TDO (1 << 0)

/**
 * axiblast_queue_byte - queue one 'bitbang mode' byte for USB Blaster
 * @param abyte the byte to queue
 *
 * Queues one byte in 'bitbang mode' to the USB Blaster. The byte is not
 * actually sent, but stored in a buffer. The write is performed once
 * the buffer is filled, or if an explicit axiblast_flush_buffer() is called.
 */
static void axiblast_queue_byte(uint8_t abyte) {
  if (nb_buf_remaining() < 1) axiblast_flush_buffer();
  info.buf[info.bufidx++] = abyte;
  if (nb_buf_remaining() == 0) axiblast_flush_buffer();
  LOG_DEBUG_IO("(byte=0x%02x)", abyte);
}

/**
 * axiblast_compute_pin - compute if gpio should be asserted
 * @param steer control (ie. TRST driven, SRST driven, of fixed)
 *
 * Returns pin value (1 means driven high, 0 mean driven low)
 */
static bool axiblast_compute_pin(enum gpio_steer steer) {
  switch (steer) {
    case FIXED_0:
      return 0;
    case FIXED_1:
      return 1;
    case SRST:
      return !info.srst_asserted;
    case TRST:
      return !info.trst_asserted;
    default:
      return 1;
  }
}

/**
 * axiblast_build_out - build bitbang mode output byte
 * @param type says if reading back TDO is required
 *
 * Returns the compute bitbang mode byte
 */
static uint8_t axiblast_build_out(enum scan_type type) {
  uint8_t abyte = 0;

  abyte |= info.tms ? TMS : 0;
  abyte |= axiblast_compute_pin(info.pin6) ? NCE : 0;
  abyte |= axiblast_compute_pin(info.pin8) ? NCS : 0;
  abyte |= info.tdi ? TDI : 0;
  abyte |= LED;
  if (type == SCAN_IN || type == SCAN_IO) abyte |= READ;
  return abyte;
}

/**
 * axiblast_reset - reset the JTAG device is possible
 * @param trst 1 if TRST is to be asserted
 * @param srst 1 if SRST is to be asserted
 */
static void axiblast_reset(int trst, int srst) {
  uint8_t out_value;

  info.trst_asserted = trst;
  info.srst_asserted = srst;
  out_value = axiblast_build_out(SCAN_OUT);
  axiblast_queue_byte(out_value);
  axiblast_flush_buffer();
}

/**
 * axiblast_clock_tms - clock a TMS transition
 * @param tms the TMS to be sent
 *
 * Triggers a TMS transition (ie. one JTAG TAP state move).
 */
static void axiblast_clock_tms(int tms) {
  uint8_t out;

  LOG_DEBUG_IO("(tms=%d)", !!tms);
  info.tms = !!tms;
  info.tdi = 0;
  out = axiblast_build_out(SCAN_OUT);
  axiblast_queue_byte(out);
  axiblast_queue_byte(out | TCK);
}

/**
 * axiblast_idle_clock - put back TCK to low level
 *
 * See axiblast_queue_tdi() comment for the usage of this function.
 */
static void axiblast_idle_clock(void) {
  uint8_t out = axiblast_build_out(SCAN_OUT);

  LOG_DEBUG_IO(".");
  axiblast_queue_byte(out);
}

/**
 * axiblast_clock_tdi - Output a TDI with bitbang mode
 * @param tdi the TDI bit to be shifted out
 * @param type scan type (ie. does a readback of TDO is required)
 *
 * Output a TDI bit and assert clock to push it into the JTAG device :
 *  - writing out TCK=0, TMS=\<old_state>=0, TDI=\<tdi>
 *  - writing out TCK=1, TMS=\<new_state>, TDI=\<tdi> which triggers the JTAG
 *    device acquiring the data.
 *
 * If a TDO is to be read back, the required read is requested (bitbang
 mode),
 * and the USB Blaster will send back a byte with bit0 representing the TDO.
 */
static void axiblast_clock_tdi(int tdi, enum scan_type type) {
  uint8_t out;

  LOG_DEBUG_IO("(tdi=%d)", !!tdi);
  info.tdi = !!tdi;

  out = axiblast_build_out(SCAN_OUT);
  axiblast_queue_byte(out);

  out = axiblast_build_out(type);
  axiblast_queue_byte(out | TCK);
}

/**
 * axiblast_clock_tdi_flip_tms - Output a TDI with bitbang mode, change JTAG
 state
 * @param tdi the TDI bit to be shifted out
 * @param type scan type (ie. does a readback of TDO is required)
 *
 * This function is the same as axiblast_clock_tdi(), but it changes also the
 TMS
 * while output the TDI. This should be the last TDI output of a TDI
 * sequence, which will change state from :
 *   - IRSHIFT -> IREXIT1
 *   - or DRSHIFT -> DREXIT1
 */
static void axiblast_clock_tdi_flip_tms(int tdi, enum scan_type type) {
  uint8_t out;

  LOG_DEBUG_IO("(tdi=%d)", !!tdi);
  info.tdi = !!tdi;
  info.tms = !info.tms;

  out = axiblast_build_out(SCAN_OUT);
  axiblast_queue_byte(out);

  out = axiblast_build_out(type);
  axiblast_queue_byte(out | TCK);

  out = axiblast_build_out(SCAN_OUT);
  axiblast_queue_byte(out);
}

/**
 * axiblast_queue_bytes - queue bytes for the USB Blaster
 * @param bytes byte array
 * @param nb_bytes number of bytes
 *
 * Queues bytes to be sent to the USB Blaster. The bytes are not
 * actually sent, but stored in a buffer. The write is performed once
 * the buffer is filled, or if an explicit axiblast_flush_buffer() is called.
 */
static void axiblast_queue_bytes(uint8_t *bytes, int nb_bytes) {
  if (info.bufidx + nb_bytes > BUF_LEN) {
    LOG_ERROR("buggy code, should never queue more that %d bytes",
              info.bufidx + nb_bytes);
    exit(-1);
  }
  LOG_DEBUG_IO("(nb_bytes=%d, bytes=[0x%02x, ...])", nb_bytes,
               bytes ? bytes[0] : 0);
  if (bytes)
    memcpy(&info.buf[info.bufidx], bytes, nb_bytes);
  else
    memset(&info.buf[info.bufidx], 0, nb_bytes);
  info.bufidx += nb_bytes;
  if (nb_buf_remaining() == 0) axiblast_flush_buffer();
}

/**
 * axiblast_tms_seq - write a TMS sequence transition to JTAG
 * @param bits TMS bits to be written (bit0, bit1 .. bitN)
 * @param nb_bits number of TMS bits (between 1 and 8)
 * @param skip number of TMS bits to skip at the beginning of the series
 *
 * Write a series of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=\<new_state>, TDI=\<???>
 *  - writing out TCK=1, TMS=\<new_state>, TDI=\<???> which triggers the
 transition
 * The function ensures that at the end of the sequence, the clock (TCK) is
 put
 * low.
 */
static void axiblast_tms_seq(const uint8_t *bits, int nb_bits, int skip) {
  int i;

  LOG_DEBUG_IO("(bits=%02x..., nb_bits=%d)", bits[0], nb_bits);
  for (i = skip; i < nb_bits; i++)
    axiblast_clock_tms((bits[i / 8] >> (i % 8)) & 0x01);
  axiblast_idle_clock();
}

/**
 * axiblast_tms - write a tms command
 * @param cmd tms command
 */
static void axiblast_tms(struct tms_command *cmd) {
  LOG_DEBUG_IO("(num_bits=%d)", cmd->num_bits);
  axiblast_tms_seq(cmd->bits, cmd->num_bits, 0);
}

/**
 * axiblast_path_move - write a TMS sequence transition to JTAG
 * @param cmd path transition
 *
 * Write a series of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=\<new_state>, TDI=\<???>
 *  - writing out TCK=1, TMS=\<new_state>, TDI=\<???> which triggers the
 transition
 * The function ensures that at the end of the sequence, the clock (TCK) is
 put
 * low.
 */
static void axiblast_path_move(struct pathmove_command *cmd) {
  int i;

  LOG_DEBUG_IO("(num_states=%d, last_state=%d)", cmd->num_states,
               cmd->path[cmd->num_states - 1]);
  for (i = 0; i < cmd->num_states; i++) {
    if (tap_state_transition(tap_get_state(), false) == cmd->path[i])
      axiblast_clock_tms(0);
    if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
      axiblast_clock_tms(1);
    tap_set_state(cmd->path[i]);
  }
  axiblast_idle_clock();
}

/**
 * axiblast_state_move - move JTAG state to the target state
 * @param state the target state
 * @param skip number of bits to skip at the beginning of the path
 *
 * Input the correct TMS sequence to the JTAG TAP so that we end up in the
 * target state. This assumes the current state (tap_get_state()) is correct.
 */
static void axiblast_state_move(tap_state_t state, int skip) {
  uint8_t tms_scan;
  int tms_len;

  LOG_DEBUG_IO("(from %s to %s)", tap_state_name(tap_get_state()),
               tap_state_name(state));
  if (tap_get_state() == state) return;
  tms_scan = tap_get_tms_path(tap_get_state(), state);
  tms_len = tap_get_tms_path_len(tap_get_state(), state);
  axiblast_tms_seq(&tms_scan, tms_len, skip);
  tap_set_state(state);
}

/**
 * axiblast_read_byteshifted_tdos - read TDO of byteshift writes
 * @param buf the buffer to store the bits
 * @param nb_bytes the number of bytes
 *
 * Reads back from USB Blaster TDO bits, triggered by a 'byteshift write',
 ie. eight
 * bits per received byte from USB interface, and store them in buffer.
 *
 * As the USB blaster stores the TDO bits in LSB (ie. first bit in (byte0,
 * bit0), second bit in (byte0, bit1), ...), which is what we want to return,
 * simply read bytes from USB interface and store them.
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read error occurred
 */
static int axiblast_read_byteshifted_tdos(uint8_t *buf, int nb_bytes) {
  uint32_t retlen;
  int ret = ERROR_OK;

  LOG_DEBUG_IO("%s(buf=%p, num_bits=%d)", __func__, buf, nb_bytes * 8);
  axiblast_flush_buffer();
  while (ret == ERROR_OK && nb_bytes > 0) {
    ret = axiblast_buf_read(buf, nb_bytes, &retlen);
    nb_bytes -= retlen;
  }
  return ret;
}

/**
 * axiblast_read_bitbang_tdos - read TDO of bitbang writes
 * @param buf the buffer to store the bits
 * @param nb_bits the number of bits
 *
 * Reads back from USB Blaster TDO bits, triggered by a 'bitbang write', ie.
 one
 * bit per received byte from USB interface, and store them in buffer, where
 :
 *  - first bit is stored in byte0, bit0 (LSB)
 *  - second bit is stored in byte0, bit 1
 *  ...
 *  - eight bit is stored in byte0, bit 7
 *  - ninth bit is stored in byte1, bit 0
 *  - etc ...
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read error occurred
 */
static int axiblast_read_bitbang_tdos(uint8_t *buf, int nb_bits) {
  int nb1 = nb_bits;
  int i, ret = ERROR_OK;
  uint32_t retlen;
  uint8_t tmp[8];

  LOG_DEBUG_IO("%s(buf=%p, num_bits=%d)", __func__, buf, nb_bits);

  /*
   * Ensure all previous bitbang writes were issued to the dongle, so that
   * it returns back the read values.
   */
  axiblast_flush_buffer();

  ret = axiblast_buf_read(tmp, nb1, &retlen);
  for (i = 0; ret == ERROR_OK && i < nb1; i++)
    if (tmp[i] & READ_TDO)
      *buf |= (1 << i);
    else
      *buf &= ~(1 << i);
  return ret;
}

/**
 * axiblast_queue_tdi - short description
 * @param bits bits to be queued on TDI (or NULL if 0 are to be queued)
 * @param nb_bits number of bits
 * @param scan scan type (ie. if TDO read back is required or not)
 *
 * Outputs a series of TDI bits on TDI.
 * As a side effect, the last TDI bit is sent along a TMS=1, and triggers a
 JTAG
 * TAP state shift if input bits were non NULL.
 *
 * In order to not saturate the USB Blaster queues, this method reads back
 TDO
 * if the scan type requests it, and stores them back in bits.
 *
 * As a side note, the state of TCK when entering this function *must* be
 * low. This is because byteshift mode outputs TDI on rising TCK and reads
 TDO
 * on falling TCK if and only if TCK is low before queuing byteshift mode
 bytes.
 * If TCK was high, the USB blaster will queue TDI on falling edge, and read
 TDO
 * on rising edge !!!
 */
static void axiblast_queue_tdi(uint8_t *bits, int nb_bits,
                               enum scan_type scan) {
  int nb8 = nb_bits / 8;
  int nb1 = nb_bits % 8;
  int nbfree_in_packet, i, trans = 0, read_tdos;
  uint8_t *tdos = calloc(1, nb_bits / 8 + 1);
  static uint8_t byte0[BUF_LEN];

  /*
   * As the last TDI bit should always be output in bitbang mode in order
   * to activate the TMS=1 transition to EXIT_?R state. Therefore a
   * situation where nb_bits is a multiple of 8 is handled as follows:
   * - the number of TDI shifted out in "byteshift mode" is 8 less than
   *   nb_bits
   * - nb1 = 8
   * This ensures that nb1 is never 0, and allows the TMS transition.
   */
  if (nb8 > 0 && nb1 == 0) {
    nb8--;
    nb1 = 8;
  }

  read_tdos = (scan == SCAN_IN || scan == SCAN_IO);
  for (i = 0; i < nb8; i += trans) {
    /*
     * Calculate number of bytes to fill USB packet of size
MAX_PACKET_SIZE
     */
    nbfree_in_packet = (MAX_PACKET_SIZE - (info.bufidx % MAX_PACKET_SIZE));
    trans = MIN(nbfree_in_packet - 1, nb8 - i);

    /*
     * Queue a byte-shift mode transmission, with as many bytes as
     * is possible with regard to :
     *  - current filling level of write buffer
     *  - remaining bytes to write in byte-shift mode
     */
    if (read_tdos)
      axiblast_queue_byte(SHMODE | READ | trans);
    else
      axiblast_queue_byte(SHMODE | trans);
    if (bits)
      axiblast_queue_bytes(&bits[i], trans);
    else
      axiblast_queue_bytes(byte0, trans);
    if (read_tdos) {
      if (info.flags & COPY_TDO_BUFFER)
        axiblast_queue_byte(CMD_COPY_TDO_BUFFER);
      axiblast_read_byteshifted_tdos(&tdos[i], trans);
    }
  }

  /*
   * Queue the remaining TDI bits in bitbang mode.
   */
  for (i = 0; i < nb1; i++) {
    int tdi = bits ? bits[nb8 + i / 8] & (1 << i) : 0;
    if (bits && i == nb1 - 1)
      axiblast_clock_tdi_flip_tms(tdi, scan);
    else
      axiblast_clock_tdi(tdi, scan);
  }
  if (nb1 && read_tdos) {
    if (info.flags & COPY_TDO_BUFFER) axiblast_queue_byte(CMD_COPY_TDO_BUFFER);
    axiblast_read_bitbang_tdos(&tdos[nb8], nb1);
  }

  if (bits) memcpy(bits, tdos, DIV_ROUND_UP(nb_bits, 8));
  free(tdos);

  /*
   * Ensure clock is in lower state
   */
  axiblast_idle_clock();
}

static void axiblast_runtest(int cycles, tap_state_t state) {
  LOG_DEBUG_IO("%s(cycles=%i, end_state=%d)", __func__, cycles, state);

  axiblast_state_move(TAP_IDLE, 0);
  axiblast_queue_tdi(NULL, cycles, SCAN_OUT);
  axiblast_state_move(state, 0);
}

static void axiblast_stableclocks(int cycles) {
  LOG_DEBUG_IO("%s(cycles=%i)", __func__, cycles);
  axiblast_queue_tdi(NULL, cycles, SCAN_OUT);
}

/**
 * axiblast_scan - launches a DR-scan or IR-scan
 * @param cmd the command to launch
 *
 * Launch a JTAG IR-scan or DR-scan
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read/write error occurred.
 */
static int axiblast_scan(struct scan_command *cmd) {
  int scan_bits;
  uint8_t *buf = NULL;
  enum scan_type type;
  int ret = ERROR_OK;
  static const char *const type2str[] = {"", "SCAN_IN", "SCAN_OUT", "SCAN_IO"};
  char *log_buf = NULL;

  type = jtag_scan_type(cmd);
  scan_bits = jtag_build_buffer(cmd, &buf);

  if (cmd->ir_scan)
    axiblast_state_move(TAP_IRSHIFT, 0);
  else
    axiblast_state_move(TAP_DRSHIFT, 0);

  log_buf = hexdump(buf, DIV_ROUND_UP(scan_bits, 8));
  LOG_DEBUG_IO("%s(scan=%s, type=%s, bits=%d, buf=[%s], end_state=%d)",
               __func__, cmd->ir_scan ? "IRSCAN" : "DRSCAN", type2str[type],
               scan_bits, log_buf, cmd->end_state);
  free(log_buf);

  axiblast_queue_tdi(buf, scan_bits, type);

  ret = jtag_read_buffer(buf, cmd);
  free(buf);
  /*
   * axiblast_queue_tdi sends the last bit with TMS=1. We are therefore
   * already in Exit1-DR/IR and have to skip the first step on our way
   * to end_state.
   */
  axiblast_state_move(cmd->end_state, 1);
  return ret;
}

static void axiblast_usleep(int us) {
  LOG_DEBUG_IO("%s(us=%d)", __func__, us);
  jtag_sleep(us);
}

static void axiblast_initial_wipeout(void) {
  static uint8_t tms_reset = 0xff;
  uint8_t out_value;
  uint32_t retlen;
  int i;

  out_value = axiblast_build_out(SCAN_OUT);
  for (i = 0; i < BUF_LEN; i++) info.buf[i] = out_value | ((i % 2) ? TCK : 0);

  /*
   * Flush USB-Blaster queue fifos
   *  - empty the write FIFO (128 bytes)
   *  - empty the read FIFO (384 bytes)
   */
  axiblast_buf_write(info.buf, BUF_LEN, &retlen);
  /*
   * Put JTAG in RESET state (five 1 on TMS)
   */
  axiblast_tms_seq(&tms_reset, 5, 0);
  tap_set_state(TAP_RESET);
}

static int axiblast_execute_queue(void) {
  struct jtag_command *cmd;
  static int first_call = 1;
  int ret = ERROR_OK;

  if (first_call) {
    first_call--;
    axiblast_initial_wipeout();
  }

  for (cmd = jtag_command_queue; ret == ERROR_OK && cmd; cmd = cmd->next) {
    switch (cmd->type) {
      case JTAG_RESET:
        axiblast_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
        break;
      case JTAG_RUNTEST:
        axiblast_runtest(cmd->cmd.runtest->num_cycles,
                         cmd->cmd.runtest->end_state);
        break;
      case JTAG_STABLECLOCKS:
        axiblast_stableclocks(cmd->cmd.stableclocks->num_cycles);
        break;
      case JTAG_TLR_RESET:
        axiblast_state_move(cmd->cmd.statemove->end_state, 0);
        break;
      case JTAG_PATHMOVE:
        axiblast_path_move(cmd->cmd.pathmove);
        break;
      case JTAG_TMS:
        axiblast_tms(cmd->cmd.tms);
        break;
      case JTAG_SLEEP:
        axiblast_usleep(cmd->cmd.sleep->us);
        break;
      case JTAG_SCAN:
        ret = axiblast_scan(cmd->cmd.scan);
        break;
      default:
        LOG_ERROR("BUG: unknown JTAG command type 0x%X", cmd->type);
        ret = ERROR_FAIL;
        break;
    }
  }

  axiblast_flush_buffer();
  return ret;
}

/*
驱动初始化函数，通过mmap函数获得axilite寄存器的虚拟映射地址，每次对axiBlaster进行读写仅仅操作这几个虚拟寄存器即可
*/
struct _addr_pair {
  unsigned int phy_addr;
  unsigned int *vaddr;
  int fd;
};

static int mapPhy2Vir(struct _addr_pair *addr) {
  void *map_base;
  //   map_base = (unsigned int*)mmap(0, sysconf(_SC_PAGESIZE), PROT_READ |
  //   PROT_WRITE, MAP_SHARED,
  //                   addr->fd, addr->phy_addr);
  // target & ~((typeof(target))pagesize - 1)
  unsigned int pagesize = (unsigned)sysconf(_SC_PAGESIZE);
  uint64_t target = addr->phy_addr;
  unsigned offset;
  printf("map: 0x%x ---> ", addr->phy_addr);
  map_base = (unsigned int *)mmap(0, sysconf(_SC_PAGESIZE),
                                  PROT_READ | PROT_WRITE, MAP_SHARED, addr->fd,
                                  target & ~((typeof(target))pagesize - 1));
  if (map_base == (void *)-1) {
    printf("Error mapping (%d) : %s\n", errno, strerror(errno));
    return -1;
  }
  offset = (unsigned int)(target & (pagesize - 1));
  addr->vaddr = (unsigned int *)map_base + offset;
  // printf("ok\n");
  printf("0x%p ok\n ", (unsigned int *)(addr->vaddr));
  return 0;
}
static void showVaddr(void){
  printf("clk phy:0x%x -> 0x%p\n", info.drv->axiblaster_phyClk_reg_addr, info.drv->vclk_addr);
  printf("full phy:0x%x -> 0x%p\n", info.drv->axiblaster_phyFifo_full_addr, info.drv->vfull_addr);
  printf("write phy:0x%x -> 0x%p\n", info.drv->axiblaster_phyFifo_write_addr, info.drv->vwrite_addr);
  printf("empty phy:0x%x -> 0x%p\n", info.drv->axiblaster_phyFifo_empty_addr, info.drv->vempty_addr);
  printf("read phy:0x%x -> 0x%p\n", info.drv->axiblaster_phyFifo_read_addr, info.drv->vread_addr);
}
static int axiblaster_init(void) {
  // unsigned int pagesize = (unsigned)sysconf(_SC_PAGESIZE);
  // uint64_t target;
  // unsigned offset;
  // info.drv->map_size = pagesize;
  LOG_INFO("%s: %s init function\n", "axiblaster", "reg addr");

  info.drv->fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (info.drv->fd == -1) {
    printf("Error opening /dev/mem (%d)  %s\n", errno, strerror(errno));
    return ERROR_FAIL;
  }

  struct _addr_pair addrPair;
  addrPair.fd = info.drv->fd;
  // map base reg addr

  addrPair.phy_addr = info.drv->axiblaster_phyreg_base;
  if (mapPhy2Vir(&addrPair) != 0) {
    return ERROR_FAIL;
  }
  info.drv->vclk_addr = addrPair.vaddr;
  info.drv->vfull_addr = addrPair.vaddr + 1;
  info.drv->vwrite_addr = addrPair.vaddr + 2;
  info.drv->vempty_addr = addrPair.vaddr + 3;
  info.drv->vread_addr = addrPair.vaddr + 4;
  showVaddr();

  unsigned int tmp = *(info.drv->vclk_addr);
	*(info.drv->vclk_addr) = 0x55aa0000;
	tmp = tmp&0x0000ffff;
	jtag_sleep(10);
	*(info.drv->vclk_addr) = tmp;
		// reset openjtag end
  // *(info.drv->vclk_addr) = 100; //set a slow clk first;
  axiblast_setSpeed(100);
  //info.drv->axiSpeed(info.drv, info.drv->axiblaster_jtagclk);

  tap_set_state(TAP_RESET);
  return ERROR_OK;
}

static int axiblaster_quit(void) {
  if (munmap(info.drv->vBase_addr, sysconf(_SC_PAGESIZE)) != 0) {
    printf("ERROR munmap (%d) %s\n", errno, strerror(errno));
  }
  // if (munmap(info.drv->vclk_addr, info.drv->map_size) != 0) {
  //   printf("ERROR munmap (%d) %s\n", errno, strerror(errno));
  // }
  // if (munmap(info.drv->vfull_addr, info.drv->map_size) != 0) {
  //   printf("ERROR munmap (%d) %s\n", errno, strerror(errno));
  // }
  // if (munmap(info.drv->vwrite_addr, info.drv->map_size) != 0) {
  //   printf("ERROR munmap (%d) %s\n", errno, strerror(errno));
  // }
  // if (munmap(info.drv->vempty_addr, info.drv->map_size) != 0) {
  //   printf("ERROR munmap (%d) %s\n", errno, strerror(errno));
  // }
  // if (munmap(info.drv->vread_addr, info.drv->map_size) != 0) {
  //   printf("ERROR munmap (%d) %s\n", errno, strerror(errno));
  // }
  close(info.drv->fd);
  return ERROR_OK;
}

// static int axiblaster_speed(int speed) {
//   info.drv->axiblaster_jtagclk = speed;
//   return ERROR_OK;
// }

// static int axiblaster_khz(int khz, int *jtag_speed)
// {
// 	if (!khz) {
// 		LOG_DEBUG("RCLK not supported");
// 		return ERROR_FAIL;
// 	}
// 	*jtag_speed = (int)(50000000/khz);
// 	if (*jtag_speed < 0)
// 		*jtag_speed = 0;
// 	return ERROR_OK;
// }

COMMAND_HANDLER(axiblast_handle_register_addr_command) {
  if (CMD_ARGC != 5) {
    LOG_WARNING(
        "please input correct reg addr"
        "(must be 5 addrs)");
    return ERROR_COMMAND_SYNTAX_ERROR;
  }

  COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], info.drv->axiblaster_phyClk_reg_addr);
  COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1],
                       info.drv->axiblaster_phyFifo_full_addr);
  COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2],
                       info.drv->axiblaster_phyFifo_write_addr);
  COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3],
                       info.drv->axiblaster_phyFifo_empty_addr);
  COMMAND_PARSE_NUMBER(u32, CMD_ARGV[4],
                       info.drv->axiblaster_phyFifo_read_addr);
  info.drv->axiblaster_phyreg_base = info.drv->axiblaster_phyClk_reg_addr;
  LOG_INFO("%s: %s is set\n", CMD_NAME, "reg addr");
  return ERROR_OK;
}

static const struct command_registration axiblast_subcommand_handlers[] = {
    {
        .name = "axiBlaster_register_addr",
        .handler = axiblast_handle_register_addr_command,
        .mode = COMMAND_CONFIG,
        .help = "input axiBlaster reg addr "
                "reg: clk, full, write, empty, read ",
        .usage = "axiBlaster_register_addr: 0,1,2,3,4",
    },
    COMMAND_REGISTRATION_DONE};

static const struct command_registration axiblast_command_handlers[] = {
    {
        .name = "axi_blaster",
        .mode = COMMAND_ANY,
        .help = "perform axi_blaster management",
        .chain = axiblast_subcommand_handlers,
        .usage = "",
    },
    COMMAND_REGISTRATION_DONE};

static struct jtag_interface axi_blaster_interface = {
    .supported = DEBUG_CAP_TMS_SEQ,
    .execute_queue = axiblast_execute_queue,
};

struct adapter_driver axi_blaster_adapter_driver = {
    .name = "axi_blaster",
    .transports = jtag_only,
    .commands = axiblast_command_handlers,

    .init = axiblaster_init,
    .quit = axiblaster_quit,

    .speed = axiblast_setSpeed,
    .khz = axiblast_khz,
    .speed_div = axiblast_speed_div,

    .jtag_ops = &axi_blaster_interface,
};