/***************************************************************************
 *   Copyright (C) 2016-2017 by Nuvoton                                    *
 *   Zale Yu <cyyu@nuvoton.com>                                            *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include "libusb_common.h"

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#define ENDPOINT_IN  0x80
#define ENDPOINT_OUT 0x00

#define NULINK_WRITE_TIMEOUT 1000
#define NULINK_READ_TIMEOUT  1000

#define NULINK_INTERFACE_NUM  0
#define NULINK2_INTERFACE_NUM 3

#define NULINK_RX_EP  (1|ENDPOINT_IN)
#define NULINK_TX_EP  (2|ENDPOINT_OUT)
#define NULINK2_RX_EP (6|ENDPOINT_IN)
#define NULINK2_TX_EP (7|ENDPOINT_OUT)

#define NULINK_HID_MAX_SIZE    (64)
#define NULINK2_HID_MAX_SIZE   (1024)
#define V6M_MAX_COMMAND_LENGTH (NULINK_HID_MAX_SIZE - 2)
#define V7M_MAX_COMMAND_LENGTH (NULINK_HID_MAX_SIZE - 3)

#define USBCMD_TIMEOUT		5000

//#define SHOW_BUFFER

struct nulink_usb_handle_s {
	struct jtag_libusb_device_handle *fd;
	struct libusb_transfer *trans;
	uint8_t interface_num;
	uint8_t rx_ep;
	uint8_t tx_ep;
	uint16_t max_packet_size;
	uint32_t usbcmdidx;
	uint8_t cmdidx;
	uint8_t cmdbuf[NULINK2_HID_MAX_SIZE];
	uint8_t tempbuf[NULINK2_HID_MAX_SIZE];
	uint8_t databuf[NULINK2_HID_MAX_SIZE];
	uint32_t max_mem_packet;
	enum hl_transports transport;
	uint16_t hardwareConfig; /* bit 0: 1:Nu-Link-Pro, 0:Normal Nu-Link | bit 1: 1:Nu-Link2, 0:Nu-Link */
	uint32_t reset_command;
} *m_nulink_usb_handle;

struct nulink_usb_internal_api_s {
	int (*nulink_usb_xfer) (void *handle, uint8_t *buf, int size);
	void (*nulink_usb_init_buffer) (void *handle, uint32_t size);
} m_nulink_usb_api;

//ICE Command
#define CMD_WRITE_REG				0xB8UL
#define CMD_WRITE_RAM				0xB9UL
#define CMD_CHECK_ID				0xA3UL
#define CMD_MCU_RESET				0xE2UL
#define CMD_CHECK_MCU_STOP			0xD8UL
#define CMD_MCU_STEP_RUN			0xD1UL
#define CMD_MCU_STOP_RUN			0xD2UL
#define CMD_MCU_FREE_RUN			0xD3UL
#define CMD_SET_CONFIG				0xA2UL
#define CMD_ERASE_FLASHCHIP         0xA4UL
#define ARM_SRAM_BASE				0x20000000

#define HARDWARECONFIG_NULINKPRO    1
#define HARDWARECONFIG_NULINK2      2

enum PROCESSOR_STATE_E {
	PROCESSOR_STOP,
	PROCESSOR_RUN,
	PROCESSOR_IDLE,
	PROCESSOR_POWER_DOWN
};

enum RESET_E
{
		RESET_AUTO			= 0,
		RESET_HW			= 1,
		RESET_SYSRESETREQ	= 2,
		RESET_VECTRESET		= 3,
		RESET_FAST_RESCUE	= 4,	/* Rescue and erase the chip, need very fast speed */
		RESET_NONE_NULINK	= 5,	/* Connect only */
		RESET_NONE2			= 6		/* For 8051 1T */
	};

enum CONNECT_E {
	CONNECT_NORMAL = 0,      /* Support all reset method */
	CONNECT_PRE_RESET = 1,   /* Support all reset method */
	CONNECT_UNDER_RESET = 2, /* Support all reset method */
	CONNECT_NONE = 3,        /* Support RESET_HW, (RESET_AUTO = RESET_HW) */
	CONNECT_DISCONNECT = 4,  /* Support RESET_NONE, (RESET_AUTO = RESET_NONE) */
	CONNECT_ICP_MODE = 5     /* Support NUC505 ICP mode*/
};

#ifdef SHOW_BUFFER
static void print64BytesBufferContent(char *bufferName, uint8_t *buf, int size)
{
	unsigned i, j;
	LOG_DEBUG("%s:", bufferName);

	for (i = 0; i < 4; i++) {
		j = i * 16;
		LOG_DEBUG("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ",
		buf[j + 0],  buf[j + 1],  buf[j + 2],  buf[j + 3],
		buf[j + 4],  buf[j + 5],  buf[j + 6],  buf[j + 7],
		buf[j + 8],  buf[j + 9],  buf[j + 10], buf[j + 11],
		buf[j + 12], buf[j + 13], buf[j + 14], buf[j + 15]
		);
	}
}
#endif

#ifndef _WIN32
double GetTickCount(void)
{
	struct timespec now;
	if (clock_gettime(CLOCK_MONOTONIC, &now))
		return 0;
	return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}
#endif

static void nulink_usb_init_buffer(void *handle, uint32_t size);

static int nulink_usb_xfer_rw(void *handle, int cmdsize, uint8_t *buf)
{
	struct nulink_usb_handle_s *h = handle;
	int startTime = GetTickCount(), cmdID;
	assert(handle != NULL);
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_lock();
#endif
	jtag_libusb_interrupt_write(h->fd, h->tx_ep, (char *)h->cmdbuf, h->max_packet_size,
		NULINK_WRITE_TIMEOUT);
#ifdef SHOW_BUFFER
	char bufName[20] = "cmd transferred";
	print64BytesBufferContent(bufName, h->cmdbuf, h->max_packet_size);
#endif
	do {
		jtag_libusb_interrupt_read(h->fd, h->rx_ep, (char *)buf,
			h->max_packet_size, NULINK_READ_TIMEOUT);
#ifdef SHOW_BUFFER
		char bufName1[20] = "data received";
		print64BytesBufferContent(bufName1, buf, h->max_packet_size);
#endif
		if(GetTickCount() - startTime > USBCMD_TIMEOUT)
		{
			break;
		}
		cmdID = h->cmdbuf[2];
	} while ((h->cmdbuf[0] != (buf[0] & 0x7F)) ||
			(cmdsize != buf[1]) ||
			(cmdID != 0xff && cmdID != CMD_WRITE_REG && cmdID != CMD_WRITE_RAM &&
			 cmdID != CMD_CHECK_MCU_STOP  && cmdID != buf[2]));
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_unlock();
#endif
	return ERROR_OK;
}

static int nulink2_usb_xfer_rw(void *handle, int cmdsize, uint8_t *buf)
{
	struct nulink_usb_handle_s *h = handle;
	int startTime = GetTickCount(), cmdID;
	assert(handle != NULL);
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_lock();
#endif
	jtag_libusb_interrupt_write(h->fd, h->tx_ep, (char *)h->cmdbuf, h->max_packet_size,
		NULINK_WRITE_TIMEOUT);
#ifdef SHOW_BUFFER
	char bufName[20] = "cmd transferred";
	print64BytesBufferContent(bufName, h->cmdbuf, h->max_packet_size);
#endif
	do {
		jtag_libusb_interrupt_read(h->fd, h->rx_ep, (char *)buf,
			h->max_packet_size, NULINK_READ_TIMEOUT);
#ifdef SHOW_BUFFER
		char bufName1[20] = "data received";
		print64BytesBufferContent(bufName1, buf, h->max_packet_size);
#endif
		if(GetTickCount() - startTime > USBCMD_TIMEOUT)
		{
			break;
		}
		cmdID = h->cmdbuf[3];
	} while ((h->cmdbuf[0] != (buf[0] & 0x7F)) ||
			(cmdsize != (((int)buf[1]) << 8) + ((int)buf[2] & 0xFF)) ||
			(cmdID != 0xff && cmdID != CMD_WRITE_REG && cmdID != CMD_WRITE_RAM &&
			 cmdID != CMD_CHECK_MCU_STOP && cmdID != buf[3]));
#if defined(_WIN32) && (NUVOTON_CUSTOMIZED)
	jtag_libusb_nuvoton_mutex_unlock();
#endif
	return ERROR_OK;
}

static int nulink_usb_xfer(void *handle, uint8_t *buf, int size)
{
	int err;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	err = nulink_usb_xfer_rw(h, size, h->tempbuf);
	memcpy(buf, h->tempbuf + 2, V6M_MAX_COMMAND_LENGTH);

	return err;
}

static int nulink2_usb_xfer(void *handle, uint8_t *buf, int size)
{
	int err;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	err = nulink2_usb_xfer_rw(h, size, h->tempbuf);
	memcpy(buf, h->tempbuf + 3, V7M_MAX_COMMAND_LENGTH);

	return err;
}

static void nulink_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, h->max_packet_size);
	memset(h->tempbuf, 0, h->max_packet_size);
	memset(h->databuf, 0, h->max_packet_size);

	h->cmdbuf[0] = (char)(++h->usbcmdidx & (unsigned char)0x7F);
	h->cmdbuf[1] = (char)size;
	h->cmdidx += 2;
}

static void nulink2_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, h->max_packet_size);
	memset(h->tempbuf, 0, h->max_packet_size);
	memset(h->databuf, 0, h->max_packet_size);

	h->cmdbuf[0] = (char)(++h->usbcmdidx & (unsigned char)0x7F);
	h->cmdbuf[1] = (char)((size >> 8) & 0xFF);
	h->cmdbuf[2] = (char)(size & 0xFF);
	h->cmdidx += 3;
}

static int nulink_usb_version(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_version");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, V6M_MAX_COMMAND_LENGTH);

	memset(h->cmdbuf + h->cmdidx, 0xFF, V6M_MAX_COMMAND_LENGTH);
	h->cmdbuf[h->cmdidx + 4] = (char)0xA1; /* host_rev_num: 6561 */;
	h->cmdbuf[h->cmdidx + 5] = (char)0x19;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 5);

	if (res != ERROR_OK)
		return res;

	LOG_INFO("NULINK firmware_version(%d), product_id(0x%08x)",
		le_to_h_u32(h->databuf),
		le_to_h_u32(h->databuf + 4 * 1));

	bool isNulinkPro = ((le_to_h_u32(h->databuf + 4 * 2) & 1) ? true : false);

	if (isNulinkPro)
	{
		LOG_INFO("NULINK is Nu-Link-Pro, target_voltage_mv(%d), usb_voltage_mv(%d)",
			(int)(unsigned short)(le_to_h_u32(h->databuf + 4 * 3)),
			(int)(unsigned short)(le_to_h_u32(h->databuf + 4 * 3) >> 16));

		h->hardwareConfig = (h->hardwareConfig & ~(HARDWARECONFIG_NULINKPRO)) | HARDWARECONFIG_NULINKPRO;
	}
	else {
		LOG_DEBUG("NULINK is Normal Nu-Link");
	}

	return ERROR_OK;
}

static int nulink_usb_assert_srst(void *handle, int srst);

static int nulink_usb_idcode(void *handle, uint32_t *idcode)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_idcode");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_ID);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->databuf + 4 * 1);

	LOG_INFO("IDCODE: 0x%08"PRIX32, *idcode);

	return ERROR_OK;
}

static int nulink_usb_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_write_debug_reg(0x%08x): 0x%08x",
		addr,
		val);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, val);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	return res;
}

static int nulink_usb_trace_read(void *handle, uint8_t *buf, size_t *size)
{
	/* not supported*/
	LOG_DEBUG("nulink_usb_trace_read is not supported");

	return ERROR_OK;
}

static enum target_state nulink_usb_state(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	//LOG_DEBUG("nulink_usb_state");

	assert(handle != NULL);

	//if (h->reconnect_pending) {
	//	LOG_INFO("Previous state query failed, trying to reconnect");
	//	res = nulink_usb_mode_enter(handle, nulink_get_mode(h->transport));

	//	if (res != ERROR_OK)
	//		return TARGET_UNKNOWN;

	//	h->reconnect_pending = false;
	//}

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_MCU_STOP);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 3);

	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	//enum PROCESSOR_STATE_E eState = (PROCESSOR_STATE_E)le_to_h_u32(h->databuf + 4 * 2);

	if (le_to_h_u32(h->databuf + 4 * 2) == 0) {
		// LOG_DEBUG("NULINK  MCU is stopping");
		// LOG_DEBUG("NULINK  stop_pc(0x%08x)", le_to_h_u32(h->databuf + 4 * 1));
		return TARGET_HALTED;
	}
	else
	{
		//LOG_DEBUG("NULINK  MCU is running");
		return TARGET_RUNNING;
	}

	//h->reconnect_pending = true;

	return TARGET_UNKNOWN;
}

static int nulink_usb_assert_srst(void *handle, int srst)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_assert_srst");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	h->cmdidx += 4;
	/* set reset type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_SYSRESETREQ);
	h->cmdidx += 4;
	/* set connect type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
	h->cmdidx += 4;
	/* set extMode */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 1);

	return res;
}

static int nulink_usb_reset(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	switch (h->reset_command) {
		case RESET_AUTO:
			LOG_DEBUG("nulink_usb_reset: RESET_AUTO");
			break;
		case RESET_HW:
			LOG_DEBUG("nulink_usb_reset: RESET_HW");
			break;
		case RESET_SYSRESETREQ:
			LOG_DEBUG("nulink_usb_reset: RESET_SYSRESETREQ");
			break;
		case RESET_VECTRESET:
			LOG_DEBUG("nulink_usb_reset: RESET_VECTRESET");
			break;
		case RESET_FAST_RESCUE:
			LOG_DEBUG("nulink_usb_reset: RESET_FAST_RESCUE");
			break;
		case RESET_NONE_NULINK:
			LOG_DEBUG("nulink_usb_reset: RESET_NONE_NULINK");
			break;
		case RESET_NONE2:
			LOG_DEBUG("nulink_usb_reset: RESET_NONE2");
			break;
		default:
			LOG_DEBUG("nulink_usb_reset: not found");
			break;
	}

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	h->cmdidx += 4;
	/* set reset type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, h->reset_command);
	h->cmdidx += 4;
	/* set connect type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
	h->cmdidx += 4;
	/* set extMode */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 1);

	return res;
}

int nulink_usb_M2351_erase(void)
{
	int res = ERROR_FAIL;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;

	LOG_DEBUG("nulink_usb_M2351_erase");

	if (m_nulink_usb_handle != NULL) {
		// SET_CONFIG for M2351
		m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 6);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_SET_CONFIG);
		h->cmdidx += 4;
		/* set max SWD clock */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 1000);
		h->cmdidx += 4;
		/* chip type: NUC_CHIP_TYPE_M2351 */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0x321);
		h->cmdidx += 4;
		/* IO voltage */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 5000);
		h->cmdidx += 4;
		/* If supply voltage to target or not */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
		/* USB_FUNC_E: USB_FUNC_HID_BULK */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 2);
		h->cmdidx += 4;

		m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 3);

		// Erase whole chip
		m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 6);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_ERASE_FLASHCHIP);
		h->cmdidx += 4;
		/* set count */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
		/* set config 0 */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFF);
		h->cmdidx += 4;
		/* set config 1 */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFF);
		h->cmdidx += 4;
		/* set config 2 */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFF);
		h->cmdidx += 4;
		/* set config 3 */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFF);
		h->cmdidx += 4;

		res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 1);
	}
	else {
		LOG_DEBUG("m_nulink_usb_handle not found");
	}

	return res;
}

int nulink_usb_assert_reset(void)
{
	int res;
	struct nulink_usb_handle_s *h = m_nulink_usb_handle;

	LOG_DEBUG("nulink_usb_assert_reset");

	m_nulink_usb_api.nulink_usb_init_buffer(m_nulink_usb_handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	h->cmdidx += 4;
	/* set reset type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_SYSRESETREQ);
	h->cmdidx += 4;
	/* set connect type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
	h->cmdidx += 4;
	/* set extMode */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(m_nulink_usb_handle, h->databuf, 4 * 1);

	return res;
}

static int nulink_usb_run(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_run");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_FREE_RUN);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 1);

	return res;
}

static int nulink_usb_halt(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_halt");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STOP_RUN);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	LOG_DEBUG("NULINK  stop_pc(0x%08x)",
		le_to_h_u32(h->databuf + 4));

	return res;
}

static int nulink_usb_step(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_step");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STEP_RUN);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	LOG_DEBUG("NULINK pc(0x%08x)",
		le_to_h_u32(h->databuf + 4));

	return res;
}

static int nulink_usb_read_regs(void *handle)
{
	/* not supported*/
	LOG_DEBUG("nulink_usb_read_regs");

	return ERROR_OK;
}

static int nulink_usb_read_reg(void *handle, int num, uint32_t *val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	//LOG_DEBUG("nulink_usb_read_reg");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_REG);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0xFF;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, num);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	*val = le_to_h_u32(h->databuf + 4 * 1);

	//LOG_DEBUG("NULINK read_reg(%d): 0x%08x",
	//	num,
	//	le_to_h_u32(h->databuf + 4 * 1));

	return res;
}

static int nulink_usb_write_reg(void *handle, int num, uint32_t val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	//LOG_DEBUG("nulink_usb_write_reg");

	assert(handle != NULL);

	m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_REG);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, num);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, val);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
	h->cmdidx += 4;

	res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 2);

	//LOG_DEBUG("NULINK write_reg(%d): %d",
	//	num,
	//	val);

	return res;
}

static int nulink_usb_read_mem8(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	unsigned alignedAddr, offset = 0;
	uint32_t bytes_remaining = 4;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_read_mem8: addr(0x%08x), len(%d)", addr, len);

	assert(handle != NULL);

	/* check whether data is word aligned */
	if (addr % 4) {
		alignedAddr = addr / 4;
		alignedAddr = alignedAddr * 4;
		offset = addr - alignedAddr;
		LOG_DEBUG("nulink_usb_read_mem8: address dose not follow alignment. addr(0x%08x)/alignedAddr(0x%08x)/offset(%d)", addr, alignedAddr, offset);

		addr = alignedAddr;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		if (len < 4)
			count = 1;
		else // len == 4
			count = 2;

		m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0xFF;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
			h->cmdidx += 4;
			/* proceed to the next one  */
			addr += 4;
		}

		res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		/* fill in the output buffer */
		for (i = 0; i < count; i++) {
			if (i == 0)
				memcpy(buffer, h->databuf + 4 + offset, len);
			else
				memcpy(buffer + 2 * 1, h->databuf + 4 * (2 * 1 + 1), len - 2);
		}

		// LOG_DEBUG("NULINK read_ram8(0x%08x): 0x%08x",
			// addr - 4,
			// le_to_h_u32(buffer));

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_write_mem8(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	unsigned alignedAddr, offset = 0;
	uint32_t bytes_remaining = 12;
	uint32_t u32bufferData;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_write_mem8: addr(0x%08x), len(%d)", addr, len);

	assert(handle != NULL);

	/* check whether data is word aligned */
	if (addr % 4) {
		alignedAddr = addr / 4;
		alignedAddr = alignedAddr * 4;
		offset = addr - alignedAddr;
		LOG_DEBUG("nulink_usb_write_mem8: address dose not follow alignment. addr(0x%08x)/alignedAddr(0x%08x)/offset(%d)", addr, alignedAddr, offset);

		addr = alignedAddr;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		if (len + offset <= 4)
			count = 1;
		else
			count = 2;

		m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			u32bufferData = buf_get_u32(buffer, 0, len * 8);
			u32bufferData = (u32bufferData << offset * 8);
			h_u32_to_le(h->cmdbuf + h->cmdidx, u32bufferData);
			h->cmdidx += 4;
			/* u32Mask */
			if (i == 0) {
				if (offset == 0) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFFFF00", i);
					}
					else if (len == 2) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFF0000", i);
					}
					else { // len == 3
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF000000);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFF000000", i);
					}
				}
				else if (offset == 1) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF00FF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFF00FF", i);
					}
					else if (len == 2) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF0000FF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFF0000FF", i);
					}
					else { // len == 3
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x000000FF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0x000000FF", i);
					}
				}
				else if (offset == 2) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF00FFFF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFF00FFFF", i);
					}
					else { // len == 2
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x0000FFFF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0x0000FFFF", i);
					}
				}
				else { // offset == 3
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00FFFFFF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0x00FFFFFF", i);
					}
				}
			}
			else { // i == 1
				if (offset == 1) {
					// len == 4
					h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
					LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFFFF00", i);
				}
				else if (offset == 2) {
					if (len == 3) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFFFF00", i);
					}
					else { // len == 4
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFF0000", i);
					}
				}
				else { // offset == 3
					if (len == 2) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFFFF00", i);
					}
					else if (len == 3) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFF0000", i);
					}
					else { // len == 4
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF000000);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFF000000", i);
					}
				}
			}
			h->cmdidx += 4;

			// LOG_DEBUG("NULINK write_ram8(0x%08x): 0x%04x",
				// addr,
				// u32bufferData);

			/* proceed to the next one */
			addr += 4;
			buffer += 4;
		}

		res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_read_mem32(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	uint32_t bytes_remaining = 12;
	struct nulink_usb_handle_s *h = handle;

	//LOG_DEBUG("nulink_usb_read_mem32: addr(0x%08x), len(%d)", addr, len);

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_ERROR("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		count = bytes_remaining / 4;

		m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0xFF;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
			h->cmdidx += 4;
			/* proceed to the next one  */
			addr += 4;
		}

		res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		/* fill in the output buffer */
		for (i = 0; i < count; i++) {
			memcpy(buffer, h->databuf + 4 * (2 * i + 1), 4);
			buffer += 4;
			//LOG_DEBUG("NULINK read_ram(0x%08x): 0x%08x",
			//	addr - 4,
			//	le_to_h_u32(buffer - 4));
		}

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
	const uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	uint32_t bytes_remaining = 12;
	uint32_t u32bufferData;
	struct nulink_usb_handle_s *h = handle;

	//LOG_DEBUG("nulink_usb_write_mem32: addr(0x%08x), len(%d)", addr, len);

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_ERROR("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		count = bytes_remaining / 4;

		m_nulink_usb_api.nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = (unsigned char)0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			u32bufferData = buf_get_u32(buffer, 0, 32);
			h_u32_to_le(h->cmdbuf + h->cmdidx, u32bufferData);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
			h->cmdidx += 4;

			// LOG_DEBUG("NULINK write_ram(0x%08x): 0x%04x",
				// addr,
				// u32bufferData);

			/* proceed to the next one */
			addr += 4;
			buffer += 4;
		}

		res = m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static uint32_t nulink_max_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	uint32_t max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));

	if (max_tar_block == 0)
		max_tar_block = 4;

	//LOG_DEBUG("nulink_max_block_size: %d", max_tar_block);

	return max_tar_block;
}

static int nulink_usb_read_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct nulink_usb_handle_s *h = handle;

	//LOG_DEBUG("nulink_usb_read_mem: addr(%04x), size(%d), count(%d)", addr, size, count);

	/* calculate byte count */
	count *= size;

	while (count) {

		bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (bytes_remaining >= 4)
			size = 4;

		/* the nulink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the nulink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the nulink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {
				uint32_t head_bytes = 4 - (addr % 4);
				retval = nulink_usb_read_mem8(handle, addr, head_bytes, buffer);
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = nulink_usb_read_mem(handle, addr, 1, bytes_remaining, buffer);
			else
				retval = nulink_usb_read_mem32(handle, addr, bytes_remaining, buffer);
		} else
			retval = nulink_usb_read_mem8(handle, addr, bytes_remaining, buffer);

		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int nulink_usb_write_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct nulink_usb_handle_s *h = handle;
	extern char *m_target_name;

	//LOG_DEBUG("nulink_usb_read_mem: addr(%04x), size(%d), count(%d)", addr, size, count);

	if (addr < ARM_SRAM_BASE) {
		if (strcmp(m_target_name, "NUC505") != 0) {
			LOG_DEBUG("since the address is below ARM_SRAM_BASE, the Nuvoton %s chip does not support this kind of writing.", m_target_name);
			return retval;
		}
		else {
			LOG_DEBUG("although the address is below ARM_SRAM_BASE, the Nuvoton %s chip supports this kind of writing.", m_target_name);
		}
	}

	/* calculate byte count */
	count *= size;

	while (count) {
		bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (bytes_remaining >= 4)
			size = 4;

		/* the nulink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the nulink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the nulink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {
				uint32_t head_bytes = 4 - (addr % 4);
				retval = nulink_usb_write_mem8(handle, addr, head_bytes, buffer);
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = nulink_usb_write_mem(handle, addr, 1, bytes_remaining, buffer);
			else
				retval = nulink_usb_write_mem32(handle, addr, bytes_remaining, buffer);

		} else
			retval = nulink_usb_write_mem8(handle, addr, bytes_remaining, buffer);

		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int nulink_usb_override_target(const char *targetname)
{
	LOG_DEBUG("nulink_usb_override_target");

	return !strcmp(targetname, "cortex_m");
}

static int nulink_speed(void *handle, int khz, bool query)
{
	struct nulink_usb_handle_s *h = handle;
	unsigned long max_ice_clock = khz;

	LOG_DEBUG("nulink_speed: query(%d)", query);

	if (max_ice_clock > 12000)
	{
		max_ice_clock = 12000;
	}
	else if ((max_ice_clock == 3 * 512) || (max_ice_clock == 1500))
	{
		max_ice_clock = 1500;
	}
	else if (max_ice_clock >= 1000)
	{
		max_ice_clock = max_ice_clock / 1000 * 1000;
	}
	else
	{
		max_ice_clock = max_ice_clock / 100 * 100;
	}

	LOG_DEBUG("NULINK nulink_speed: %lu",
			max_ice_clock);

	if (!query) {
		m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 6);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_SET_CONFIG);
		h->cmdidx += 4;
		/* set max SWD clock */
		h_u32_to_le(h->cmdbuf + h->cmdidx, max_ice_clock);
		h->cmdidx += 4;
		/* chip type: NUC_CHIP_TYPE_GENERAL_V6M */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
		/* IO voltage */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 3300);
		h->cmdidx += 4;
		/* If supply voltage to target or not */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
		/* USB_FUNC_E: USB_FUNC_HID_BULK */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 2);
		h->cmdidx += 4;

		m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 3);

		LOG_DEBUG("nulink_speed: h->hardwareConfig(%d)", h->hardwareConfig);
		if (h->hardwareConfig & 1) {
			LOG_INFO("NULINK target_voltage_mv[0](%04x), target_voltage_mv[1](%04x), target_voltage_mv[2](%04x), if_target_power_supplied(%d)",
				le_to_h_u32(h->databuf + 4 * 1),
				le_to_h_u32(h->databuf + 4 * 1) >> 16,
				le_to_h_u32(h->databuf + 4 * 2),
				(le_to_h_u32(h->databuf + 4 * 2) >> 16) & 1
				);
		}
		/* wait for NUC505 IBR operations */
		busy_sleep(50);
	}

	return max_ice_clock;
}

static int nulink_usb_close(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_close");

	if (handle != NULL) {
		LOG_DEBUG("trying to disconnect with nulink");
		m_nulink_usb_api.nulink_usb_init_buffer(handle, 4 * 4);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
		h->cmdidx += 4;
		/* set reset type */
		h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_NONE_NULINK);
		h->cmdidx += 4;
		/* set connect type */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_DISCONNECT);
		h->cmdidx += 4;
		/* set extMode */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;

		m_nulink_usb_api.nulink_usb_xfer(handle, h->databuf, 4 * 1);
	}

	// if (h && h->fd)
		// jtag_libusb_close(h->fd);

	// free(h);

	return ERROR_OK;
}

static int nulink_usb_open(struct hl_interface_param_s *param, void **fd)
{
	int err, retry_count = 1, result = 0;
	struct nulink_usb_handle_s *h;

	LOG_DEBUG("nulink_usb_open");
	char buf[512];

	m_nulink_usb_handle = NULL;

	struct stat fileStat;
	err = stat("c:\\Program Files\\Nuvoton Tools\\OpenOCD\\bin\\NuLink.exe", &fileStat);
	LOG_DEBUG("Stat Case 1: %d", err);
	if(err >= 0) {
		sprintf(buf, "\"c:\\Program Files\\Nuvoton Tools\\OpenOCD\\bin\\NuLink.exe\" -o conflict");
		result = system(buf);
		LOG_DEBUG("Run NuLink.exe on Win32 (result: %d)", result);
		if (result == -46) {
			LOG_DEBUG("A conflict happened! (result: %d)", result);
			LOG_ERROR("The ICE has been used by another Nuvoton tool. OpenOCD cannot work with the ICE unless we close the connection between the ICE and Nuvoton tool.");
			return ERROR_FAIL;
		}
		else if (result == -6) {
			LOG_DEBUG("Cannot find a target chip! (result: %d)", result);
			LOG_ERROR("We cannot find any Nuvoton device. Please check the hardware connection.");
			LOG_ERROR("If the ICE is used by another Nuvoton tool, please close the connection between the ICE and Nuvoton tool.");
			return ERROR_FAIL;
		}
		else if (result == 0) {
			sprintf(buf, "start /b \"\" \"c:\\Program Files\\Nuvoton Tools\\OpenOCD\\bin\\NuLink.exe\" -o wait");
			result = system(buf);
			LOG_DEBUG("Wait NuLink.exe (result: %d)", result);
		}
	}
	else {
		err = stat("c:\\Program Files (x86)\\Nuvoton Tools\\OpenOCD\\bin\\NuLink.exe", &fileStat);
		LOG_DEBUG("Stat Case 2: %d", err);
		if(err >= 0) {
			sprintf(buf, "\"c:\\Program Files (x86)\\Nuvoton Tools\\OpenOCD\\bin\\NuLink.exe\" -o conflict");
			result = system(buf);
			LOG_DEBUG("Run NuLink.exe on Win64 (result: %d)", result);
			if (result == -46) {
				LOG_DEBUG("A conflict happened! (result: %d)", result);
				LOG_ERROR("The ICE has been used by another Nuvoton Tool. OpenOCD cannot work with the ICE unless we close the connection between the ICE and Nuvoton tool.");
				return ERROR_FAIL;
			}
			else if (result == -6) {
				LOG_DEBUG("Cannot find a target chip! (result: %d)", result);
				LOG_ERROR("We cannot find any Nuvoton device. Please check the hardware connection.");
				LOG_ERROR("If the ICE is used by another Nuvoton tool, please close the connection between the ICE and Nuvoton tool.");
				return ERROR_FAIL;
			}
			else if (result == 0) {
				sprintf(buf, "start /b \"\" \"c:\\Program Files (x86)\\Nuvoton Tools\\OpenOCD\\bin\\NuLink.exe\" -o wait");
				result = system(buf);
				LOG_DEBUG("Wait NuLink.exe (result: %d)", result);
			}
		}
		else {
			LOG_DEBUG("Skip running NuLink.exe");
		}
	}

	h = calloc(1, sizeof(struct nulink_usb_handle_s));

	if (h == 0) {
		LOG_ERROR("malloc failed");
		return ERROR_FAIL;
	}

	h->transport = param->transport;

	const uint16_t vids[] = { param->vid, 0 };
	const uint16_t pids[] = { param->pid, 0 };
	const uint16_t vid_nulink2[] = { 0x0416, 0x0416, 0 };
	const uint16_t pid_nulink2[] = { 0x5200, 0x5201, 0 };
	const char *serial = param->serial;

	if (param->vid != 0 && param->pid != 0) {
		LOG_DEBUG("transport: %d vid: 0x%04x pid: 0x%04x serial: %s",
			param->transport, param->vid, param->pid,
			param->serial ? param->serial : "");
	}

	do {
		/* get the Nu-Link version */
		if (jtag_libusb_open(vid_nulink2, pid_nulink2, serial, &h->fd) == ERROR_OK) {
			h->hardwareConfig = (h->hardwareConfig & ~(HARDWARECONFIG_NULINK2)) | HARDWARECONFIG_NULINK2;
			m_nulink_usb_api.nulink_usb_xfer = nulink2_usb_xfer;
			m_nulink_usb_api.nulink_usb_init_buffer = nulink2_usb_init_buffer;
			h->interface_num = NULINK2_INTERFACE_NUM;
			h->rx_ep = NULINK2_RX_EP;
			h->tx_ep = NULINK2_TX_EP;
			h->max_packet_size = jtag_libusb_get_maxPacketSize(h->fd, 0, h->interface_num);
			if (h->max_packet_size == (uint16_t)-1) {
				h->max_packet_size = NULINK2_HID_MAX_SIZE;
			}
			LOG_DEBUG("max_packet_size: %d", h->max_packet_size);
			LOG_INFO("NULINK is Nu-Link2");
		}
		else {
			if (jtag_libusb_open(param->vids, param->pids, serial, &h->fd) != ERROR_OK) {
				if (jtag_libusb_open(vids, pids, serial, &h->fd) != ERROR_OK) {
					LOG_ERROR("open failed");
					goto error_open;
				}
			}

			m_nulink_usb_api.nulink_usb_xfer = nulink_usb_xfer;
			m_nulink_usb_api.nulink_usb_init_buffer = nulink_usb_init_buffer;
			h->interface_num = NULINK_INTERFACE_NUM;
			h->rx_ep = NULINK_RX_EP;
			h->tx_ep = NULINK_TX_EP;
			h->max_packet_size = jtag_libusb_get_maxPacketSize(h->fd, 0, h->interface_num);
			if (h->max_packet_size == (uint16_t)-1) {
				h->max_packet_size = NULINK_HID_MAX_SIZE;
			}
			LOG_DEBUG("max_packet_size: %d", h->max_packet_size);
			LOG_INFO("NULINK is Nu-Link1");
		}

		LOG_DEBUG("jtag_libusb_open succeeded");

		jtag_libusb_set_configuration(h->fd, 0);

		err = jtag_libusb_detach_kernel_driver(h->fd, h->interface_num);
		if (err != ERROR_OK) {
			LOG_DEBUG("detach kernel driver failed(%d)", err);
			//goto error_open;
		}
		else {
			LOG_DEBUG("jtag_libusb_detach_kernel_driver succeeded");
		}

		err = jtag_libusb_claim_interface(h->fd, h->interface_num);
		if (err != ERROR_OK) {
			LOG_ERROR("claim interface failed(%d)", err);
			goto error_open;
		}
		else {
			LOG_DEBUG("jtag_libusb_claim_interface succeeded");
		}

		h->usbcmdidx = 0;
		h->hardwareConfig = 0;

		/* get the device version */
		err = nulink_usb_version(h);

		if (err == ERROR_OK) {
			break;
		}
		else {
			err = jtag_libusb_release_interface(h->fd, 0);
			if (err != ERROR_OK) {
				LOG_ERROR("release interface failed");
				goto error_open;
			}

			err = jtag_libusb_reset_device(h->fd);
			if (err != ERROR_OK) {
				LOG_ERROR("reset device failed");
				goto error_open;
			}

			jtag_libusb_close(h->fd);
			/*
			  Give the device one second to settle down and
			  reenumerate.
			 */
			usleep(1 * 1000 * 1000);
			retry_count--;
		}
	} while (1);

	/* SWD clock rate : 1MHz */
	nulink_speed(h, 1000, false);

	LOG_DEBUG("nulink_usb_open: we manually perform nulink_usb_reset");
	nulink_usb_write_debug_reg(h, 0xe000edf0, 0xa05f0001);
	// //nulink_usb_write_debug_reg(h, 0xe000edfc, 0x01000000); /* reset but not halt */
	// //nulink_usb_write_debug_reg(h, 0xe000ed0c, 0x05fa0004);
	h->reset_command = RESET_HW;
	nulink_usb_reset(h);
	h->reset_command = RESET_SYSRESETREQ;
	nulink_usb_reset(h);

	/* get cpuid, so we can determine the max page size
	* start with a safe default for Cortex-M0*/
	h->max_mem_packet = (1 << 10);

	uint8_t buffer[4];
	err = nulink_usb_read_mem32(h, CPUID, 4, buffer);
	if (err == ERROR_OK) {
		uint32_t cpuid = le_to_h_u32(buffer);
		int i;

		if (((cpuid >> 4) & 0xfff) == V8MBL_CPUID_PARTNO || ((cpuid >> 4) & 0xfff) == V8MML_CPUID_PARTNO) {
			i = 23;
		}
		else {
			i = (cpuid >> 4) & 0xf;
		}

		if (i == 4 || i == 3 || i == 23) {
			/* Cortex-M3/M4/M23 has 4096 bytes autoincrement range */
			h->max_mem_packet = (1 << 12);
		}
	}

	LOG_DEBUG("max page size: %" PRIu32, h->max_mem_packet);

	*fd = h;
	m_nulink_usb_handle = h;

	return ERROR_OK;

error_open:
	//nulink_usb_close(h);

	if (h && h->fd)
		jtag_libusb_close(h->fd);

	free(h);

	return ERROR_FAIL;
}

int nulink_config_trace(void *handle, bool enabled, enum tpio_pin_protocol pin_protocol,
			uint32_t port_size, unsigned int *trace_freq)
{
	/* not supported */
	LOG_DEBUG("nulink_config_trace");

	return ERROR_OK;
}

struct hl_layout_api_s nulink_usb_layout_api = {

	.open = nulink_usb_open,

	.close = nulink_usb_close,

	.idcode = nulink_usb_idcode,

	.state = nulink_usb_state,

	.reset = nulink_usb_reset,

	.assert_srst = nulink_usb_assert_srst,

	.run = nulink_usb_run,

	.halt = nulink_usb_halt,

	.step = nulink_usb_step,

	.read_regs = nulink_usb_read_regs,

	.read_reg = nulink_usb_read_reg,

	.write_reg = nulink_usb_write_reg,

	.read_mem = nulink_usb_read_mem,

	.write_mem = nulink_usb_write_mem,

	.write_debug_reg = nulink_usb_write_debug_reg,

	.override_target = nulink_usb_override_target,

	.speed = nulink_speed,

	.config_trace = nulink_config_trace,

	.poll_trace = nulink_usb_trace_read,
};
