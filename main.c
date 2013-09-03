/*
** Filename: main.c
**
** Automatically created by Application Wizard 1.4.2
**
** Part of solution BH-EU in project USB-Writeblocker
**
** Based on USBSlaveHIDKbd example by FTDI
** Some ideas also borrowed from examples from John Hyde's USB by Design
**
** Modifications by Philip A. Polstra, Sr.
**
** Part of the USB Writeblocker Solution as presented at Blackhat Europe 2012
**
** Comments:
**
** Important: Sections between markers "FTDI:S*" and "FTDI:E*" will be overwritten by
** the Application Wizard
*/

#include "stdlib.h"
#include "string.h"
#include "main.h"
#include "GPIO.h"

#include "USBSlaveBomsDrv.h"
#include "USBHostBomsDrv.h"

/* FTDI:STP Thread Prototypes */

vos_tcb_t *tcbHostIn, *tcbHostOut, *tcbHostEnum;

void handleCbw();
void hostEnum();

/* FTDI:ETP */

/* FTDI:SDH Driver Handles */
/* FTDI:EDH */

VOS_HANDLE hUSBHOST_2;
VOS_HANDLE hUSBHOSTBOMS;
VOS_HANDLE hUSBSLAVE_1;
VOS_HANDLE hUSBSLAVEBOMS;
VOS_HANDLE hGPIO_PORT_E;

unsigned short blockSize=512; // default to 512
unsigned long deviceCapacity;
unsigned short maxLuns=1; // default to 1

unsigned char illegalRequest=0; // this flag is set after an illegal request

extern usbSlaveBoms_context *slaveBomsCtx;
extern usbhostBoms_context_t *hostBomsCtx;

/* Declaration for IOMUx setup function */
void iomux_setup(void);

/* Main code - entry point to firmware */
void main(void)
{
	/* FTDI:SDD Driver Declarations */
	/* FTDI:EDD */
	gpio_context_t gpioContextE;

	// USB Host configuration context
	usbhost_context_t usbhostContext;

	/* FTDI:SKI Kernel Initialisation */
	vos_init(50, VOS_TICK_INTERVAL, VOS_NUMBER_DEVICES);
	vos_set_clock_frequency(VOS_48MHZ_CLOCK_FREQUENCY);
	vos_set_idle_thread_tcb_size(512);
	/* FTDI:EKI */

	iomux_setup();

	/* FTDI:SDI Driver Initialisation */

	// initialize Vinco USB Host
	gpioContextE.port_identifier = GPIO_PORT_E;
	gpio_init(VOS_DEV_GPIO_PORT_E, &gpioContextE);

	usbslave_init(0, VOS_DEV_USBSLAVE_1);

	usbslaveboms_init(VOS_DEV_USBSLAVEBOMS);

	// Initialise USB Host
	usbhostContext.if_count = 4;
	usbhostContext.ep_count = 8;
	usbhostContext.xfer_count = 4;
	usbhostContext.iso_xfer_count = 0;
	usbhost_init(-1, VOS_DEV_USBHOST_2, &usbhostContext);

	usbhostBoms_init(VOS_DEV_USBHOSTBOMS);

	/* FTDI:SCT Thread Creation */
	tcbHostIn = vos_create_thread_ex(27, 1024, handleCbw, "Handle_CBW", 0);
	tcbHostEnum = vos_create_thread_ex(28, 1024, hostEnum, "Host_Enum", 0);


	/* FTDI:ECT */

	vos_start_scheduler();

main_loop:
	goto main_loop;
}

/* FTDI:SSP Support Functions */
/* FTDI:ESP */

unsigned char usbhost_connect_state(VOS_HANDLE hUSB)
{
	unsigned char connectstate = PORT_STATE_DISCONNECTED;
	usbhost_ioctl_cb_t hc_iocb;

	if (hUSB)
	{
		hc_iocb.ioctl_code = VOS_IOCTL_USBHOST_GET_CONNECT_STATE;
		hc_iocb.get = &connectstate;
		vos_dev_ioctl(hUSB, &hc_iocb);
	}

	return connectstate;
}

void open_drivers(void)
{
	gpio_ioctl_cb_t gpio_iocb;
	unsigned char leds;

	/* Code for opening and closing drivers - move to required places in Application Threads */
	/* FTDI:SDA Driver Open */
	hGPIO_PORT_E = vos_dev_open(VOS_DEV_GPIO_PORT_E);

	// power up Vinco USB Host
	// this must happen before we want to enumerate the flash drive
	gpio_iocb.ioctl_code = VOS_IOCTL_GPIO_SET_MASK;
	gpio_iocb.value = 0x60;            // set power and LED as output
	vos_dev_ioctl(hGPIO_PORT_E, &gpio_iocb);
	leds = 0x00;
	vos_dev_write(hGPIO_PORT_E, &leds, 1, NULL);

	hUSBHOST_2 = vos_dev_open(VOS_DEV_USBHOST_2);

	/* FTDI:EDA */

	hUSBHOSTBOMS = vos_dev_open(VOS_DEV_USBHOSTBOMS);

	hUSBSLAVE_1 = vos_dev_open(VOS_DEV_USBSLAVE_1);

	hUSBSLAVEBOMS = vos_dev_open(VOS_DEV_USBSLAVEBOMS);

}

void attach_drivers(void)
{
	common_ioctl_cb_t bomsAttach;

	/* FTDI:SUA Layered Driver Attach Function Calls */
	/* FTDI:EUA */

	// attach BOMS to USB Host port B
	bomsAttach.ioctl_code = VOS_IOCTL_USBHOSTBOMS_ATTACH;
	bomsAttach.set.data = (void *) hUSBHOST_2;
	vos_dev_ioctl(hUSBHOSTBOMS, &bomsAttach);

	// attach BOMS to USB Slave port A
	bomsAttach.ioctl_code = VOS_IOCTL_USBSLAVEBOMS_ATTACH;
	bomsAttach.set.data = (void *) hUSBSLAVE_1;
	vos_dev_ioctl(hUSBSLAVEBOMS, &bomsAttach);

}

void close_drivers(void)
{

	vos_dev_close(hUSBHOST_2);
	vos_dev_close(hUSBHOSTBOMS);
	vos_dev_close(hUSBSLAVE_1);
	vos_dev_close(hUSBSLAVEBOMS);
	vos_dev_close(hGPIO_PORT_E);
}

/* Application Threads */

void hostEnum()
{

	unsigned char i;
	unsigned char status;
	unsigned char buf[64];
	unsigned short num_read;
	unsigned int handle;

	usbhostBoms_ioctl_t generic_iocb;
	usbhost_device_handle_ex ifDev;
	usbhost_ioctl_cb_t hc_iocb;
	usbhost_ioctl_cb_class_t hc_iocb_class;
	usbhostBoms_ioctl_cb_attach_t genericAtt;

	num_read = 0;  // just here to set breakpoint

	open_drivers();

	attach_drivers(); // enumerate flash drive then connect slave

	do
	{
		// see if bus available
		if (usbhost_connect_state(hUSBHOST_2) == PORT_STATE_ENUMERATED)
		{
				hc_iocb_class.dev_class = USB_CLASS_MASS_STORAGE;
				hc_iocb_class.dev_subclass = USB_SUBCLASS_MASS_STORAGE_SCSI;
				hc_iocb_class.dev_protocol = USB_PROTOCOL_MASS_STORAGE_BOMS;

				// user ioctl to find first hub device
				hc_iocb.ioctl_code = VOS_IOCTL_USBHOST_DEVICE_FIND_HANDLE_BY_CLASS;
				hc_iocb.handle.dif = NULL;
				hc_iocb.set = &hc_iocb_class;
				hc_iocb.get = &ifDev;

				if (vos_dev_ioctl(hUSBHOST_2, &hc_iocb) == USBHOST_OK)
				{
				}

				genericAtt.hc_handle = hUSBHOST_2;
				genericAtt.ifDev = ifDev;

				generic_iocb.ioctl_code = VOS_IOCTL_USBHOSTBOMS_ATTACH;
				generic_iocb.set.att = &genericAtt;

				// we use a simple variable to indicate if the flash drive
				// is attached
				// this is not as elegent as using a semaphore, but
				// this is the only thread that updates this variable and
				// if the device is disconnected and reconnected that is hard
				// to handle with a semaphore
				if (vos_dev_ioctl(hUSBHOSTBOMS, &generic_iocb) == USBHOSTBOMS_OK)
				{
					slaveBomsCtx->flashConnected = 1;
					vos_signal_semaphore(&slaveBomsCtx->enumed);
				} else
				{
					slaveBomsCtx->flashConnected = 0;
				}// if attach
				// this code is in here so that if the drive gets disconnected
				// we can try to restart it
				// also, hopefully the the traffice every few seconds will keep
				// the drive from going to sleep
				vos_delay_msecs(2000);
		} // if enumerated
			vos_delay_msecs(10); // recheck every .01 seconds for new connect
	} // outer do
	while (1);

}

unsigned short forward_cbw_to_device(boms_cbw_t *cbw)
{
	unsigned short num_written;
	usbhostBoms_write((void*)cbw, sizeof(boms_cbw_t), &num_written, hostBomsCtx);

	return num_written;
}

unsigned short receive_data_from_device(void* buffer, unsigned short expected)
{
	unsigned short num_read;
	unsigned char status;

	status = usbhostBoms_read(buffer, expected, &num_read, hostBomsCtx);
	if (status == USBHOST_EP_HALTED)
	{
		// the endpoint is halted so let's halt the slave endpoint
		usbslaveboms_stall_bulk_in(slaveBomsCtx);
	}

	return num_read;
}

unsigned short forward_data_to_slave(void* buffer, unsigned short bytes)
{
	unsigned short num_written;
	usbSlaveBoms_write(buffer, bytes, &num_written, slaveBomsCtx);

	return num_written;
}

unsigned short forward_data_to_slave_then_stall(void* buffer, unsigned short bytes)
{
	unsigned short num_written;
	usbSlaveBoms_short_write(buffer, bytes, &num_written, slaveBomsCtx);

	return num_written;
}

unsigned short receive_csw_from_device(boms_csw_t *csw)
{
	unsigned short num_read;
	usbhostBoms_read((void*)csw, 13, &num_read, hostBomsCtx);

	return num_read;
}

unsigned short forward_csw_to_slave(boms_csw_t *csw)
{
	unsigned short num_written;
	usbSlaveBoms_write((void*)csw, 13, &num_written, slaveBomsCtx);

	return num_written;
}

void handle_inquiry(boms_cbw_t *cbw)
{
	unsigned char buffer[64];
	unsigned short responseSize;
	boms_csw_t csw;

	// forward the CBW to device
	if (forward_cbw_to_device(cbw))
	{
		// receive response from device
		// note we will assume that only the standard 36 bytes will be requested
		if (responseSize = receive_data_from_device(&buffer[0], 36))
		{
			// forward response to slave
			forward_data_to_slave(&buffer[0], responseSize);

			// receive CSW from device
			if (receive_csw_from_device(&csw))
			{
				// forward CSW to slave
				forward_csw_to_slave(&csw);
			}
		}
	}
}

void handle_test_unit_ready(boms_cbw_t *cbw)
{
	boms_csw_t csw;

	// forward the CBW to device
	if (forward_cbw_to_device(cbw))
	{
		// receive response from device
		if (receive_csw_from_device(&csw))
		{
			// forward CSW to slave
			forward_csw_to_slave(&csw);
		}
	}
}

void handle_read(boms_cbw_t *cbw)
{
	// this same routine handles all 3 possible read commands
	// most likely read command is read(10)

	unsigned long lba; // logical block address for start block
	unsigned short blocks; // number of blocks to read
	unsigned short i;
	boms_csw_t csw;
	unsigned char *buffer;
	unsigned short num_read;
	unsigned short num_written;

	switch (cbw->cb.formated.command)
	{
		case BOMS_READ_6:
			lba = cbw->cb.raw[1]*65536 + cbw->cb.raw[2]*256 + cbw->cb.raw[3];
			blocks = cbw->cb.raw[4];
			break;
		case BOMS_READ_10:
			lba = cbw->cb.raw[2]*16777216 + cbw->cb.raw[3]*65536 + cbw->cb.raw[4]*256 +cbw->cb.raw[5];
			blocks = cbw->cb.raw[7] * 256 + cbw->cb.raw[8];
			break;
		case BOMS_READ_12:
			lba = cbw->cb.raw[2]*16777216 + cbw->cb.raw[3]*65536 + cbw->cb.raw[4]*256 +cbw->cb.raw[5];
			// we are being a little bad here the number of blocks is actually a long
			// it is extremely unlikely that anyone would request this much at once, however
			blocks = cbw->cb.raw[8] * 256 + cbw->cb.raw[9];
			break;
	}

	// now forward the cbw to the device
	forward_cbw_to_device(cbw);

	// receive the appropriate number of blocks from the device
	// forward the blocks to the slave
	// most requests are probably 1 block of 512 bytes
	// read in 512 byte chunks (packet size is 64 bytes, but VOS should handle this)
	// If devices with larger blocks are encountered, 512 should still work
	buffer = vos_malloc(blockSize);
	while(blocks>0)
	{
		usbhostBoms_read((void*)buffer, blockSize, &num_read, hostBomsCtx);
		usbSlaveBoms_write((void*)buffer, num_read, &num_written, slaveBomsCtx);
		blocks--;
	}
	vos_free(buffer);

	// receive the csw from the device
	receive_csw_from_device(&csw);

	// forward the csw to the slave
	forward_csw_to_slave(&csw);
}

void handle_read_capacity(boms_cbw_t *cbw)
{
	boms_csw_t csw;
	unsigned char buffer[8];
	unsigned short received;

	// forward cbw to device
	forward_cbw_to_device(cbw);

	// receive response from device
	if (received = receive_data_from_device(&buffer[0], 8))
	{
		deviceCapacity = buffer[0]*16777216 + buffer[1]*65536 + buffer[2]*256 +buffer[3];
		blockSize = buffer[4]*16777216 + buffer[5]*65536 + buffer[6]*256 +buffer[7];
		// forward response to slave
		forward_data_to_slave(&buffer[0], received);
	}

	// receive csw from device
	receive_csw_from_device(&csw);

	// forward csw to slave
	forward_csw_to_slave(&csw);
}

void handle_report_luns(boms_cbw_t *cbw)
{
	boms_csw_t csw;
	unsigned char buffer[64];
	unsigned short received;

	// forward cbw
	forward_cbw_to_device(cbw);

	// receive response from device
	// response is 8 bytes + maxLuns * 8
	if (received = receive_data_from_device(&buffer[0], 8 + 8 * maxLuns))
	{
		// forward response to slave
		forward_data_to_slave(&buffer[0], received);
	}

	// receive csw from device
	receive_csw_from_device(&csw);

	// forward csw to slave
	forward_csw_to_slave(&csw);
}

void handle_request_sense(boms_cbw_t *cbw)
{
	boms_csw_t csw;
	unsigned char bytesRequested;
	unsigned short bytesRead, bytesWritten;
	unsigned char *buffer;
	request_sense_response_t rsr;

	if (illegalRequest)
	{
		// if we are here this a a request sense that came right after an illegal
		// command request - perhaps somebody tried to modify our drive!
		// we return the appropriate error directly and a CSW
		// the device is never touched

		illegalRequest = 0; // reset so next request goes to actual device

		rsr.formated.responseCode =0x70; //0x70 current error
		rsr.formated.valid = 0; // 1=INFORMATION field valid
		rsr.formated.obsolete = 0;
		rsr.formated.senseKey = 0x05; // 0x05 for illegal request
		rsr.formated.resvered = 0;
		rsr.formated.ili = 0; // incorrect length indicator
		rsr.formated.eom = 0; // end of media for streaming devices
		rsr.formated.filemark = 0; // for streaming devices
		rsr.formated.information = 0; // device specific info
		rsr.formated.addSenseLen = 0x0a; // additional bytes that follow 244 max
		rsr.formated.cmdSpecInfo = 0; // command specific info
		rsr.formated.asc = 0x20; // additional sense code 0x20 for illegal command
		rsr.formated.ascq = 0; // additional sense code qualifier 0-unused
		rsr.formated.fruc = 0; // field replaceable unit code set to 0
		rsr.formated.senseKeySpecific[0] = 0; //senses key spec info if b7=1
		rsr.formated.senseKeySpecific[1] = 0;
		rsr.formated.senseKeySpecific[2] = 0;
		bytesWritten = forward_data_to_slave(&rsr, 18);

		// now send an appropriate CSW to indicate success of this command
		csw.sig[0] = 'U'; //"USBS"
		csw.sig[1] = 'S';
		csw.sig[2] = 'B';
		csw.sig[3] = 'S';
		csw.tag = cbw->tag;
		csw.residue = 0;
		csw.status = 0; // 0x00=success 0x01=failure 0x02=phase error
		forward_csw_to_slave(&csw);
	} else
	{
		// forward cbw
		bytesRequested = cbw->cb.raw[4];
		forward_cbw_to_device(cbw);
		buffer = vos_malloc((unsigned short)bytesRequested);

		// receive data from device
		if (bytesRead = receive_data_from_device(buffer, (unsigned short)bytesRequested))
		{
			bytesWritten = forward_data_to_slave(buffer, bytesRead);
		}

		vos_free(buffer);
		// receive csw from device
		receive_csw_from_device(&csw);

		// forward csw to slave
		forward_csw_to_slave(&csw);
	}
}

void handle_mode_sense(boms_cbw_t *cbw)
{
	boms_csw_t csw;
	unsigned short allocLength=0;
	unsigned char *buffer=NULL;
	unsigned short bytesReceived=0;

	// forward the cbw to the device
	switch (cbw->cb.formated.command)
	{
		case BOMS_MODE_SENSE_6:
			allocLength = cbw->cb.raw[4];
			break;
		case BOMS_MODE_SENSE_10:
			allocLength = cbw->cb.raw[7]*256 + cbw->cb.raw[8];
			break;
	}
	forward_cbw_to_device(cbw);

	// receive data from device
	if (allocLength)
	{
		buffer = vos_malloc(allocLength);
		bytesReceived = receive_data_from_device(buffer, allocLength);
		// forward data to slave
		forward_data_to_slave(buffer, bytesReceived);
		vos_free(buffer);
	}

	// receive csw from device
	receive_csw_from_device(&csw);

	// forward csw to slave
	forward_csw_to_slave(&csw);
}

void handle_mode_select(boms_cbw_t *cbw)
{
	boms_csw_t csw;
	unsigned short allocLength=0;
	unsigned char *buffer=NULL;
	unsigned short bytesReceived=0;

	// forward the cbw to the device
	switch (cbw->cb.formated.command)
	{
		case BOMS_MODE_SELECT_6:
			allocLength = cbw->cb.raw[4];
			break;
		case BOMS_MODE_SELECT_10:
			allocLength = cbw->cb.raw[7]*256 + cbw->cb.raw[8];
			break;
	}
	forward_cbw_to_device(cbw);

	// receive data from device
	if (allocLength)
	{
		buffer = vos_malloc(allocLength);
		bytesReceived = receive_data_from_device(buffer, allocLength);
		// forward data to slave
		forward_data_to_slave(buffer, bytesReceived);
		vos_free(buffer);
	}

	// receive csw from device
	receive_csw_from_device(&csw);

	// forward csw to slave
	forward_csw_to_slave(&csw);
}

void handle_illegal_request(boms_cbw_t *cbw)
{
	usbslave_ioctl_cb_t iocb;
	boms_csw_t csw;

	// now send the CSW
	csw.sig[0]='U';
	csw.sig[1]='S';
	csw.sig[2]='B';
	csw.sig[3]='S';//"USBS"
	csw.tag=cbw->tag;
	csw.residue=0;
	csw.status=0x01; // 0x00=success 0x01=failure 0x02=phase error
	forward_csw_to_slave(&csw);

	// flag the error for the anticipated call to REQUEST SENSE
	illegalRequest=1;
}

void handle_illegal_write_request(boms_cbw_t *cbw)
{
	usbslave_ioctl_cb_t iocb;
	boms_csw_t csw;
	unsigned short blocks;
	unsigned char *buffer;
	unsigned short num_read;
	unsigned short i;


	// as strange as it may seem, there is no way to tell the host to quit
	// instead we need to receive all this data and throw it away!
	switch (cbw->cb.formated.command)
	{
		case BOMS_WRITE_6:
			blocks = cbw->cb.raw[4];
			break;
		case BOMS_WRITE_10:
			blocks = cbw->cb.raw[7] * 256 + cbw->cb.raw[8];
			break;
		case BOMS_WRITE_12:
			// we are being a little bad here the number of blocks is actually a long
			// it is extremely unlikely that anyone would request this much at once, however
			blocks = cbw->cb.raw[8] * 256 + cbw->cb.raw[9];
			break;
	}

	buffer = vos_malloc(512);
	iocb.ioctl_code = VOS_IOCTL_USBSLAVE_TRANSFER;
	iocb.handle = slaveBomsCtx->out_ep;
	iocb.request.setup_or_bulk_transfer.buffer = buffer;
	iocb.request.setup_or_bulk_transfer.size = 512;
	iocb.request.setup_or_bulk_transfer.bytes_transferred = 0;
	for (i = 0; i < (blocks * (512/blockSize)); i++)
	{
		// process bytes received from host
		vos_dev_ioctl(slaveBomsCtx->handle,&iocb);
	}
	vos_free(buffer);

	// now send the CSW
	csw.sig[0]='U';
	csw.sig[1]='S';
	csw.sig[2]='B';
	csw.sig[3]='S';//"USBS"
	csw.tag=cbw->tag;
	csw.residue=0;
	csw.status=0x00; // 0x00=success 0x01=failure 0x02=phase error
	//forward_csw_to_slave(&csw);
	iocb.ioctl_code = VOS_IOCTL_USBSLAVE_TRANSFER;
	iocb.handle = slaveBomsCtx->in_ep;
	iocb.request.setup_or_bulk_transfer.buffer = &csw;
	iocb.request.setup_or_bulk_transfer.size = sizeof(boms_csw_t);
	vos_dev_ioctl(slaveBomsCtx->handle, &iocb);

}

void handle_send_diagnostic(boms_cbw_t *cbw)
{
	usbslave_ioctl_cb_t iocb;
	boms_csw_t csw;

	// first send ZLDP to ACK the command
	iocb.ioctl_code = VOS_IOCTL_USBSLAVE_TRANSFER;
	iocb.handle = slaveBomsCtx->in_ep;
	iocb.request.setup_or_bulk_transfer.buffer = NULL;
	iocb.request.setup_or_bulk_transfer.size = 0;
	vos_dev_ioctl(slaveBomsCtx->handle, &iocb);

	// now send the CSW
	csw.sig[0]='U';
	csw.sig[1]='S';
	csw.sig[2]='B';
	csw.sig[3]='S';//"USBS"
	csw.tag=cbw->tag;
	csw.residue=0;
	csw.status=0x00; // 0x00=success 0x01=failure 0x02=phase error
	forward_csw_to_slave(&csw);

}

void handle_start_stop_unit(boms_cbw_t *cbw)
{
	boms_csw_t csw;

	// forward the CBW to device
	if (forward_cbw_to_device(cbw))
	{
		// receive response from device
		if (receive_csw_from_device(&csw))
		{
			// forward CSW to slave
			forward_csw_to_slave(&csw);
		}
	}

}

void handle_synchronize_cache(boms_cbw_t *cbw)
{
	usbslave_ioctl_cb_t iocb;
	boms_csw_t csw;

	// first send ZLDP to ACK the command
	iocb.ioctl_code = VOS_IOCTL_USBSLAVE_TRANSFER;
	iocb.handle = slaveBomsCtx->in_ep;
	iocb.request.setup_or_bulk_transfer.buffer = NULL;
	iocb.request.setup_or_bulk_transfer.size = 0;
	vos_dev_ioctl(slaveBomsCtx->handle, &iocb);

	// now send the CSW
	csw.sig[0]='U';
	csw.sig[1]='S';
	csw.sig[2]='B';
	csw.sig[3]='S';//"USBS"
	csw.tag=cbw->tag;
	csw.residue=0;
	csw.status=0x00; // 0x00=success 0x01=failure 0x02=phase error
	forward_csw_to_slave(&csw);

}

void handle_read_format_capacities(boms_cbw_t *cbw)
{
	unsigned char *buffer;
	unsigned short responseSize;
	unsigned short allocLength;
	boms_csw_t csw;

	allocLength = cbw->cb.raw[7] * 256 + cbw->cb.raw[8];
	buffer = vos_malloc(allocLength);

	// forward the CBW to device
	if (forward_cbw_to_device(cbw))
	{
		// receive response from device
		if (responseSize = receive_data_from_device(buffer, allocLength))
		{
			// forward response to slave
			forward_data_to_slave(&buffer[0], responseSize);

			// receive CSW from device
			if (receive_csw_from_device(&csw))
			{
				// forward CSW to slave
				forward_csw_to_slave(&csw);
			}
		}
	}

	vos_free(buffer);
}

void handle_read_toc_pma_atip(boms_cbw_t *cbw)
{
	unsigned char *buffer;
	unsigned short responseSize;
	unsigned short allocLength;
	boms_csw_t csw;

	allocLength = cbw->cb.raw[7] * 256 + cbw->cb.raw[8];
	buffer = vos_malloc(allocLength);

	// forward the CBW to device
	if (forward_cbw_to_device(cbw))
	{
		// receive response from device
		if (responseSize = receive_data_from_device(buffer, allocLength))
		{
			// forward response to slave
			forward_data_to_slave(&buffer[0], responseSize);

			// receive CSW from device
			if (receive_csw_from_device(&csw))
			{
				// forward CSW to slave
				forward_csw_to_slave(&csw);
			}
		}
	}

	vos_free(buffer);
}

// This function handles the call to prevent/allow removal
// If we fail this command when prevent=1 (true) then
// Windows will not cache writes.  This actually leads
// to better performance.
void handle_prevent_allow_removal(boms_cbw_t *cbw)
{
	usbslave_ioctl_cb_t iocb;
	boms_csw_t csw;

	// now send the CSW
	csw.sig[0]='U';
	csw.sig[1]='S';
	csw.sig[2]='B';
	csw.sig[3]='S';//"USBS"
	csw.tag=cbw->tag;
	csw.residue=0;
	csw.status=0x01; // 0x00=success 0x01=failure 0x02=phase error
	forward_csw_to_slave(&csw);

	// flag the error for the anticipated call to REQUEST SENSE
	illegalRequest=1;

}

void handleCbw()
{
	unsigned short num_read, num_written;
	boms_cbw_t *cbw = vos_malloc(sizeof(boms_cbw_t));

	vos_wait_semaphore(&slaveBomsCtx->enumed);
	vos_signal_semaphore(&slaveBomsCtx->enumed);

	while(1)
	{
		if(slaveBomsCtx)
		{
			while(slaveBomsCtx && slaveBomsCtx->flashConnected)
				{
				// get the CBW
				memset(cbw, 0, sizeof(boms_cbw_t));
				usbSlaveBoms_readCbw(cbw, slaveBomsCtx);
				// TO DO: Check for valid CBW

				switch (cbw->cb.formated.command)
				{
					case BOMS_INQUIRY:
						handle_inquiry(cbw);
						break;
					case BOMS_MODE_SELECT_6:
					case BOMS_MODE_SELECT_10:
						handle_mode_select(cbw);
						break;
					case BOMS_MODE_SENSE_6:
					case BOMS_MODE_SENSE_10:
						handle_mode_sense(cbw);
						break;
					case BOMS_READ_6:
					case BOMS_READ_10:
					case BOMS_READ_12:
						handle_read(cbw);
						break;
					case BOMS_READ_CAPACITY:
						handle_read_capacity(cbw);
						break;
					case BOMS_REPORT_LUNS:
						handle_report_luns(cbw);
						break;
					case BOMS_REQUEST_SENSE:
						handle_request_sense(cbw);
						break;
					case BOMS_TEST_UNIT_READY:
						handle_test_unit_ready(cbw);
						break;
					case BOMS_SEND_DIAGNOSTIC:
						handle_send_diagnostic(cbw);
						break;
					case BOMS_START_STOP_UNIT:
						handle_start_stop_unit(cbw);
						break;
					case BOMS_SYCHRONIZE_CACHE:
						handle_synchronize_cache(cbw);
						break;
					case BOMS_READ_FORMAT_CAPACITIES:
						handle_read_format_capacities(cbw);
						break;
					case BOMS_PREVENT_ALLOW_REMOVAL:
						handle_prevent_allow_removal(cbw);
						break;
					case BOMS_READ_TOC_PMA_ATIP:
						handle_read_toc_pma_atip(cbw);
						break;
					case BOMS_VERIFY:
					case BOMS_FORMAT_UNIT:
						// tell them NO! by failing command
						handle_illegal_request(cbw);
						break;
					case BOMS_WRITE_6:
					case BOMS_WRITE_10:
					case BOMS_WRITE_12:
						handle_illegal_write_request(cbw);
						break;
					default:
						handle_illegal_request(cbw);
						break;
				} // switch
			} // inner while
		} else
		{
			vos_delay_msecs(1000);
		}
	} // outer while

	vos_free(cbw);
}


