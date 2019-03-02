#include "vistacnc_p2s_usb.h"

#include <stdint.h>
#include "usb_ch9.h"
#include "Usb.h"
#include "hid.h"
#include "Arduino.h"
#include "confdescparser.h"

const uint32_t VistaCNC_P2S::epControlIndex  = 0;
const uint32_t VistaCNC_P2S::epDataInIndex  = 1;
const uint32_t VistaCNC_P2S::epDataOutIndex = 2;

/* prints hex numbers with leading zeroes */
// copyright, Peter H Anderson, Baltimore, MD, Nov, '07
// source: http://www.phanderson.com/arduino/arduino_display.html
void print_hex(uint32_t v, uint8_t num_places) {
  int mask=0, n, num_nibbles, digit;
 
  for (n=1; n<=num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask; // truncate v to specified number of places
 
  num_nibbles = num_places / 4;
  if ((num_places % 4) != 0) {
    ++num_nibbles;
  }
 
  do {
    digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
    Serial.print(digit, HEX);
  } while(--num_nibbles);
}

VistaCNC_P2S::VistaCNC_P2S(USBHost *p):
		pUsb(p),
		bAddress(0),
		bNumEP(1),
		encoderOffset(0),
		ready(false)
{
  	// Initialize endpoint data structures
	for (uint32_t i = 0; i < P2S_NUM_EP; ++i) {
		epInfo[i].deviceEpNum	= 0;
		epInfo[i].hostPipeNum	= 0;
		epInfo[i].maxPktSize	= (i) ? 0 : 8;
		epInfo[i].epAttribs	= 0;
		epInfo[i].bmNakPower  	= (i) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
	}

	// Register in USB subsystem
	if (pUsb) {
		pUsb->RegisterDeviceClass(this);
	}
}

uint32_t VistaCNC_P2S::Init(uint32_t parent, uint32_t port, uint32_t lowspeed) {
  	TRACE_VISTACNC(printf("VistaCNC_P2S::Init\r\n");)

	uint8_t		buf[sizeof(USB_DEVICE_DESCRIPTOR)];
	uint32_t	rcode = 0;
	UsbDevice	*p = NULL;
	EpInfo		*oldep_ptr = NULL;

	// Get memory address of USB device address pool
	AddressPool &addrPool = pUsb->GetAddressPool();

	// Check if address has already been assigned to an instance
	if (bAddress) {
		return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
	}

	// Get pointer to pseudo device with address 0 assigned
	p = addrPool.GetUsbDevicePtr(0);

	if (!p) {
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
 	}
	if (!p->epinfo) {
		return USB_ERROR_EPINFO_IS_NULL;
	}

	// Save old pointer to EP_RECORD of address 0
	oldep_ptr = p->epinfo;

	// Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
	p->epinfo = epInfo;

	p->lowspeed = lowspeed;

	// Get device descriptor
	rcode = pUsb->getDevDescr(0, 0, sizeof(USB_DEVICE_DESCRIPTOR), (uint8_t*)buf);

	// Restore p->epinfo
	p->epinfo = oldep_ptr;

	if (rcode) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::Init getDevDescr : ");)
		goto Fail;
	}

	if (((USB_DEVICE_DESCRIPTOR*)buf)->idVendor != P2S_VID || (((USB_DEVICE_DESCRIPTOR*)buf)->idProduct != P2S_PID)) {
		return USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
	}

	// Allocate new address according to device class
	bAddress = addrPool.AllocAddress(parent, false, port);

	// Extract Max Packet Size from device descriptor
	epInfo[0].maxPktSize = (uint8_t)((USB_DEVICE_DESCRIPTOR*)buf)->bMaxPacketSize0;

	// Assign new address to the device
	rcode = pUsb->setAddr(0, 0, bAddress);
	p->lowspeed = false;
	if (rcode) {
		addrPool.FreeAddress(bAddress);
		bAddress = 0;
		TRACE_VISTACNC(printf("VistaCNC_P2S::Init : setAddr failed with rcode %lu\r\n", rcode);)
		return rcode;
	}
	TRACE_VISTACNC(printf("VistaCNC_P2S::Init : device address is now %lu\r\n", bAddress);)
	
	// Wait for device to change its address.
	// Spec says you should wait at least 200ms
	delay(200);

	//get pointer to assigned address record
	p = addrPool.GetUsbDevicePtr(bAddress);
	if (!p) {
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
	}

	p->lowspeed = lowspeed;

	// Assign epInfo to epinfo pointer - only EP0 is known
	rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);
	if (rcode) {
		goto FailSetDevTblEntry;
	}

	//Parse Config.
	if(P2S_AUTOCONFIG_EXPERIMENTAL) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::Init Parsing Configuration\r\n");)
		ConfigDescParser<03, 0, 0, 0> confDescrParser(this);
		delay(1);
		rcode = pUsb->getConfDescr(bAddress, epControlIndex, P2S_CONF_NUM, &confDescrParser);
		if (rcode) {
			TRACE_VISTACNC(printf("VistaCNC_P2S::Init getConf : ");)
			goto Fail;
		} else {
			TRACE_VISTACNC(printf("VistaCNC_P2S::Init Parsing Configuration : Completed\r\n");)
		}
	} else {
		TRACE_VISTACNC(printf("VistaCNC_P2S::Init Manual Configuration\r\n");)
		epInfo[epControlIndex].deviceEpNum = 0x00;
		epInfo[epControlIndex].maxPktSize = 8;
		epInfo[epControlIndex].epAttribs = 0x00;
		epInfo[epControlIndex].bmNakPower = USB_NAK_NONAK;
  
		epInfo[epDataInIndex].deviceEpNum = 0x81; //0x81
		epInfo[epDataInIndex].maxPktSize = 4;
		epInfo[epDataInIndex].epAttribs = 0x00;
		epInfo[epDataInIndex].bmNakPower = USB_NAK_DEFAULT;

		epInfo[epDataOutIndex].deviceEpNum = 0x1;
		epInfo[epDataOutIndex].maxPktSize = 20;
		epInfo[epDataOutIndex].epAttribs = 0x00;
		epInfo[epDataOutIndex].bmNakPower = USB_NAK_DEFAULT;
		
		epInfo[epDataInIndex].hostPipeNum = UHD_Pipe_Alloc(bAddress,
                                                           epInfo[epDataInIndex].deviceEpNum,
                                                           UOTGHS_HSTPIPCFG_PTYPE_INTRPT,
                                                           UOTGHS_HSTPIPCFG_PTOKEN_IN,
                                                           epInfo[epDataInIndex].maxPktSize,
                                                           10,
                                                           UOTGHS_HSTPIPCFG_PBK_1_BANK);
		if (epInfo[epDataInIndex].hostPipeNum == 0) {
			TRACE_VISTACNC(printf("VistaCNC_P2S::Init Input pipe : ");)
			rcode = USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE;
			goto Fail;
		}
                                                                 
		epInfo[epDataOutIndex].hostPipeNum = UHD_Pipe_Alloc(bAddress,
                                                            epInfo[epDataOutIndex].deviceEpNum,
                                                            UOTGHS_HSTPIPCFG_PTYPE_INTRPT,
                                                            UOTGHS_HSTPIPCFG_PTOKEN_OUT,
                                                            epInfo[epDataOutIndex].maxPktSize,
                                                            10,
                                                            UOTGHS_HSTPIPCFG_PBK_1_BANK);
		if (epInfo[epDataOutIndex].hostPipeNum == 0) {
			TRACE_VISTACNC(printf("VistaCNC_P2S::Init Output pipe : ");)
			rcode = USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE;
			goto Fail;
		}
		
		TRACE_VISTACNC(printf("VistaCNC_P2S::Init Manual Configuration : Completed\r\n");)
	}
	
	// Assign epInfo to epinfo pointer - this time all 3 endpoins
	rcode = pUsb->setEpInfoEntry(bAddress, P2S_NUM_EP, epInfo);
	if (rcode) {
		goto FailSetDevTblEntry;
	}
	
	// Set Configuration Value
	rcode = pUsb->setConf(bAddress, 0, P2S_CONF_NUM);
	if (rcode) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::Init setConf : ");)
		goto Fail;
	}

	TRACE_VISTACNC(printf("VistaCNC_P2S::Init : Device configured successfully\r\n");)
	ready = true;
	return 0;

	// Diagnostic messages
FailSetDevTblEntry:
	TRACE_VISTACNC(printf("VistaCNC_P2S::Init setDevTblEn : ");)
	goto Fail;
Fail:
	TRACE_VISTACNC(printf("error code: %lu\r\n", rcode);)
	Release();
	return rcode;
}

void VistaCNC_P2S::EndpointXtract(uint32_t conf, uint32_t iface, uint32_t alt, uint32_t proto, const USB_ENDPOINT_DESCRIPTOR *pep) {
	TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : Parsing\r\n");)
	
	if (P2S_CONF_NUM != conf) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : Wrong config\r\n");)
		return;
	}
	if (bNumEP == P2S_NUM_EP) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : All needed endpoints are already known\r\n");)
		return;
	}
	TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : EndpointAddress: %lu\r\n", pep->bEndpointAddress);)
	TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : Attributes : %lu\r\n", pep->bmAttributes);)
	
	uint32_t index = 0;
	uint32_t pipe = 0;
	if ((pep->bmAttributes & 0x03) == 3) {
		index = ((pep->bEndpointAddress & 0x80) == 0x80) ? epDataInIndex : epDataOutIndex;
	}

	// Fill in the endpoint info structure
	epInfo[index].deviceEpNum = pep->bEndpointAddress & 0x0F;
	epInfo[index].maxPktSize = pep->wMaxPacketSize;

	TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : Found new endpoint\r\n");)
	TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : deviceEpNum: %lu\r\n", epInfo[index].deviceEpNum);)
	TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : maxPktSize: %lu\r\n", epInfo[index].maxPktSize);)
	TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : index: %lu\r\n", index);)
	
	/*
	 * #define   UOTGHS_HSTPIPCFG_PTYPE_CTRL (0x0u << 12)
	 * #define   UOTGHS_HSTPIPCFG_PTYPE_ISO (0x1u << 12)
	 * #define   UOTGHS_HSTPIPCFG_PTYPE_BLK (0x2u << 12)
	 * #define   UOTGHS_HSTPIPCFG_PTYPE_INTRPT (0x3u << 12)
	 * */

	if (index == epDataInIndex) {
		pipe = UHD_Pipe_Alloc(bAddress, epInfo[index].deviceEpNum, UOTGHS_HSTPIPCFG_PTYPE_INTRPT, UOTGHS_HSTPIPCFG_PTOKEN_IN, epInfo[index].maxPktSize, 0, UOTGHS_HSTPIPCFG_PBK_1_BANK);
	} else if (index == epDataOutIndex) {
		pipe = UHD_Pipe_Alloc(bAddress, epInfo[index].deviceEpNum, UOTGHS_HSTPIPCFG_PTYPE_INTRPT, UOTGHS_HSTPIPCFG_PTOKEN_OUT, epInfo[index].maxPktSize, 0, UOTGHS_HSTPIPCFG_PBK_1_BANK);
	}

	// Ensure pipe allocation is okay
	if (pipe == 0) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::EndpointXtract : Pipe allocation failure\r\n");)
		// Enumeration failed, so user should not perform write/read since isConnected will return false
		return;
	}

	epInfo[index].hostPipeNum = pipe;

	bNumEP++;
}

uint32_t VistaCNC_P2S::Release()
{
	// Free allocated host pipes
	UHD_Pipe_Free(epInfo[epDataInIndex].hostPipeNum);
	UHD_Pipe_Free(epInfo[epDataOutIndex].hostPipeNum);

	// Free allocated USB address
	pUsb->GetAddressPool().FreeAddress(bAddress);

	// Must have to be reset to 1
	bNumEP = 1;

	bAddress = 0;
	ready = false;

	return 0;
}

uint32_t VistaCNC_P2S::Poll() {
	if (!ready) {
		return 0;
	}
	uint32_t currentTime = millis();
	if (currentTime < lastPoll) {
		lastPoll = currentTime;
		return 0;
	}
	if (currentTime < lastPoll + P2S_POLL_DELAY) {
		return 0;
	}
	if (currentTime + P2S_POLL_DELAY < currentTime) {
		lastPoll = 0;
	} else {
		lastPoll = currentTime;
	}
	
	uint32_t readbytes = epInfo[epDataInIndex].maxPktSize;
	uint8_t data[readbytes];
	uint32_t rcode = read(&readbytes, (uint8_t*)&data);
	if (rcode) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::Poll : ");)
		TRACE_VISTACNC(printUsbErr (rcode);)
		TRACE_VISTACNC(printf("\r\n");)
		return rcode;
	}
	
	uint8_t oldAbs = encoderAbsolute;
	encoderAbsolute = data[0] + encoderOffset;
	encoderSpeed = data[1];
	inputData = (data[2] << 8) | data[3];
	
	if (abs(encoderAbsolute - oldAbs) > 128) {
		//Overflow
		if (encoderAbsolute < 128) {
			encoderRelative += 255 - oldAbs;
			encoderRelative += encoderAbsolute;
		} else {
			encoderRelative -= oldAbs;
			encoderRelative -= 255 - encoderAbsolute;
		}
	} else {
		encoderRelative += encoderAbsolute - oldAbs;
	}

	setDisplay();

	return 0;
}

/**
 * \brief Read from Device.
 *
 * \param nreadbytes Return value containing the number of read bytes.
 * \param datalen Buffer length.
 * \param dataptr Buffer to store the incoming data.
 *
 * \return 0 on success, error code otherwise.
 */
inline uint32_t VistaCNC_P2S::read(uint32_t *nreadbytes, uint8_t *dataptr) {
	return pUsb->inTransfer(bAddress, epInfo[epDataInIndex].deviceEpNum, nreadbytes, dataptr);
}

void VistaCNC_P2S::setEncoderPosition(uint8_t absEnc) {
	encoderOffset = absEnc - encoderAbsolute;
}

int32_t VistaCNC_P2S::readMovement() {
	uint32_t tmpMove = encoderRelative;
	encoderRelative = 0;
	return tmpMove;
}

uint32_t VistaCNC_P2S::setDisplay() {
	uint32_t datlen = 20;
	uint8_t displayText[datlen];
	
	//ID
	displayText[0] = 0;
	//Reserved
	displayText[1] = 0;
  
	//1Row
	displayText[2] = 'T';
	displayText[3] = 'E';
	displayText[4] = 'S';
	displayText[5] = 'T';
	displayText[6] = 'T';
	displayText[7] = 'E';
	displayText[8] = 'S';
	displayText[9] = 'T';

	//2Row
	displayText[10] = 'T';
	displayText[11] = 'E';
	displayText[12] = 'S';
	displayText[13] = 'T';
	displayText[14] = 'T';
	displayText[15] = 'E';
	displayText[16] = 'S';
	displayText[17] = 'T';

	//Activity...
	//displayText[18]++;
	//Content in out[18] (0 to 255) needs to change every 1/8 second or less, change could be random or sequential. If no change, LCD will show banner only.
	displayText[18] = (uint8_t)millis();
  
	uint32_t rcode = write(datlen, (uint8_t*)&displayText);
   	if (rcode) {
		TRACE_VISTACNC(printf("VistaCNC_P2S::setDisplay : ");)
		TRACE_VISTACNC(printUsbErr (rcode);)
		TRACE_VISTACNC(printf("\r\n");)
		return rcode;
	}
}

/**
 * \brief Write to Device.
 *
 * \param datalen Amount of data to send. This parameter shall not exceed
 * dataptr length.
 * \param dataptr Buffer containing the data to be sent.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t VistaCNC_P2S::write(uint32_t datalen, uint8_t *dataptr)
{
	return pUsb->outTransfer(bAddress, epInfo[epDataOutIndex].deviceEpNum, datalen, dataptr);
}

void VistaCNC_P2S::printUsbErr (uint32_t rcode) {
  switch (rcode) {
    case USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED:
      printf("Usb device configuration error: Device not supported.");
      break;
    case USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE:
      printf("Usb device configuration error: Device initialization incomplete.");
      break;
    case USB_ERROR_UNABLE_TO_REGISTER_DEVICE_CLASS:
      printf("Usb error: Unable to register device class.");
      break;
    case USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL:
      printf("Usb error: Out of address space in pool.");
      break;
    case USB_ERROR_HUB_ADDRESS_OVERFLOW:
      printf("Usb error: HUB Address overflow.");
      break;
    case USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL:
      printf("Usb error: Address not found in pool.");
      break;
    case USB_ERROR_EPINFO_IS_NULL:
      printf("Usb error: EPINFO is Null.");
      break;
    case USB_ERROR_INVALID_ARGUMENT:
      printf("Usb error: INVALID ARGUMENT");
      break;
    case USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE:
      printf("Usb error: Class instance already in use.");
      break;
    case USB_ERROR_INVALID_MAX_PKT_SIZE:
      printf("Usb error: Invalid maximum packet size.");
      break;
    case USB_ERROR_EP_NOT_FOUND_IN_TBL:
      printf("Usb error: Endpoint not found in TBL");
      break;
    case USB_ERROR_TRANSFER_TIMEOUT:
      printf("Usb error: Transfer Timeout");
      break;
    default:
      printf("Usb error: %lu", rcode);
      break;
  }
}



/*

byte displayText[19];
void setupEpInfo() {
  P2S_ep_record[P2S_CONTROL_PIPE].deviceEpNum = 0x00;
  P2S_ep_record[P2S_CONTROL_PIPE].maxPktSize = 8;
  P2S_ep_record[P2S_CONTROL_PIPE].epAttribs = 0x00;
  P2S_ep_record[P2S_CONTROL_PIPE].bmNakPower = USB_NAK_NOWAIT;
  
  P2S_ep_record[P2S_INPUT_PIPE].deviceEpNum = 0x81; //0x81
  P2S_ep_record[P2S_INPUT_PIPE].maxPktSize = 4;
  P2S_ep_record[P2S_INPUT_PIPE].epAttribs = 0x00;
  P2S_ep_record[P2S_INPUT_PIPE].bmNakPower = USB_NAK_MAX_POWER;

  P2S_ep_record[P2S_OUTPUT_PIPE].deviceEpNum = 0x1;
  P2S_ep_record[P2S_OUTPUT_PIPE].maxPktSize = 20;
  P2S_ep_record[P2S_OUTPUT_PIPE].epAttribs = 0x00;
  P2S_ep_record[P2S_OUTPUT_PIPE].bmNakPower = USB_NAK_MAX_POWER;
}


void setupUsb() {
  Serial.println("\r\nUsbHost Setup");
  
  //Wait for usb stack to be ready.
  byte usbState = Host.getUsbTaskState();
  if (usbState != USB_STATE_CONFIGURING && usbState != USB_STATE_RUNNING) {
    Serial.println("\r\nWaiting for UsbHost....");
  }
  unsigned long timeout = millis() + configTimeoutDelay;
  while(usbState != USB_STATE_CONFIGURING && usbState != USB_STATE_RUNNING && millis() < timeout) {
    Host.Task();
    usbState = Host.getUsbTaskState();
  }
  //Check for timeout or usb state errors.
  if (usbState == USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE) {
    Serial.println("\r\nUsbHost nothing connected.");
    return;
  }
  if (usbState == USB_STATE_ERROR) {
    Serial.println("\r\nUsb state error.");
    return;
  }
  if (usbState != USB_STATE_CONFIGURING && usbState != USB_STATE_RUNNING) {
      Serial.print("\r\nTimedout waiting for UsbHost, UsbHost State:\t");
      print_hex(usbState, 8);
      return;
  }
  //state configuring or higher
  
  // get memory address of USB device address pool
  AddressPool &addrPool = Host.GetAddressPool();
        
  // Get pointer to pseudo device with address 0 assigned
  usbDevice = addrPool.GetUsbDevicePtr(0);
  if (!usbDevice) {
    Serial.println("\r\nUsb device at address 0 not found.");
    return;
  }
  
  USB_DEVICE_DESCRIPTOR descBuf;
  byte rcode = Host.getDevDescr(0, 0, DEV_DESCR_LEN, (uint8_t*)&descBuf);
  if (rcode) {
    Serial.print("\r\nRequest error. Error code:\t");
    printUsbErr (rcode);
  }
  
  if (descBuf.idVendor != P2S_VID || descBuf.idProduct != P2S_PID) {
    Serial.print("\r\nUnknown Usb Device:\t");
    Serial.print(" Vendor  ID:\t");
    print_hex(descBuf.idVendor, 16);
    Serial.print(" Product ID:\t");
    print_hex(descBuf.idProduct, 16);
  }
  
  setupEpInfo();
  
  // Print new max packet size. Only if changed.
  if (P2S_ep_record[0].maxPktSize != (uint8_t)descBuf.bMaxPacketSize0) {
    P2S_ep_record[0].maxPktSize = (uint8_t)descBuf.bMaxPacketSize0;
    Serial.print("\r\nP2S maxPktSize updated to: 0x");
    print_hex(usbDevice->epinfo[0].maxPktSize, 32);
    Serial.println("");
  }

  // Allocate new address according to device class
  uint8_t parent = 0, port = 0;
  deviceAddress = addrPool.AllocAddress(parent, false, port);
  if (!deviceAddress) {
    Serial.println("\r\nUsbHost could not assin a new address to the device.");
    return;
  }
  
  // Assign new address to the device
  rcode = Host.setAddr(0, 0, deviceAddress);
  if (rcode) {
    //device.lowspeed = false;
    addrPool.FreeAddress(deviceAddress);
    deviceAddress = 0;
    
    Serial.print("\r\nUsb Set Address. Error code:\t");
    printUsbErr (rcode);
    return;
  }
  
  // Wait for device to change its address.
  // Spec says you should wait at least 200ms
  delay(200);
  
  //get pointer to assigned address record
  usbDevice = addrPool.GetUsbDevicePtr(deviceAddress);
  if (!usbDevice) {
    Serial.println("\r\nUsb Address not found.");
    return;
  }
  if (!usbDevice->epinfo) {
    Serial.println("\r\nUsbDevice has no endpoint information.");
    return;
  }
  
  rcode = Host.setConf(deviceAddress, P2S_CONTROL_PIPE, 1);
  if (rcode) {
    Serial.print("\r\nUsb Set Configuration returned error code:\t");
    printUsbErr (rcode);
    return;
  }
  
  if (InitPipe()) {
    Serial.println("\r\nError while initializing endpoints pipes.");
    return;
  }

  // Assign epInfo to epinfo pointer
  rcode = Host.setEpInfoEntry(deviceAddress, P2S_NUM_EP, P2S_ep_record);
  if (rcode) {
    Serial.print("\r\nUsb Set EpInfo. Error code:\t");
    printUsbErr (rcode);
    return;
  }
  
  usbDeviceConfigured = true;
  Serial.println("\r\nUsb device sucessfully configured.");
  
  Serial.print("\r\nUsbHost epcount: ");
  print_hex(usbDevice->epcount, 32);
  Serial.println("");
  
  Serial.print("\r\nUsbHost epinfo[1].deviceEpNum: ");
  print_hex(usbDevice->epinfo[1].deviceEpNum, 32);
  Serial.println("");

  Serial.print("\r\nUsbHost epinfo[1].maxPktSize: ");
  print_hex(usbDevice->epinfo[1].maxPktSize, 8);
  Serial.println("");
  
  //printdevdescr(deviceAddress);
}

void updateUsb() {
  if (!usbDeviceConfigured) {
    static unsigned long nextUsbCheck;
    if (nextUsbCheck < millis()) {
      nextUsbCheck = millis() + checkConfigDelay;
      setupUsb();
      return;
    }
  }
  Host.Task();
  
  //ID
  displayText[0] = 0;
  //Reserved
  displayText[1] = 0;
  
  //1Row
  displayText[2] = 'T';
  displayText[3] = 'E';
  displayText[4] = 'S';
  displayText[5] = 'T';
  displayText[6] = 'T';
  displayText[7] = 'E';
  displayText[8] = 'S';
  displayText[9] = 'T';

  //2Row
  displayText[10] = 'T';
  displayText[11] = 'E';
  displayText[12] = 'S';
  displayText[13] = 'T';
  displayText[14] = 'T';
  displayText[15] = 'E';
  displayText[16] = 'S';
  displayText[17] = 'T';

  //Activity...
  displayText[18]++;
  
  byte rcode = 0;
  
  //Update Text Display
  rcode = Host.outTransfer (deviceAddress, usbDevice->epinfo[P2S_OUTPUT_PIPE].deviceEpNum, 19, (uint8_t*)&displayText);
  if (rcode) {
    Serial.print("\r\nDisplay update error. Error code: \t");
    printUsbErr (rcode);
    Serial.println ("");
  }
  
  uint32_t readbytes = 8;
  uint8_t data[readbytes];
  rcode = Host.inTransfer(deviceAddress, usbDevice->epinfo[P2S_INPUT_PIPE].deviceEpNum, &readbytes, (uint8_t*)&data);
  if (rcode) {
    Serial.print("\r\nInput update error. Error code: \t");
    printUsbErr (rcode);
    Serial.println ("");
  } else {
    Serial.print("\r\Input update: \t");
    for(byte i = 0; i < readbytes; i++) {
      if(i > 0) {
        Serial.print (", ");
      }
      Serial.print ("0x");
      print_hex(data[i], 8);
    }
    Serial.println ("");
  }
}

byte printdevdescr(byte addr) {
  USB_DEVICE_DESCRIPTOR buf;
  byte rcode;
  rcode = Host.getDevDescr(addr, 0, DEV_DESCR_LEN, (uint8_t*)&buf);
  if (rcode) {
    return rcode;
  }
  
  Serial.println("\r\nDevice descriptor: ");
  Serial.print("Descriptor Length:\t");
  print_hex( buf.bLength, 8 );
  Serial.print("\r\nDescriptor type:\t");
  print_hex( buf.bDescriptorType, 8 );
  Serial.print("\r\nUSB version:\t");
  print_hex( buf.bcdUSB, 16 );
  Serial.print("\r\nDevice class:\t");
  print_hex( buf.bDeviceClass, 8 );
  Serial.print("\r\nDevice Subclass:\t");
  print_hex( buf.bDeviceSubClass, 8 );
  Serial.print("\r\nDevice Protocol:\t");
  print_hex( buf.bDeviceProtocol, 8 );
  Serial.print("\r\nMax.packet size:\t");
  print_hex( buf.bMaxPacketSize0, 8 );
  Serial.print("\r\nVendor  ID:\t");
  print_hex( buf.idVendor, 16 );
  Serial.print("\r\nProduct ID:\t");
  print_hex( buf.idProduct, 16 );
  Serial.print("\r\nRevision ID:\t");
  print_hex( buf.bcdDevice, 16 );
  Serial.print("\r\nMfg.string index:\t");
  print_hex( buf.iManufacturer, 8 );
  Serial.print("\r\nProd.string index:\t");
  print_hex( buf.iProduct, 8 );
  Serial.print("\r\nSerial number index:\t");
  print_hex( buf.iSerialNumber, 8 );
  Serial.print("\r\nNumber of conf.:\t");
  print_hex( buf.bNumConfigurations, 8 );
  return (0);
}
*/

