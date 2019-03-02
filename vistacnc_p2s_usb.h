#ifndef VISTACNC_P2S_USB
#define VISTACNC_P2S_USB

#include "Usb.h"
#include "Arduino.h"
#include "confdescparser.h"

#define P2S_AUTOCONFIG_EXPERIMENTAL false

#define TRACE_VISTACNC(x)	x
//#define TRACE_VISTACNC(x)

#define P2S_VID 0x04D8
#define P2S_PID 0xFCE2
#define P2S_NUM_EP 3
#define P2S_CONF_NUM 1
#define P2S_POLL_DELAY 10

#define P2S_INPUTMASK_ENABLE 0x1000
#define P2S_INPUTMASK_ESTOP 0x2000
#define P2S_INPUTMASK_SELECT 0x4000
#define P2S_INPUTMASK_MODE_MOVE 0x0007
#define P2S_INPUTMASK_MODE_AXIS 0x0700

#define P2S_AXIS_X 0x01
#define P2S_AXIS_Y 0x02
#define P2S_AXIS_Z 0x03
#define P2S_AXIS_A 0x07

#define P2S_MODE_F1 0x01
#define P2S_MODE_F2 0x02
#define P2S_MODE_F3 0x03
#define P2S_MODE_F4 0x07
#define P2S_MODE_F5 0x04
#define P2S_MODE_F6 0x05
#define P2S_MODE_F7 0x06

#define P2S_MOVE_MODE_FUNCTIONS 0x06

/**
 * \class VISTACNC_P2S definition.
 */
class VistaCNC_P2S : public USBDeviceConfig, public UsbConfigXtracter
{
private:
	void printUsbErr(uint32_t rcode);
	uint8_t encoderOffset;							//Absolute position offset.
	int32_t encoderRelative;						//Relative to last reading.
	uint8_t encoderAbsolute;						//Absolute encoder position.
	uint8_t encoderSpeed;
	uint16_t inputData;								//Buttons inputs.
	uint32_t lastPoll;

protected:

	static const uint32_t epControlIndex;			// Control endpoint index
	static const uint32_t epDataInIndex;			// DataIn endpoint index
	static const uint32_t epDataOutIndex;			// DataOut endpoint index

	/* Mandatory members */
	USBHost		*pUsb;
	uint32_t	bAddress;							// Device USB address
	uint32_t	bNumEP;								// total number of EP in the configuration
	bool		ready;

	/* Endpoint data structure describing the device EP */
	EpInfo		epInfo[P2S_NUM_EP];

public:
	VistaCNC_P2S(USBHost *pUsb);

	// Methods for receiving and sending data
	uint32_t read(uint32_t *nreadbytes, uint8_t *dataptr);
	uint32_t write(uint32_t datalen, uint8_t *dataptr);

	// USBDeviceConfig implementation
	virtual uint32_t Init(uint32_t parent, uint32_t port, uint32_t lowspeed);
	virtual uint32_t Release();
	virtual uint32_t Poll();
	virtual uint32_t GetAddress() { return bAddress; };
	virtual bool isReady() { return ready; };

	// UsbConfigXtracter implementation
	virtual void EndpointXtract(uint32_t conf, uint32_t iface, uint32_t alt, uint32_t proto, const USB_ENDPOINT_DESCRIPTOR *ep);
	
	uint32_t setDisplay();
	
	void setEncoderPosition(uint8_t absEnc);
	uint8_t getEncoderPosition() { return encoderAbsolute; };
	uint8_t getEncoderSpeed() { return encoderSpeed; };
	
	int32_t peekMovement () { return encoderRelative; };
	int32_t readMovement ();
	
	bool eStop() { return (inputData & P2S_INPUTMASK_ESTOP) != 0; };
	bool select() { return (inputData & P2S_INPUTMASK_SELECT) != 0; };
	uint8_t modeMove() { return (inputData & P2S_INPUTMASK_MODE_MOVE); };
	uint8_t modeAxis() { return (inputData & P2S_INPUTMASK_MODE_AXIS) >> 8; };
	bool enabled() { return ready && (inputData & P2S_INPUTMASK_ENABLE) == 0; };
};

#endif

