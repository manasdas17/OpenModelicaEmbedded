
/*  ModelPlug: Modelica connection to Firmata
 *  Author: Leonardo Laguna Ruiz
 *  Based on Firmata GUI-friendly queries test by Paul Stoffregen (paul@pjrc.com)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <vector>

#include "serial.h"
#include "modelPlugFirmata.h"


typedef struct {
	uint8_t  mode;
	uint8_t  analog_channel;
	uint64_t supported_modes;
	uint32_t value;
	uint32_t next_value;
	uint8_t  ready;
} pin_t;

#define MODE_INPUT    0x00
#define MODE_OUTPUT   0x01
#define MODE_ANALOG   0x02
#define MODE_PWM      0x03
#define MODE_SERVO    0x04
#define MODE_SHIFT    0x05
#define MODE_I2C      0x06

#define START_SYSEX             0xF0 // start a MIDI Sysex message
#define END_SYSEX               0xF7 // end a MIDI Sysex message
#define PIN_MODE_QUERY          0x72 // ask for current and supported pin modes
#define PIN_MODE_RESPONSE       0x73 // reply with current and supported pin modes

#define RESERVED_COMMAND        0x00 // 2nd SysEx data byte is a chip-specific command (AVR, PIC, TI, etc).
#define ANALOG_MAPPING_QUERY    0x69 // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE 0x6A // reply with mapping info
#define CAPABILITY_QUERY        0x6B // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE     0x6C // reply with supported modes and resolution
#define PIN_STATE_QUERY         0x6D // ask for a pin's current mode and value
#define PIN_STATE_RESPONSE      0x6E // reply with a pin's current mode and value
#define EXTENDED_ANALOG         0x6F // analog write (PWM, Servo, etc) to any pin
#define SERVO_CONFIG            0x70 // set max angle, minPulse, maxPulse, freq
#define STRING_DATA             0x71 // a string message with 14-bits per char
#define SHIFT_DATA              0x75 // shiftOut config/data message (34 bits)
#define I2C_REQUEST             0x76 // I2C request messages from a host to an I/O board
#define I2C_REPLY               0x77 // I2C reply messages from an I/O board to a host
#define I2C_CONFIG              0x78 // Configure special I2C settings such as power pins and delay times
#define REPORT_FIRMWARE         0x79 // report name and version of the firmware
#define SAMPLING_INTERVAL       0x7A // sampling interval
#define SYSEX_NON_REALTIME      0x7E // MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME          0x7F // MIDI Reserved for realtime messages

class firmataBoard
{
public:
	firmataBoard(std::string port = "dummy",bool showCapabilitites=false,int samplingMs=10,int baudRate=57600,bool fake=true);
	~firmataBoard(){};

	std::string getPortName();
	int openPort();
	void closePort();
	std::vector<std::string> getPortList();
	void setPinMode(uint32_t pin, uint32_t mode);
	void writeDigitalPin(uint32_t pin,uint32_t value);
	void writeAnalogPin(uint32_t pin, uint32_t value);
	void writeServoPin(uint32_t pin, uint32_t value, int min, int max);
	double readAnalogPin(uint32_t pin, double min, double max, double init);
	uint32_t readDigitalPin(uint32_t pin, int init);
	void setServoConfig(uint32_t pin,uint32_t min,uint32_t max);

public:
	void initialize(bool dtr);
	void updateBoard(int timeout);
	void reportFirmware();
	void showPinCapabilities();
	void setSamplingInterval();
	void setAsFakePort();
private:

	int write(const void *ptr, int len);
	int read(void *ptr, int count,int timeout);
	void Parse(const uint8_t *buf, int len);
	void DoMessage(void);

	pin_t pin_info[128];
	std::string port_name;
	bool show_capabilitites;
	int sampling;
	int baudrate;
	Serial serial_port;
	bool ready;
	bool is_fake_port;


	std::string firmata_name;
	int parse_count;
	int parse_command_len;
	uint8_t parse_buf[4096];
};

// This is an external object that we want to use in our modelica code.
class firmataBoardHandler  {
	firmataBoardHandler(){
		try
		{
			std::vector<std::string> available_ports = dummyPort.getPortList();
			printf("[ModelPlug.Firmata]: Available ports:\n"); fflush(stdout);
			for (std::vector<std::string>::iterator it = available_ports.begin() ; it != available_ports.end(); ++it){
				printf("- %s\n",(*it).c_str());fflush(stdout);
			}
		}
		catch(std::exception&)
		{
			printf("[ModelPlug.Firmata]: Failed to show the port names\n");fflush(stdout);
		}
	}

public:
	static firmataBoardHandler *instance()
    {
        if (!s_instance)
          s_instance = new firmataBoardHandler;
        return s_instance;
    }
    firmataBoard* registerBoard(char* port,bool showCapabilitites,int samplingMs,int baudRate,bool dtr);
    int getPortIndex(firmataBoard* elem);

    firmataBoard* getBoard(uint32_t id);

private:
	std::vector<firmataBoard*> ports;

	firmataBoard dummyPort;

	static firmataBoardHandler *s_instance;

};

firmataBoardHandler *firmataBoardHandler::s_instance = 0;

int firmataBoard::write(const void *ptr, int len){
	if(!is_fake_port) return serial_port.Write(ptr,len);
	return 0;
}

int firmataBoard::read(void *ptr, int count, int timeout){
	if(!is_fake_port){
		if (serial_port.Is_open() && (serial_port.Input_wait(timeout)>0))
			return serial_port.Read(ptr, count);
		return 0;
	}
	return 0;
}

void firmataBoard::setAsFakePort(){
	is_fake_port = true;
}

firmataBoard::firmataBoard(std::string port, bool showCapabilitites, int samplingMs, int baudRate, bool fake)
	:port_name(port),show_capabilitites(showCapabilitites),sampling(samplingMs),baudrate(baudRate),is_fake_port(fake){
	// Initialize the pin information
	for (int i=0; i < 128; i++) {
		pin_info[i].mode = 255;
		pin_info[i].analog_channel = 127;
		pin_info[i].supported_modes = 0;
		pin_info[i].value = 0;
		pin_info[i].next_value = 0;
		pin_info[i].ready = 0;
	}
	ready = false;
}

void firmataBoard::setPinMode(uint32_t pin, uint32_t mode){

	if (mode != pin_info[pin].mode && ready) {
		// send the mode change message
		uint8_t buf[4];
		buf[0] = 0xF4;
		buf[1] = pin;
		buf[2] = mode;
		write(buf, 3);
		pin_info[pin].mode = mode;
		pin_info[pin].value = 0;
		pin_info[pin].next_value = 0;
		printf("[ModelPlug.Firmata]: %s setting pin %i to mode %i\n",port_name.c_str(),pin,mode);
	}
}

void firmataBoard::setServoConfig(uint32_t pin,uint32_t min,uint32_t max){
	if(ready){
		uint8_t buf[8];
		buf[0] = START_SYSEX;
		buf[1] = SERVO_CONFIG;
		buf[2] = pin;
		buf[3] = min & 0x7F;
		buf[4] = (min >> 7) & 0x7F;
		buf[5] = max & 0x7F;
		buf[6] = (max >> 7) & 0x7F;
		buf[7] = END_SYSEX;
		write(buf, 8);
	}
}

void firmataBoard::writeDigitalPin(uint32_t pin,uint32_t value){
	if(ready){
		if(pin_info[pin].mode!=MODE_OUTPUT)
			setPinMode(pin,MODE_OUTPUT);
		pin_info[pin].next_value = value;
	}
}

void firmataBoard::writeAnalogPin(uint32_t pin, uint32_t value){
	if(ready){
		if(pin_info[pin].mode!=MODE_PWM)
			setPinMode(pin,MODE_PWM);
		pin_info[pin].next_value = value;
	}
}

void firmataBoard::writeServoPin(uint32_t pin, uint32_t value, int min, int max){
	if(ready){
		if(pin_info[pin].mode!=MODE_SERVO){
			setPinMode(pin,MODE_SERVO);
			setServoConfig(pin,min,max);
		}
		pin_info[pin].next_value = value;
	}
}

double firmataBoard::readAnalogPin(uint32_t pin,double min, double max, double init){
	if(ready){
		if(pin_info[pin].mode!=MODE_ANALOG)
			setPinMode(pin,MODE_ANALOG);
		if(pin_info[pin].ready)
			return min+((pin_info[pin].value/1023.0)*(max-min));
		else
			return init;
	}
	return init;
}

uint32_t firmataBoard::readDigitalPin(uint32_t pin, int init){
	if(ready){
		if(pin_info[pin].mode!=MODE_INPUT)
			setPinMode(pin,MODE_INPUT);
		if(pin_info[pin].ready)
			return pin_info[pin].value;
		else
			return init;
	}
	return init;
}

void firmataBoard::reportFirmware(){
	uint8_t buf[3];
	buf[0] = START_SYSEX;
	buf[1] = REPORT_FIRMWARE; // read firmata name & version
	buf[2] = END_SYSEX;
	write(buf, 3);
}

void firmataBoard::setSamplingInterval(){
	uint8_t buf[5];
	buf[0] = START_SYSEX;
	buf[1] = SAMPLING_INTERVAL; // read firmata name & version
	buf[2] = sampling & 0x7F;
	buf[3] = (sampling >> 7) & 0x7F;
	buf[4] = END_SYSEX;
	write(buf, 5);
	printf("[ModelPlug.Firmata]: Setting sampling interval to %i ms for board %s\n",sampling,port_name.c_str()); fflush(stdout);
}

void firmataBoard::showPinCapabilities(){
	printf("[ModelPlug.Firmata]: Board %s capabilities\n", port_name.c_str());
	for (int pin=0; pin<128; pin++) {
		if(pin_info[pin].supported_modes!=0){
			printf("- Pin %i supports: ",pin);
			if (pin_info[pin].supported_modes & (1<<MODE_INPUT))  printf(" DigitalInput");
			if (pin_info[pin].supported_modes & (1<<MODE_OUTPUT)) printf(" - DigitalOutput");
			if (pin_info[pin].supported_modes & (1<<MODE_ANALOG)) printf(" - AnalogInput(A%d)",pin_info[pin].analog_channel);
			if (pin_info[pin].supported_modes & (1<<MODE_PWM))    printf(" - AnalogOutput");
			if (pin_info[pin].supported_modes & (1<<MODE_SERVO))  printf(" - Servo");
			printf("\n");
		}
	}
	fflush(stdout);
}

void firmataBoard::updateBoard(int timeout){
	uint8_t buf[1024];
	int r;
	// write the output values
	if(ready)
	for (int pin=0; pin<128; pin++) {
		if(pin_info[pin].value!=pin_info[pin].next_value && (pin_info[pin].mode==MODE_OUTPUT || pin_info[pin].mode==MODE_PWM || pin_info[pin].mode==MODE_SERVO)){

			pin_info[pin].value=pin_info[pin].next_value;

			// Digital output
			if(pin_info[pin].mode==MODE_OUTPUT){
				int port_num = pin / 8;
				int port_val = 0;
				for (int i=0; i<8; i++) {
					int p = port_num * 8 + i;
					if (pin_info[p].mode == MODE_OUTPUT || pin_info[p].mode == MODE_INPUT) {
						if (pin_info[p].value) {
							port_val |= (1<<i);
						}
					}
				}
				uint8_t buf[3];
				buf[0] = 0x90 | port_num;
				buf[1] = port_val & 0x7F;
				buf[2] = (port_val >> 7) & 0x7F;
				write(buf, 3);
			}
			// Analog output or servo
			if(pin_info[pin].mode==MODE_PWM || pin_info[pin].mode==MODE_SERVO){
				uint32_t value = pin_info[pin].value;

				if (pin <= 15 && value <= 16383) {
					uint8_t buf[3];
					buf[0] = 0xE0 | pin;
					buf[1] = value & 0x7F;
					buf[2] = (value >> 7) & 0x7F;
					write(buf, 3);
				} else {
					uint8_t buf[12];
					int len=4;
					buf[0] = 0xF0;
					buf[1] = 0x6F;
					buf[2] = pin;
					buf[3] = value & 0x7F;
					if (value > 0x00000080) buf[len++] = (value >> 7) & 0x7F;
					if (value > 0x00004000) buf[len++] = (value >> 14) & 0x7F;
					if (value > 0x00200000) buf[len++] = (value >> 21) & 0x7F;
					if (value > 0x10000000) buf[len++] = (value >> 28) & 0x7F;
					buf[len++] = 0xF7;
					write(buf, len);
				}
			}
		}
	}
	// receieve bytes from the serial port
	r = read(buf, sizeof(buf),timeout);
	if (r) {
		Parse(buf, r);
	}
}

void firmataBoard::Parse(const uint8_t *buf, int len)
{
	const uint8_t *p, *end;

	p = buf;
	end = p + len;
	for (p = buf; p < end; p++) {
		uint8_t msn = *p & 0xF0;
		if (msn == 0xE0 || msn == 0x90 || *p == 0xF9) {
			parse_command_len = 3;
			parse_count = 0;
		} else if (msn == 0xC0 || msn == 0xD0) {
			parse_command_len = 2;
			parse_count = 0;
		} else if (*p == START_SYSEX) {
			parse_count = 0;
			parse_command_len = sizeof(parse_buf);
		} else if (*p == END_SYSEX) {
			parse_command_len = parse_count + 1;
		} else if (*p & 0x80) {
			parse_command_len = 1;
			parse_count = 0;
		}
		if (parse_count < (int)sizeof(parse_buf)) {
			parse_buf[parse_count++] = *p;
		}
		if (parse_count == parse_command_len) {
			DoMessage();
			parse_count = parse_command_len = 0;
		}
	}
}

void firmataBoard::DoMessage(void)
{
	uint8_t cmd = (parse_buf[0] & 0xF0);

	if (cmd == 0xE0 && parse_count == 3) {
		int analog_ch = (parse_buf[0] & 0x0F);
		int analog_val = parse_buf[1] | (parse_buf[2] << 7);
		for (int pin=0; pin<128; pin++) {
			if (pin_info[pin].analog_channel == analog_ch) {
				pin_info[pin].value = analog_val;
				pin_info[pin].ready = 1;
				return;
			}
		}
		return;
	}
	if (cmd == 0x90 && parse_count == 3) {
		int port_num = (parse_buf[0] & 0x0F);
		int port_val = parse_buf[1] | (parse_buf[2] << 7);
		int pin = port_num * 8;
		//printf("port_num = %d, port_val = %d\n", port_num, port_val);
		for (int mask=1; mask & 0xFF; mask <<= 1, pin++) {
			if (pin_info[pin].mode == MODE_INPUT) {
				uint32_t val = (port_val & mask) ? 1 : 0;
				if (pin_info[pin].value != val) {
					pin_info[pin].value = val;
					pin_info[pin].ready = 1;
				}
			}
		}
		return;
	}


	if (parse_buf[0] == START_SYSEX && parse_buf[parse_count-1] == END_SYSEX) {
		// Sysex message
		if (parse_buf[1] == REPORT_FIRMWARE) {
			char name[140];
			int len=0;
			for (int i=4; i < parse_count-2; i+=2) {
				name[len++] = (parse_buf[i] & 0x7F)
				  | ((parse_buf[i+1] & 0x7F) << 7);
			}
			name[len++] = '-';
			name[len++] = parse_buf[2] + '0';
			name[len++] = '.';
			name[len++] = parse_buf[3] + '0';
			name[len++] = 0;
			firmata_name = name;
			printf("[ModelPlug.Firmata]: %s %s\n",port_name.c_str(),name);
			ready = true;
			// query the board's capabilities only after hearing the
			// REPORT_FIRMWARE message.  For boards that reset when
			// the port open (eg, Arduino with reset=DTR), they are
			// not ready to communicate for some time, so the only
			// way to reliably query their capabilities is to wait
			// until the REPORT_FIRMWARE message is heard.
			uint8_t buf[80];
			len=0;
			buf[len++] = START_SYSEX;
			buf[len++] = ANALOG_MAPPING_QUERY; // read analog to pin # info
			buf[len++] = END_SYSEX;
			buf[len++] = START_SYSEX;
			buf[len++] = CAPABILITY_QUERY; // read capabilities
			buf[len++] = END_SYSEX;
			for (int i=0; i<16; i++) {
				buf[len++] = 0xC0 | i;  // report analog
				buf[len++] = 1;
				buf[len++] = 0xD0 | i;  // report digital
				buf[len++] = 1;
			}
			write(buf, len);
			setSamplingInterval();
		} else if (parse_buf[1] == CAPABILITY_RESPONSE) {
			int pin, i, n;
			for (pin=0; pin < 128; pin++) {
				pin_info[pin].supported_modes = 0;
			}
			for (i=2, n=0, pin=0; i<parse_count; i++) {
				if (parse_buf[i] == 127) {
					pin++;
					n = 0;
					continue;
				}
				if (n == 0) {
					// first byte is supported mode
					pin_info[pin].supported_modes |= (1<<parse_buf[i]);
				}
				n = n ^ 1;
			}
			if(show_capabilitites) {
				showPinCapabilities();
				show_capabilitites = false;
			}
			// send a state query for for every pin with any modes
			for (pin=0; pin < 128; pin++) {
				uint8_t buf[512];
				int len=0;
				if (pin_info[pin].supported_modes) {
					buf[len++] = START_SYSEX;
					buf[len++] = PIN_STATE_QUERY;
					buf[len++] = pin;
					buf[len++] = END_SYSEX;
				}
				write(buf, len);
			}
		} else if (parse_buf[1] == ANALOG_MAPPING_RESPONSE) {
			int pin=0;
			for (int i=2; i<parse_count-1; i++) {
				pin_info[pin].analog_channel = parse_buf[i];
				pin++;
			}
			return;
		} else if (parse_buf[1] == PIN_STATE_RESPONSE && parse_count >= 6) {
			int pin = parse_buf[2];
			pin_info[pin].mode = parse_buf[3];
			pin_info[pin].value = parse_buf[4];
			if (parse_count > 6) pin_info[pin].value |= (parse_buf[5] << 7);
			if (parse_count > 7) pin_info[pin].value |= (parse_buf[6] << 14);
			//add_pin(pin);
		}
		return;
	}
}

std::vector<std::string> firmataBoard::getPortList(){
	return serial_port.port_list();
}

firmataBoard* firmataBoardHandler::getBoard(uint32_t id){
	if(id<0)
		return &dummyPort;
	else
		return ports[id];
}

firmataBoard* firmataBoardHandler::registerBoard(char* port,bool showCapabilitites,int samplingMs,int baudRate,bool dtr){

	// Check if the port exists
	bool port_exists = false;
	bool fake_port = false;
	std::string port_name(port);
	std::vector<std::string> available_ports = dummyPort.getPortList();
	for (std::vector<std::string>::iterator it = available_ports.begin() ; it != available_ports.end(); ++it){
		if((*it).compare(port_name)==0){
			port_exists = true;
			break;
		}
	}

	if(!port_exists) {
		printf("[ModelPlug.Firmata]: ERROR The port %s does not exist\n",port); fflush(stdout);
		fake_port |= true;
	}

	// Check if the port is already used
	for (std::vector<firmataBoard*>::iterator it = ports.begin() ; it != ports.end(); ++it){
		if ((*it)->getPortName().compare(port_name)==0)
		{
			// Print duplicated port, report error
			printf("[ModelPlug.Firmata]: ERROR: Trying to use two boards with the port %s\n",port);fflush(stdout);
			fake_port |= true;
		}
	}

	firmataBoard* new_port = new firmataBoard(port_name,showCapabilitites,samplingMs,baudRate,fake_port);

	if(new_port->openPort()==0){
		ports.push_back(new_port);
		new_port->initialize(dtr);
		return new_port;
	}
	else{
		// Could not open the port return error
		printf("[ModelPlug.Firmata]: ERROR Failed to open the port %s\n",port); fflush(stdout);
		new_port->setAsFakePort();
		return new_port;
	}
	return new_port;
}

int firmataBoardHandler::getPortIndex(firmataBoard* elem) {
	unsigned pos = std::find(ports.begin(), ports.end(), elem) - ports.begin();
	if(pos >= ports.size()){ // not found
		return -1;
	}
	return pos;
}

std::string firmataBoard::getPortName(){
	return port_name;
}

int firmataBoard::openPort(){
	if(is_fake_port){
		return 0;
	} else {
		serial_port.Open(port_name);
		serial_port.Set_baud(baudrate);
		if(serial_port.Is_open()){
			printf("[ModelPlug.Firmata]: Using port %s with baud rate %i\n",port_name.c_str(),baudrate); fflush(stdout);
			return 0;
		}
		else
			return -1;
	}
	return -1;
}

void firmataBoard::initialize(bool dtr){
	serial_port.Set_control(dtr?1:0,-1);
	updateBoard(5);
	reportFirmware();
	updateBoard(5);
}

void firmataBoard::closePort(){
	if(is_fake_port) return;
	else
		if(serial_port.Is_open())
			serial_port.Close();
}

extern "C"

{


EXPORT void* boardConstructor(char* port,bool showCapabilitites,int samplingMs,int baudRate,bool dtr){
	firmataBoardHandler* boards = firmataBoardHandler::instance();
	void* object = (void*)boards->registerBoard(port,showCapabilitites,samplingMs,baudRate,dtr);
	return object;
}
EXPORT void boardDestructor(void* object){
	if(object!=NULL){
		firmataBoard* board = (firmataBoard*)object;
		board->closePort();
		delete board;
	}
}

EXPORT void updateBoard(int id){
	firmataBoard* board = firmataBoardHandler::instance()->getBoard(id);
	board->updateBoard(0);
}

EXPORT int getBoardId(void* object){
	if(object!=NULL){
		return firmataBoardHandler::instance()->getPortIndex((firmataBoard*)object);
	}
	else
		return -1;
}

EXPORT double readAnalogPin(int pin, double min, double max, double init, int id){
	firmataBoard* board = firmataBoardHandler::instance()->getBoard(id);
	double value = board->readAnalogPin(pin,min,max,init);
	return value;
}
EXPORT int readDigitalPin(int pin, int init, int id){
	firmataBoard* board = firmataBoardHandler::instance()->getBoard(id);
	int value = board->readDigitalPin(pin,init);
	return value;
}
EXPORT void writeAnalogPin(int pin, int id,double value){
	double croped = value>1.0?1.0:(value<0.0?0.0:value);
	firmataBoard* board = firmataBoardHandler::instance()->getBoard(id);
	board->writeAnalogPin(pin,(uint32_t)(croped*1023));
}
EXPORT void writeDigitalPin(int pin, int id,int value){
	firmataBoard* board = firmataBoardHandler::instance()->getBoard(id);
	board->writeDigitalPin(pin,value);
}
EXPORT void writeServoPin(int pin, int id,double value,int min,int max){
	double croped = value>1.0?1.0:(value<0.0?0.0:value);
	firmataBoard* board = firmataBoardHandler::instance()->getBoard(id);
	board->writeServoPin(pin,(int)(croped*180),min,max);
}

} // end extern "C"
