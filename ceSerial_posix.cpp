// File: ceSerial.h
// Description: ceSerial communication class for Windows and Linux
// WebSite: http://cool-emerald.blogspot.sg/2017/05/serial-port-programming-in-c-with.html
// MIT License (https://opensource.org/licenses/MIT)
// Copyright (c) 2018 Yan Naing Aye

// References
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// http://www.silabs.com/documents/public/application-notes/an197.pdf
// https://msdn.microsoft.com/en-us/library/ff802693.aspx
// http://www.cplusplus.com/forum/unices/10491/

#include "ceSerial.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>


void ceSerial::Delay(unsigned long ms) {
	usleep(ms*1000);
}

ceSerial::ceSerial() :
	ceSerial("/dev/ttyS0", 9600, 8, 'N', 1)	
{
}

ceSerial::ceSerial(std::string Device, long BaudRate,long DataSize,char ParityType,float NStopBits):stdbaud(true) {
	fd = -1;
	SetBaudRate(BaudRate);
	SetDataSize(DataSize);
	SetParity(ParityType);
	SetStopBits(NStopBits);
	SetPortName(Device);
	SetFlowControl(FlowControl_None);
}

ceSerial::~ceSerial() {
	Close();
}

void ceSerial::SetPortName(std::string Device) {
	port = Device;
}

std::string ceSerial::GetPort() {
	return port;
}

void ceSerial::SetDataSize(long nbits) {
	if ((nbits < 5) || (nbits > 8)) nbits = 8;
	dsize=nbits;
}

long ceSerial::GetDataSize() {
	return dsize;
}

void ceSerial::SetParity(char p) {
	if ((p != 'N') && (p != 'E') && (p != 'O')) {
		p = 'N';
	}
	parity = p;
}

char ceSerial::GetParity() {
	return parity;
}

void ceSerial::SetStopBits(float nbits) {
	if (nbits >= 2) stopbits = 2;
	else stopbits = 1;
}

float ceSerial::GetStopBits() {
	return stopbits;
}

void ceSerial::SetFlowControl(ceSerial::FlowControl flow)
{
	flowControl = flow;
}

ceSerial::FlowControl ceSerial::GetFlowControl()
{
	return flowControl;
}


long ceSerial::Open(void) 
{
	fd = open(port.c_str(), O_RDWR | O_NONBLOCK);
	if (fd == -1) 
	{
		return errno;
	}

	struct serial_struct serinfo;
	memset(&serinfo, 0, sizeof(serinfo));
	struct termios settings;
	memset(&settings, 0, sizeof(settings));
	if (tcgetattr(fd, &settings) < 0)
	{
		return errno;
	}

	settings.c_iflag = 0;
	settings.c_oflag = 0;

	settings.c_cflag = CREAD | CLOCAL;//see termios.h for more information
	settings.c_cflag &= ~CSIZE;
	switch (dsize)
	{
		case 5:  settings.c_cflag |= CS5;	break;	//no change
		case 6:  settings.c_cflag |= CS6;	break;
		case 7:  settings.c_cflag |= CS7;	break;
		default: settings.c_cflag |= CS8;	break;
	}

	if(stopbits==2) settings.c_cflag |= CSTOPB;

	if(parity!='N') settings.c_cflag |= PARENB;

	if (parity == 'O') settings.c_cflag |= PARODD;

	settings.c_lflag = 0;
	settings.c_cc[VMIN] = 1;
	settings.c_cc[VTIME] = 0;

	if (!stdbaud) 
	{
		// serial driver to interpret the value B38400 differently		
		serinfo.reserved_char[0] = 0;
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {	return errno;}
		serinfo.flags &= ~ASYNC_SPD_MASK;
		serinfo.flags |= ASYNC_SPD_CUST;
		serinfo.custom_divisor = (serinfo.baud_base + (baud / 2)) / baud;
		if (serinfo.custom_divisor < 1) serinfo.custom_divisor = 1;
		if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0) { return errno; }
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) { return errno; }
		if (serinfo.custom_divisor * baud != serinfo.baud_base) {
			/*
			warnx("actual baudrate is %d / %d = %f\n",
				serinfo.baud_base, serinfo.custom_divisor,
				(float)serinfo.baud_base / serinfo.custom_divisor);
			*/
		}
		cfsetospeed(&settings, B38400);
		cfsetispeed(&settings, B38400);
	}
	else 
	{
		cfsetospeed(&settings, baud);
		cfsetispeed(&settings, baud);
	}	

	switch (flowControl)
	{
		default:
			// fall through to 'None'

		case FlowControl_None:
			settings.c_cflag &= ~CRTSCTS;
			break;

		case FlowControl_Software:		// XON, XOFF
			settings.c_cflag |= IXON | IXOFF | IXANY;
			break;

		case FlowControl_Hardware:		// RTS, CTS
			settings.c_cflag |= CRTSCTS;
			break;
	}

	tcflush(fd, TCIFLUSH);
	cfmakeraw(&settings);

	if (tcsetattr(fd, TCSANOW, &settings) < 0)
	{
		return errno;
	}

	int flags = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);

	if (!stdbaud) {
		// driver to interpret B38400 as 38400 baud again
		ioctl(fd, TIOCGSERIAL, &serinfo);
		serinfo.flags &= ~ASYNC_SPD_MASK;
		ioctl(fd, TIOCSSERIAL, &serinfo);
	}
	return 0;
}

void ceSerial::Close() {
	if(IsOpened()) close(fd);
	fd=-1;
}

bool ceSerial::IsOpened() {
	if(fd== (-1)) return false;
	else return true;
}

void ceSerial::SetBaudRate(long baudrate) {
	stdbaud = true;

    switch (baudrate)
    {
        case 0: baud = B0; return;
        case 50: baud = B50; return;
        case 75: baud = B75; return;
        case 110: baud = B110; return;
        case 134: baud = B134; return;
        case 150: baud = B150; return;
        case 200: baud = B200; return;
        case 300: baud = B300; return;
        case 600: baud = B600; return;
        case 1200: baud = B1200; return;
        case 2400: baud = B2400; return;
        case 4800: baud = B4800; return;
        case 9600: baud = B9600; return;
        case 19200: baud = B19200; return;
        case 38400: baud = B38400; return;
        case 57600: baud = B57600; return;
        case 115200: baud = B115200; return;
        case 230400: baud = B230400; return;

        // Extra output baud rates (not in POSIX).  termios-baud.h
        case 460800: baud = B460800; return;
        case 500000: baud = B500000; return;
        case 576000: baud = B576000; return;
        case 921600: baud = B921600; return;
        case 1000000: baud = B1000000; return;
        case 1152000: baud = B1152000; return;
        case 1500000: baud = B1500000; return;
        case 2000000: baud = B2000000; return;
        case 2500000: baud = B2500000; return;
        case 3000000: baud = B3000000; return;
        case 3500000: baud = B3500000; return;
        case 4000000: baud = B4000000; return;
        
        default:
                baud = baudrate;
                stdbaud = false;
                return;
	}
}

long ceSerial::GetBaudRate() {
	return baud;
}

bool ceSerial::ReadChar(char& ch) 
{
	return IsOpened() && read(fd, &ch, 1) == 1;
}

bool ceSerial::WriteChar(char ch) 
{
	return IsOpened() && write(fd, &ch, 1) == 1;
}

long ceSerial::Write(char *data, long n) 
{
	if (!IsOpened() || n < 0 || n > 1024) 
	{
		return -1;
	}

	// wait buffer is flushed, could not observe difference though
	int iRc = tcdrain(fd);
	if (iRc != 0)
	{
		return -1;
	}

	return write(fd, data, n);
}


bool ceSerial::SetRTS(bool value) {
	long RTS_flag = TIOCM_RTS;
	bool success=true;
	if (value) {//Set RTS pin
		if (ioctl(fd, TIOCMBIS, &RTS_flag) == -1) success=false;
	}
	else {//Clear RTS pin
		if (ioctl(fd, TIOCMBIC, &RTS_flag) == -1) success=false;
	}
	return success;
}

bool ceSerial::SetDTR(bool value) {
	long DTR_flag = TIOCM_DTR;
	bool success=true;
	if (value) {//Set DTR pin
		if (ioctl(fd, TIOCMBIS, &DTR_flag) == -1) success=false;
	}
	else {//Clear DTR pin
		if (ioctl(fd, TIOCMBIC, &DTR_flag) == -1) success=false;
	}
	return success;
}

bool ceSerial::GetCTS(bool& success) {
	success=true;
	long status;
	if(ioctl(fd, TIOCMGET, &status)== -1) success=false;
	return ((status & TIOCM_CTS) != 0);
}

bool ceSerial::GetDSR(bool& success) {
	success=true;
	long status;
	if(ioctl(fd, TIOCMGET, &status)== -1) success=false;
	return ((status & TIOCM_DSR) != 0);
}

bool ceSerial::GetRI(bool& success) {
	success=true;
	long status;
	if(ioctl(fd, TIOCMGET, &status)== -1) success=false;
	return ((status & TIOCM_RI) != 0);
}

bool ceSerial::GetCD(bool& success) {
	success=true;
	long status;
	if(ioctl(fd, TIOCMGET, &status)== -1) success=false;
	return ((status & TIOCM_CD) != 0);
}

