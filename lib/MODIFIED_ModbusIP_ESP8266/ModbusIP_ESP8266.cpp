/*
    ModbusIP_ESP8266.cpp - Source for Modbus IP ESP8266 Library
    Copyright (C) 2015 André Sarmento Barbosa
*/

#include "ModbusIP_ESP8266.h"

WiFiServer server(MODBUSIP_PORT);

//server.setTimeout(1000);

ModbusIP::ModbusIP() {
}

void ModbusIP::config() {

	server.begin();
	server.setNoDelay(true);
}

void ModbusIP::stop(){
	server.stop();
}

int ModbusIP::task()
{
	bool debug = 0;
	unsigned long ccm = 0;
	unsigned long cam = 0;
	unsigned long sam = 0;
	unsigned long rm = 0;
	unsigned long sm = 0;
	unsigned long em = 0;
	bool writeStatus = 0;

	enum  MBerrorStates {
		NO_CLIENT,
		GOOD_PACKET,
		BAD_PACKET,
		NOT_MODBUS_PKT,
		LARGE_FRAME,
		FAILED_WRITE,
	};

	const size_t bufferSize = 128;
	uint8_t rbuf[bufferSize] = {};
	int saveX = 0;
	int raw_len = 0;
	int x = 0;
	bool clientConnected = false;

	sm = micros();
	WiFiClient client = server.available();
	client.setTimeout(1000);
	client.setNoDelay(1);
	sam = micros();
	if (client)
	{
		for (x = 1; x < 270; x++)
		{
			if (debug) Serial.print(x);
			if (clientConnected==false) ccm = micros();
			clientConnected = true;
			saveX = x;
			if (debug) Serial.print(" cc ");
			if (client.available())
			{
				if (debug) Serial.print(" ca ");
				cam = micros();
				raw_len = client.available();
			}
			if (raw_len >= 12) break;
			rm = micros();
			delayMicroseconds(100);
			yield();
			delay(0);
			if (debug) Serial.print(" dn ");
		}
		client.flush();
		if (raw_len < 12 ){
			client.stop();
			return BAD_PACKET;
		}

		if (raw_len == 0 ){
		if (debug) Serial.println(x);
		if (debug) Serial.println(raw_len);
		}

		if (raw_len >= 12)
		{
			client.read(rbuf, raw_len);
			for (int i=0; i<7; i++)
			{
				_MBAP[i] = rbuf[i];
			} //Get MBAP

			_len = _MBAP[4] << 8 | _MBAP[5];
			_len--; // Do not count with last byte from MBAP

			if (_MBAP[2] !=0 || _MBAP[3] !=0){
				client.stop();
				return NOT_MODBUS_PKT;
			}   //Not a MODBUSIP packet
			if (_len > MODBUSIP_MAXFRAME){
				client.stop();
				return LARGE_FRAME;
			}      //Length is over MODBUSIP_MAXFRAME
			_frame = (byte*) malloc(_len);

			raw_len = raw_len - 7;
			for (int i=0; i< raw_len; i++)	_frame[i] = rbuf[i + 7]; //Get Modbus PDU

			this->receivePDU(_frame);

			if (_reply != MB_REPLY_OFF) {
			    //MBAP
				_MBAP[4] = (_len+1) >> 8;     //_len+1 for last byte from MBAP
				_MBAP[5] = (_len+1) & 0x00FF;

				size_t send_len = (unsigned int)_len + 7;
				uint8_t sbuf[send_len];
				for (int i=0; i<7; i++)	    sbuf[i] = _MBAP[i];
				for (int i=0; i<_len; i++)	sbuf[i+7] = _frame[i];
				writeStatus = client.write(sbuf, send_len);
				if (writeStatus == false) return FAILED_WRITE;
				if (debug) Serial.println(" wr ");
			}

			if (debug) Serial.print(" cs ");
			em = micros();

			free(_frame);
			_len = 0;
			if (debug) Serial.println(" fn ");
			clientConnected = false;
			client.stop();
			return GOOD_PACKET;
		}

		em = micros();
		if (clientConnected)
		{
			if (debug) Serial.print("SaveX:");
			if (debug) Serial.println(saveX);
			if (debug) Serial.print("SM:         ");    	//start
			if (debug) Serial.println(sm);
			if (debug) Serial.print("SAM:        ");    	//server avail
			if (debug) Serial.println(sam - sm);
			if (debug) Serial.print("CCM:        ");			//client connected
			if (debug) Serial.println(ccm - sm);
			if (debug) Serial.print("CAM:        ");			//client avaliable
			if (debug) Serial.println(cam - sm);
			if (debug) Serial.print("RM:         ");			//receive
		  if (debug) Serial.println(rm - sm);
			if (debug) Serial.print("EM:         ");   	//total send and recv
			if (debug) Serial.println(em - sm);
		}
		clientConnected = false;
		client.stop();
		return BAD_PACKET;
	}
	return NO_CLIENT;
}
