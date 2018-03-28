/*
 * ringbuffer.cpp
 *
 * Created: 12/18/2015 5:29:18 PM
 *  Author: Brandon
 */ 

#include "../include/ringbuffer.h"



//Constructor -- Unused except for friend classes
RingBufferClass::RingBufferClass(void) {}

//Constructor -- Used for object instantiation
RingBufferClass::RingBufferClass(uint8_t *buffer, int bufferLength) {
	writeLocation = 0;
	readLocation = 0;
	bufferedData = 0;

	bufferSize = bufferLength;
	uint8_tBuffer = buffer;
	bufferType = BUFFER_TYPE_UINT8_T;

	memset((void *)uint8_tBuffer, 0, bufferSize);
}

RingBufferClass::RingBufferClass(int *buffer, int bufferLength) {
	writeLocation = 0;
	readLocation = 0;
	bufferedData = 0;
	
	bufferSize = bufferLength;
	intBuffer = buffer;
	bufferType = BUFFER_TYPE_INT;

	memset((void *)intBuffer, 0, bufferSize);
}

RingBufferClass::RingBufferClass(char **buffer, int bufferLength)
{
	writeLocation = 0;
	readLocation = 0;
	bufferedData = 0;

	bufferSize = bufferLength;
	stringBuffer = buffer;
	bufferType = BUFFER_TYPE_STRING;

	memset((void *)stringBuffer, 0, bufferSize);
}

int RingBufferClass::availableToRead() {
	return bufferedData;
}

int RingBufferClass::availableToWrite() {
	return bufferSize - bufferedData;
}

void RingBufferClass::flush() {
	readLocation = 0;
	writeLocation = 0;
	bufferedData = 0;

	switch (bufferType)
	{
	case BUFFER_TYPE_UINT8_T:
		memset((void *)uint8_tBuffer, 0, bufferSize);
		break;

	case BUFFER_TYPE_INT:
		memset((void *)intBuffer, 0, bufferSize);
		break;

	case BUFFER_TYPE_STRING:	
		memset((void *)stringBuffer, 0, bufferSize);
	}
	
}


/*----------------------------
* Unsigned 8-bit Integer Buffer
*----------------------------*/
int RingBufferClass::uint8_t_write(uint8_t data)
{
	if (this->availableToWrite())
	{
		uint8_tBuffer[writeLocation] = data;

		if ((0 <= bufferedData) && (bufferedData < bufferSize))
		{
			++bufferedData;
			writeLocation = (writeLocation + 1) % bufferSize;
		}

		return 1; //Success	
	}
	else
		return -1; //Buffer full
}

int RingBufferClass::uint8_t_write(uint8_t *dataBuffer, int writeLength)
{
	flush();

	if (writeLength > bufferSize)
		writeLength = bufferSize;

	memcpy((void*)uint8_tBuffer, dataBuffer, writeLength);

	writeLocation = writeLength;
	bufferedData = writeLength;
}

uint8_t RingBufferClass::uint8_t_read()
{
	if (this->availableToRead())
	{
		uint32_t tempIndex = readLocation;

		if ((0 < bufferedData) && (bufferedData <= bufferSize))
		{
			--bufferedData;
			readLocation = (readLocation + 1) % bufferSize;
		}
		return uint8_tBuffer[tempIndex];
	}
	else
		return -1; //Buffer empty
}

uint8_t RingBufferClass::uint8_t_peek()
{
	return uint8_tBuffer[readLocation];
}




/*----------------------------
* Integer Buffer
*----------------------------*/
int RingBufferClass::int_write(int data) 
{
	if (this->availableToWrite()) 
	{
		intBuffer[writeLocation] = data;
		
		//Make sure the buffer fill count can't overrun bounds
		if ((0 <= bufferedData) && (bufferedData < bufferSize)) 
		{
			++bufferedData;
			writeLocation = (writeLocation + 1) % bufferSize;
		}
		
		return 1; //Success	
	}
	else
		return -1; //Buffer full
}

int RingBufferClass::int_write(int *dataBuffer, int writeLength)
{
	flush();

	if (writeLength > bufferSize)
		writeLength = bufferSize;

	memcpy((void*)intBuffer, dataBuffer, writeLength);

	writeLocation = writeLength;
	bufferedData = writeLength;
}

int RingBufferClass::int_read() 
{
	if (this->availableToRead()) 
	{
		uint32_t tempIndex = readLocation;
		
		if ((0 < bufferedData) && (bufferedData <= bufferSize)) 
		{
			--bufferedData;
			readLocation = (readLocation + 1) % bufferSize;
		}
		return intBuffer[tempIndex];
	}
	else
		return -1; //Buffer empty
}

int RingBufferClass::int_peek() 
{
	return intBuffer[readLocation];
}


/*----------------------------
* Char Pointer (string) Buffer
*----------------------------*/
int RingBufferClass::string_write(char *string)
{
	if (this->availableToWrite())
	{
		// Write at the current location pointer
		stringBuffer[writeLocation] = string;

		//Make sure the buffer fill count can't overrun bounds
		if ((0 <= bufferedData) && (bufferedData < bufferSize)) {
			++bufferedData;
			writeLocation = (writeLocation + 1) % bufferSize;
		}

		return 1;
	}
	else
		return -1;
}

char *RingBufferClass::string_read()
{
	if (this->availableToRead()) 
	{
		uint32_t tempIndex = readLocation;

		if ((0 < bufferedData) && (bufferedData <= bufferSize)) {
			--bufferedData;
			readLocation = (readLocation + 1) % bufferSize;
		}
		return stringBuffer[tempIndex];
	}
	else
		return nullptr;
}