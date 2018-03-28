#pragma once
#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

namespace SmartBuffer
{
	template<typename T>
	class RingBuffer
	{
	public:

		int availableToRead()
		{
			return bufferedData;
		}
		
		int availableToWrite()
		{
			return bufferSize - bufferedData;
		}

		void flush()
		{
			if (!locked)
			{
				readLocation = 0;
				writeLocation = 0;
				bufferedData = 0;

				memset((void*)data_buffer, 0, bufferSize);
			}
		}

		int write(T data)
		{
			if (availableToWrite() && !locked)
			{
				data_buffer[writeLocation] = data;

				if ((0 <= bufferedData) && (bufferedData < bufferSize))
				{
					++bufferedData;
					writeLocation = (writeLocation + 1) % bufferSize;
				}

				return 1;
			}
			else
				return -1;
		}

		int write(T* data, size_t writeLength)
		{
			//For this one you need to write starting from
			//the last location and go until the buffer is full...
			//Calculate how much room is left, then do a memset. Be careful of going 
			//over the limit of the buffer size. Do a partial calculation and segment
			//off the data...probably will need to do two memsets total.

			/* Until the fancy version is working, do the simple for loop */
			volatile int totalDataWritten = 0;
			for (int i = 0; i < writeLength; i++)
			{
				if (write(data[i]) == -1)
					break;
				
				totalDataWritten += 1;
			}
			
			return totalDataWritten;
		}

		void read(T* output, size_t readLength)
		{
			/* Again, probably could do a fancy version with 2 memsets, BUT 
			   the for loop is simpler for the time being. */
			for (int i = 0; i < readLength; i++)
			{
				if (availableToRead() == 0)
					break;
				else
					output[i] = read();
			}
		}

		T read()
		{
			if (availableToRead() && !locked)
			{
				uint32_t tempIndex = readLocation;

				if ((0 < bufferedData) && (bufferedData <= bufferSize))
				{
					--bufferedData;
					readLocation = (readLocation + 1) % bufferSize;
				}

				return data_buffer[tempIndex];
			}
			else
				return (T)0;
		}

		T peek()
		{
			return data_buffer[readLocation];
		}
		
		/* Prevents reading/writing to the buffer */
		void lock()
		{
			locked = true;
		}
		
		/* Allows reading/writing to the buffer */
		void unlock()
		{
			locked = false;
		}
		
		RingBuffer(T* buffer, size_t bufferLength)
		{
			locked = false;
			
			writeLocation = 0;
			readLocation = 0;
			bufferedData = 0;

			bufferSize = bufferLength;
			data_buffer = buffer;

			memset((void *)data_buffer, 0, bufferSize);
		};
		
		~RingBuffer(){};
		
	private:
		static const int DEFAULT_BUFFER_SIZE = 16;
		volatile size_t bufferSize;
		volatile int readLocation;
		volatile int writeLocation;
		volatile int bufferedData;
		
		volatile bool locked;
		
		T* data_buffer;
	};

}



enum BufferType
{
	BUFFER_TYPE_STRING,
	BUFFER_TYPE_UINT8_T,
	BUFFER_TYPE_INT,
	BUFFER_TYPE_FLOAT,
	BUFFER_TYPE_DOUBLE,
	BUFFER_TYPE_CHAR
};

/* Legacy. Remove.*/
class RingBufferClass {
public:

	/*----------------------------
	* Constructors
	*----------------------------*/
	RingBufferClass(void);
	RingBufferClass(uint8_t *buffer, int bufferLength);
	RingBufferClass(int *buffer, int bufferLength);
	RingBufferClass(char **buffer, int bufferLength);

	/*----------------------------
	* Generic functions that apply to all buffer types
	*----------------------------*/
	uint32_t bufferType;
	int	availableToRead();
	int	availableToWrite();
	void flush();

	/*----------------------------
	* Unsigned 8-bit Integer Buffer
	*----------------------------*/
	int uint8_t_write(uint8_t data);
	int uint8_t_write(uint8_t *dataBuffer, int writeLength);
	uint8_t uint8_t_read();
	uint8_t uint8_t_peek();

	/*----------------------------
	* Integer Buffer
	*----------------------------*/
	int	int_write(int data);
	int	int_write(int *dataBuffer, int writeLength);
	int	int_read();
	int	int_peek();
	

	/*----------------------------
	* Char Pointer (string) Buffer
	*----------------------------*/
	int string_write(char *string);
	char *string_read();

	
private:
	
	static const int DEFAULT_BUFFER_SIZE = 16;
	volatile int bufferSize;
	volatile int readLocation;
	volatile int writeLocation;
	volatile int bufferedData;

	uint8_t *uint8_tBuffer;
	int *intBuffer;
	char **stringBuffer;
	
};

#endif // !__RINGBUFFER_H__
