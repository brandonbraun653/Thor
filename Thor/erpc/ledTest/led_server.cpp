/*
 * Generated by erpcgen 1.6.0 on Mon May  7 13:51:33 2018.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#include "led_server.h"
#include <new>
#include "erpc_port.h"

#if 10600 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif


#if !defined(ERPC_GENERATED_CRC) || ERPC_GENERATED_CRC != 49684
#error "Macro 'ERPC_GENERATED_CRC' should be defined with value 49684."
#endif


using namespace erpc;
#if !(__embedded_cplusplus)
using namespace std;
#endif

#if ERPC_NESTED_CALLS_DETECTION
extern bool nestingDetection;
#endif

// for mdk/keil do not forgett add "--muldefweak" for linker
extern const uint32_t erpc_generated_crc;
#pragma weak erpc_generated_crc
extern const uint32_t erpc_generated_crc = 49684;

// Call the correct server shim based on method unique ID.
erpc_status_t IO_service::handleInvocation(uint32_t methodId, uint32_t sequence, Codec * codec, MessageBufferFactory *messageFactory)
{
    switch (methodId)
    {
        case kIO_turnGreenLEDON_id:
            return turnGreenLEDON_shim(codec, messageFactory, sequence);

        case kIO_turnGreenLEDOFF_id:
            return turnGreenLEDOFF_shim(codec, messageFactory, sequence);

        default:
            return kErpcStatus_InvalidArgument;
    }
}

// Server shim for turnGreenLEDON of IO interface.
erpc_status_t IO_service::turnGreenLEDON_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    // startReadMessage() was already called before this shim was invoked.


    // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
    nestingDetection = true;
#endif
    if (!err)
    {
        turnGreenLEDON();
    }
#if ERPC_NESTED_CALLS_DETECTION
    nestingDetection = false;
#endif

    // preparing MessageBuffer for serializing data
    if (!err)
    {
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    // preparing codec for serializing data
    codec->reset();

    // Build response message.
    if (!err)
    {
        err = codec->startWriteMessage(kReplyMessage, kIO_service_id, kIO_turnGreenLEDON_id, sequence);
    }



    return err;
}

// Server shim for turnGreenLEDOFF of IO interface.
erpc_status_t IO_service::turnGreenLEDOFF_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    // startReadMessage() was already called before this shim was invoked.


    // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
    nestingDetection = true;
#endif
    if (!err)
    {
        turnGreenLEDOFF();
    }
#if ERPC_NESTED_CALLS_DETECTION
    nestingDetection = false;
#endif

    // preparing MessageBuffer for serializing data
    if (!err)
    {
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    // preparing codec for serializing data
    codec->reset();

    // Build response message.
    if (!err)
    {
        err = codec->startWriteMessage(kReplyMessage, kIO_service_id, kIO_turnGreenLEDOFF_id, sequence);
    }



    return err;
}
erpc_service_t create_IO_service()
{
    return new (nothrow) IO_service();
}
