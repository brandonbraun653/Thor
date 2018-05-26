/*
 * Generated by erpcgen 1.6.0 on Sat May 26 15:28:13 2018.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#include "client_manager.h"
#include "erpc_port.h"
extern "C"
{
#include "uart_rpc.h"
}

#if 10600 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif


#if !defined(ERPC_GENERATED_CRC) || ERPC_GENERATED_CRC != 1785
#error "Macro 'ERPC_GENERATED_CRC' should be defined with value 1785."
#endif


using namespace erpc;
#if !(__embedded_cplusplus)
using namespace std;
#endif

extern ClientManager *g_client;
// for mdk/keil do not forgett add "--muldefweak" for linker
extern const uint32_t erpc_generated_crc;
#pragma weak erpc_generated_crc
extern const uint32_t erpc_generated_crc = 1785;


// IO interface turnGreenLEDON function client shim.
void turnGreenLEDON(void)
{
    erpc_status_t err = kErpcStatus_Success;

    // Get a new request.
    RequestContext request = g_client->createRequest(false);

    // Encode the request.
    Codec * codec = request.getCodec();
    if (codec == NULL)
    {
        err = kErpcStatus_MemoryError;
    }

    if (!err)
    {
        err = codec->startWriteMessage(kInvocationMessage, kIO_service_id, kIO_turnGreenLEDON_id, request.getSequence());
    }

    // Send message to server
    if (!err)
    {
        err = g_client->performRequest(request);
    }



    // Dispose of the request.
    g_client->releaseRequest(request);

    if (err)
    {
        g_client->callErrorHandler(err);
        
    }

    return;
}

// IO interface turnGreenLEDOFF function client shim.
void turnGreenLEDOFF(void)
{
    erpc_status_t err = kErpcStatus_Success;

    // Get a new request.
    RequestContext request = g_client->createRequest(false);

    // Encode the request.
    Codec * codec = request.getCodec();
    if (codec == NULL)
    {
        err = kErpcStatus_MemoryError;
    }

    if (!err)
    {
        err = codec->startWriteMessage(kInvocationMessage, kIO_service_id, kIO_turnGreenLEDOFF_id, request.getSequence());
    }

    // Send message to server
    if (!err)
    {
        err = g_client->performRequest(request);
    }



    // Dispose of the request.
    g_client->releaseRequest(request);

    if (err)
    {
        g_client->callErrorHandler(err);
        
    }

    return;
}
