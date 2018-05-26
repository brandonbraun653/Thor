/*
 * Generated by erpcgen 1.6.0 on Sat May 26 15:28:13 2018.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#if !defined(_uart_rpc_server_h_)
#define _uart_rpc_server_h_

#ifdef __cplusplus
#include "server.h"
extern "C"
{
#include "uart_rpc.h"
#include <stdint.h>
#include <stdbool.h>
}

#if 10600 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif


#if !defined(ERPC_GENERATED_CRC) || ERPC_GENERATED_CRC != 1785
#error "Macro 'ERPC_GENERATED_CRC' should be defined with value 1785."
#endif



/*!
 * @brief Service subclass for IO.
 */
class IO_service : public erpc::Service
{
public:
    IO_service() : Service(kIO_service_id) {}

    /*! @brief Call the correct server shim based on method unique ID. */
    virtual erpc_status_t handleInvocation(uint32_t methodId, uint32_t sequence, erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory);

private:
    /*! @brief Server shim for turnGreenLEDON of IO interface. */
    erpc_status_t turnGreenLEDON_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);

    /*! @brief Server shim for turnGreenLEDOFF of IO interface. */
    erpc_status_t turnGreenLEDOFF_shim(erpc::Codec * codec, erpc::MessageBufferFactory *messageFactory, uint32_t sequence);
};

extern "C" {
#else
#include "uart_rpc.h"
#endif // __cplusplus

typedef void * erpc_service_t;

erpc_service_t create_IO_service(void);
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _uart_rpc_server_h_
