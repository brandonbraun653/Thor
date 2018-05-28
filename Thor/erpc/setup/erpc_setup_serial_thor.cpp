#include "erpc_transport_setup.h"
#include "manually_constructed.h"
#include "serial_thor_transport.h"

using namespace erpc;

static ManuallyConstructed<SerialTransport> s_transport;

erpc_transport_t erpc_transport_thor_serial_init(int channel)
{
	s_transport.construct(channel);
	s_transport->init();
	return reinterpret_cast<erpc_transport_t>(s_transport.get());
}