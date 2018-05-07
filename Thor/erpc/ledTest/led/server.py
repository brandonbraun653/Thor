#
# Generated by erpcgen 1.6.0 on Mon May  7 13:54:12 2018.
#
# AUTOGENERATED - DO NOT EDIT
#

import erpc
from . import common, interface

# Client for IO
class IOService(erpc.server.Service):
    def __init__(self, handler):
        super(IOService, self).__init__(interface.IIO.SERVICE_ID)
        self._handler = handler
        self._methods = {
                interface.IIO.TURNGREENLEDON_ID: self._handle_turnGreenLEDON,
                interface.IIO.TURNGREENLEDOFF_ID: self._handle_turnGreenLEDOFF,
            }

    def _handle_turnGreenLEDON(self, sequence, codec):
        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.turnGreenLEDON()

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.IIO.SERVICE_ID,
            request=interface.IIO.TURNGREENLEDON_ID,
            sequence=sequence))

    def _handle_turnGreenLEDOFF(self, sequence, codec):
        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.turnGreenLEDOFF()

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.IIO.SERVICE_ID,
            request=interface.IIO.TURNGREENLEDOFF_ID,
            sequence=sequence))


