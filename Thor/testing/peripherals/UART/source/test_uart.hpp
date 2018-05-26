#pragma once
#ifndef TEST_UART_HPP
#define TEST_UART_HPP

/* Thor Includes */
#include <Thor/include/thor.hpp>
#include <Thor/include/nucleo.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/serial.hpp>
#include <Thor/include/threads.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Project Includes */
#include "test_uart_settings.hpp"

/* Simple status led that blinks. Lets the user know if a crash occurred. */
extern void ledTask(void* arguments);

/* Handles polling the rpc server when new data arrives */
extern void serverTask(void* arguments);

extern void setupRPCServer();

#endif /* !TEST_UART_HPP */