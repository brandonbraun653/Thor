/******************************************************************************
 *  File Name:
 *    etl_profile.h
 *
 *  Description:
 *    Feature configuration for the Embedded Template Library (ETL)
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_ETL_CONFIG_H
#define THOR_ETL_CONFIG_H

/*-------------------------------------------------------------------------------
Generic C++17 without STL support, copied from etl/profiles directory
-------------------------------------------------------------------------------*/
//#define ETL_TARGET_DEVICE_GENERIC
//#define ETL_TARGET_OS_NONE
//#define ETL_COMPILER_GENERIC
#define ETL_CPP11_SUPPORTED 1
#define ETL_CPP14_SUPPORTED 1
#define ETL_CPP17_SUPPORTED 1
#define ETL_NO_NULLPTR_SUPPORT 0
#define ETL_NO_LARGE_CHAR_SUPPORT 0
#define ETL_CPP11_TYPE_TRAITS_IS_TRIVIAL_SUPPORTED 0

#endif  /* !THOR_ETL_CONFIG_H */
