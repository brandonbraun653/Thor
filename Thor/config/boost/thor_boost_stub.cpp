/********************************************************************************
*  File Name:
*    thor_boost_stub.cpp
*
*  Description:
*    Stubs for boost so we can compile
*
*  2020 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#include <exception>
#include <Thor/config/boost/boostStub.hpp>

namespace boost
{
#if defined( BOOST_NO_EXCEPTIONS )
  void throw_exception( std::exception const &e )
  {
    while ( 1 ) {}
  }
#endif
}    // namespace boost
