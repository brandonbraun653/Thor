#include <Thor/boostStub.hpp>
#include <Thor/exceptions.hpp>

namespace boost
{
#if defined( BOOST_NO_EXCEPTIONS )
  void throw_exception( std::exception const &e )
  {
    while ( 1 ) {}
  }
#endif
}    // namespace boost
