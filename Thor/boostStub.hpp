#ifndef BOOST_STUB_HPP
#define BOOST_STUB_HPP

#include <exception>

namespace boost
{
#if defined( BOOST_NO_EXCEPTIONS )
  extern void throw_exception( std::exception const &e );
#endif

}    // namespace boost


#endif /* !BOOST_STUB_HPP */
