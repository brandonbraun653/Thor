/******************************************************************************
 *  File Name:
 *    sys_call_stubs.c
 *
 *  Description:
 *    Stubs for missing system calls
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <reent.h>

/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

  int _close_r( struct _reent *ptr, int fd )
  {
    RT_DBG_ASSERT( false );
    return -1;
  }

  int _fstat_r( struct _reent *ptr, int fd, struct stat *pstat )
  {
    RT_DBG_ASSERT( false );
    return -1;
  }

  int _getpid_r( struct _reent *ptr )
  {
    RT_DBG_ASSERT( false );
    return -1;
  }

  int _isatty_r( struct _reent *ptr, int fd )
  {
    RT_DBG_ASSERT( false );
    return -1;
  }

  int _kill_r( struct _reent *ptr, int pid, int sig )
  {
    RT_DBG_ASSERT( false );
    return -1;
  }

  _off_t _lseek_r( struct _reent *ptr, int fd, _off_t pos, int whence )
  {
    RT_DBG_ASSERT( false );
    return -1;
  }

  _ssize_t _read_r( struct _reent *ptr, int fd, void *buf, size_t cnt )
  {
    RT_DBG_ASSERT( false );
    return -1;
  }
#ifdef __cplusplus
} /* extern "C" */
#endif
