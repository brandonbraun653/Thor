/********************************************************************************
 *   File Name:
 *    hw_iwdg_driver.hpp
 *
 *   Description:
 *    Declares the low level hardware watchdog driver interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_DRIVER_HPP
#define THOR_HW_IWDG_DRIVER_HPP

/* C++ Includes */
#include <memory>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/model/watchdog_model.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_IWDG == 1 )
namespace Thor::Driver::IWDG
{
  class Driver : public Thor::Driver::Watchdog::Basic
  {
  public:
    /**
     *  Low level driver constructor
     *
     *  @param[in]  periph      Memory mapped structure of the IWDG to be controlled
     */
    Driver( RegisterMap *const periph );
    ~Driver();

    uint32_t calculatePrescaler( const size_t ms ) final override;

    uint32_t calculateReload( const size_t ms, const uint32_t prescaler ) final override;

    Chimera::Status_t setPrescaler( const uint32_t val ) final override;

    Chimera::Status_t setReload( const uint32_t val ) final override;

    void start() final override;

    void reload() final override;

    void enableClock() final override;

    size_t getMaxTimeout( const uint32_t prescaler ) final override;

    size_t getMinTimeout( const uint32_t prescaler ) final override;

    size_t getTimeout() final override;

  private:
    RegisterMap *const periph;
    size_t resourceIndex;
  };

  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;

}    // namespace Thor::Driver::IWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_IWDG_DRIVER_HPP */