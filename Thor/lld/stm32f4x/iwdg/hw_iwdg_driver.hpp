/********************************************************************************
 *  File Name:
 *    hw_iwdg_driver.hpp
 *
 *  Description:
 *    Declares the low level hardware watchdog driver interface
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_DRIVER_HPP
#define THOR_HW_IWDG_DRIVER_HPP

/* C++ Includes */
#include <memory>

/* Driver Includes */
#include <Thor/lld/interface/watchdog/watchdog_model.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_types.hpp>

namespace Thor::LLD::IWDG
{
  /**
   *  Initializes the low level driver
   *
   *  @return void
   */
  void initialize();

  class Driver : public Thor::LLD::Watchdog::Basic
  {
  public:
    /**
     *  Low level driver constructor
     *
     *  @param[in]  periph      Memory mapped structure of the IWDG to be controlled
     */
    Driver( RegisterMap *const periph );
    ~Driver();

    Reg32_t calculatePrescaler( const size_t ms ) override;

    Reg32_t calculateReload( const size_t ms, const Reg32_t prescaler ) override;

    Chimera::Status_t setPrescaler( const Reg32_t val ) override;

    Chimera::Status_t setReload( const Reg32_t val ) override;

    void start() final override;

    void reload() final override;

    void enableClock() final override;

    size_t getMaxTimeout( const Reg32_t prescaler ) override;

    size_t getMinTimeout( const Reg32_t prescaler ) override;

    size_t getTimeout() final override;

  private:
    RegisterMap *const periph;
    size_t resourceIndex;
  };

  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_DRIVER_HPP */