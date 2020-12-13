/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * GPIODriver.h
 * Uses data in a DMXBuffer to drive GPIO pins.
 * Copyright (C) 2014 Simon Newton
 */

#ifndef PLUGINS_GPIO_GPIODRIVER_H_
#define PLUGINS_GPIO_GPIODRIVER_H_

#include <stdint.h>
#include <ola/DmxBuffer.h>
#include <ola/base/Macro.h>
#include <ola/thread/Thread.h>

#include <string>
#include <vector>

namespace ola {
namespace plugin {
namespace gpio {

/**
 * @brief Uses data in a DMXBuffer to drive GPIO pins.
 */
class GPIODriver : private ola::thread::Thread {
 public:
  /**
   * @brief The Options.
   */
  struct Options {
   public:
    Options(): start_address(1), pwm_frequency(1000) {}

    /**
     * @brief A list of I/O pins to map to slots.
     */
    std::vector<uint16_t> gpio_pins;

    /**
     * @brief The DMX512 start address of the first pin
     */
    uint16_t start_address;

    /**
     * @brief The PWM frequency controlling gpio pins
     */
    uint16_t pwm_frequency;

    /**
     * @brief The IP address used to connect to pigpiod
     */
    std::string pi_address;

    /**
     * @brief The port used to connect to pigpiod
     */
    uint16_t pi_port;
  };

  /**
   * @brief Create a new GPIODriver.
   * @param options the Options struct.
   */
  explicit GPIODriver(const Options &options);

  /**
   * @brief Destructor.
   */
  ~GPIODriver();

  /**
   * @brief Initialize the GPIODriver.
   * @returns true is successful, false otherwise.
   */
  bool Init();

  /**
   * @brief Get a list of the GPIO pins controlled by this driver.
   * @returns A list of GPIO pin numbers.
   */
  std::vector<uint16_t> PinList() const { return m_options.gpio_pins; }

  /**
   * @brief Set the values of the GPIO pins from the data in the DMXBuffer.
   * @param dmx the DmxBuffer with the values to use.
   * @returns true is the GPIO pins were updated.
   */
  bool SendDmx(const DmxBuffer &dmx);

  void *Run();

 private:
  struct GPIOPin {
    uint16_t pin;
    uint16_t dmx_slot;
    uint8_t dmx_level;
    uint16_t duty_cycle; //should always match dmx level, but can be as large as range
    int frequency; // TODO allow custom frequency per pin
    // int range; // TODO allow custom range per pin
  };

  typedef std::vector<GPIOPin> GPIOPins;
  int pi_id;

  const Options m_options;
  DmxBuffer m_buffer;
  bool m_term;  // GUARDED_BY(m_mutex);
  bool m_dmx_changed;  // GUARDED_BY(m_mutex);
  ola::thread::Mutex m_mutex;
  ola::thread::ConditionVariable m_cond;

  bool SetupGPIO();
  bool UpdateGPIOPins(const DmxBuffer &dmx);

  DISALLOW_COPY_AND_ASSIGN(GPIODriver);
};
}  // namespace gpio
}  // namespace plugin
}  // namespace ola
#endif  // PLUGINS_GPIO_GPIODRIVER_H_
