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
 * GPIODriver.cpp
 * Uses data in a DMXBuffer to drive GPIO pins.
 * Copyright (C) 2014 Simon Newton
 */

#include "plugins/gpio/GPIODriver.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pigpiod_if2.h> // requires pigpiod daemon to be installed

#include <sstream>
#include <string>
#include <vector>

#include "ola/io/IOUtils.h"
#include "ola/Logging.h"
#include "ola/thread/Mutex.h"

namespace ola {
namespace plugin {
namespace gpio {

using ola::thread::MutexLocker;
using std::string;
using std::vector;

/*By default pigpio uses a sample rate of 5 microseconds using PCM.
At the default sample rate only the following PWM frequencies (Hz) are available:
8000, 4000, 2000, 1600, 1000, 800, 500, 400, 320,
250, 200, 160, 100, 80, 50, 40, 20, 10
When the frequency is set, it will be rounded to the nearest available frequency
*/
const int PWM_MIN_FREQUENCY = 10;
const int PWM_MAX_FREQUENCY = 8000;
const int PI_MIN_PORT = 1024;
const int PI_MAX_PORT = 49151;
const uint8_t DUTY_CYCLE_RANGE = 255;

GPIODriver::GPIODriver(const Options &options)
    : m_options(options),
      m_term(false),
      m_dmx_changed(false) {
        string* p_address = NULL;
        uint16_t* p_port = NULL;
        if (!m_options.pi_address.empty()) {
          p_address = &m_options.pi_address;
        }
        if (!m_options.pi_port < PI_MIN_PORT) {
          p_address = &m_options.pi_port;
        }

        pi_id = gpio_start(p_address, p_port);
}

GPIODriver::~GPIODriver() {
  {
    MutexLocker locker(&m_mutex);
    m_term = true;
  }
  m_cond.Signal();
  Join();

  gpio_stop(pi_id);
}

bool GPIODriver::Init() {
  if (!SetupGPIO()) {
    return false;
  }

  return Start();
}

bool GPIODriver::SendDmx(const DmxBuffer &dmx) {
  {
    MutexLocker locker(&m_mutex);
    // avoid the reference counting
    m_buffer.Set(dmx);
    m_dmx_changed = true;
  }
  m_cond.Signal();
  return true;
}

void *GPIODriver::Run() {
  Clock clock;
  DmxBuffer output;

  while (true) {
    bool update_pins = false;

    TimeStamp wake_up;
    // Use real time here because wake_up is passed to pthread_cond_timedwait
    clock.CurrentRealTime(&wake_up);
    wake_up += TimeInterval(1, 0);

    // Wait for one of: i) termination ii) DMX changed iii) timeout
    m_mutex.Lock();
    if (!m_term && !m_dmx_changed) {
      m_cond.TimedWait(&m_mutex, wake_up);
    }

    if (m_term) {
      m_mutex.Unlock();
      break;
    } else if (m_dmx_changed) {
      output.Set(m_buffer);
      m_dmx_changed = false;
      update_pins = true;
    }
    m_mutex.Unlock();
    if (update_pins) {
      UpdateGPIOPins(output);
    }
  }
  return NULL;
}

bool GPIODriver::SetupGPIO() {

  int mode_result = -1; // assume the worst
  int range_result = -1; // assume the worst
  int freq_result = -1; // assume the worst

  uint16_t current_slot = m_options.start_address - 1;
  GPIOPin new_pin;

  vector<uint16_t>::const_iterator p = m_options.gpio_pins.begin();
  for (; p != m_options.gpio_pins.end(); ++p) {
    mode_result = set_mode(pi_id, *p, PI_OUTPUT);
    if (mode_result < 0) {
      OLA_WARN << "Could not set pin " << *p << " to output mode, error: " << mode_result;
      continue;
    }

    // TODO Not sure if the following is needed/desired
    // gpioSetPullUpDown(static_cast<int>(*iter), PI_PUD_DOWN);
    // could result in errors PI_BAD_GPIO or PI_BAD_PUD

    // Turn everything off at first
    gpio_write(pi_id, *p, 0)

    //TODO - allow individual ranges for each pin
    range_result = set_PWM_range(pi_id, *p, DUTY_CYCLE_RANGE)
    if (range_result < 0) {
      OLA_WARN << "Could not set range on pin " << *p << " error: " << range_result;
      continue;
    }

    //TODO - allow individual frequencies for each pin
    freq_result = set_PWM_frequency(pi_id, *p, m_options.pwm_frequency)
    if (freq_result < 0) {
      OLA_WARN << "Could not set frequency on pin " << *p << " error: " << freq_result;
      continue;
    }

    // Do we have a valid slot to assign?
    if (current_slot > DMX_UNIVERSE_SIZE) {
      OLA_WARN << "Maximum DMX slot exceeded for pin " << *p;
      return true;
    }

    // If we made it this far, we have a valid pin and we can add it to the vector
    new_pin.pin = p;
    new_pin.dmx_slot = current_slot;
    new_pin.frequency = freq_result;

    GPIOPins.push_back(new_pin);
  }

  return true;
}

bool GPIODriver::UpdateGPIOPins(const DmxBuffer &dmx) {

  const uint16_t first_slot = m_options.start_address - 1;

  for (uint16_t i = 0;
       i < m_options.gpio_pins.size() && (i + first_slot < dmx.Size());
       i++) {
    uint8_t slot_value = dmx.Get(i + first_slot);

    gpioPWM(m_options.gpio_pins[i], slot_value)

    switch ( [i].state) {
      case ON:
        action = (slot_value <= m_options.turn_off ? TURN_OFF : NO_CHANGE);
        break;
      case OFF:
        action = (slot_value >= m_options.turn_on ? TURN_ON : NO_CHANGE);
        break;
      case UNDEFINED:
      default:
        // If the state if undefined and the value is in the mid-range, then
        // default to turning off.
        action = (slot_value >= m_options.turn_on ? TURN_ON : TURN_OFF);
    }

    // Change the pin state if required.
    if (action != NO_CHANGE) {
      char data = (action == TURN_ON ? '1' : '0');
      if (write(m_gpio_pins[i].fd, &data, sizeof(data)) < 0) {
        OLA_WARN << "Failed to toggle GPIO pin " << i << ", fd "
                 << static_cast<int>(m_gpio_pins[i].fd) << ": "
                 << strerror(errno);
        return false;
      }
      m_gpio_pins[i].state = (action == TURN_ON ? ON : OFF);
    }
  }
  return true;
}

void GPIODriver::CloseGPIOFDs() {
  GPIOPins::iterator iter = m_gpio_pins.begin();
  for (; iter != m_gpio_pins.end(); ++iter) {
    close(iter->fd);
  }
  m_gpio_pins.clear();
  // All done with pi gpio
  gpioTerminate();

}
}  // namespace gpio
}  // namespace plugin
}  // namespace ola
