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
 * GPIOPlugin.cpp
 * The General Purpose digital I/O plugin.
 * Copyright (C) 2014 Simon Newton
 */

#include "plugins/gpio/GPIOPlugin.h"

#include <memory>
#include <string>
#include <vector>
#include "olad/Preferences.h"
#include "ola/Logging.h"
#include "ola/StringUtils.h"
#include "plugins/gpio/GPIODevice.h"
#include "plugins/gpio/GPIODriver.h"

namespace ola {
namespace plugin {
namespace gpio {

using std::auto_ptr;
using std::string;
using std::vector;

// The values of these keys should match the argument names in the README file
const char GPIOPlugin::GPIO_PINS_KEY[] = "gpio_pins";
const char GPIOPlugin::GPIO_SLOT_OFFSET_KEY[] = "gpio_slot_offset";
const char GPIOPlugin::GPIO_PWM_FREQUENCY_KEY[] = "gpio_pwm_frequency";

const char GPIOPlugin::PLUGIN_NAME[] = "GPIO";
const char GPIOPlugin::PLUGIN_PREFIX[] = "gpio";

bool GPIOPlugin::StartHook() {
  // Let's populate the driver options gpio_pins, start_address, and pwm_frequency
  GPIODriver::Options options;

  // Process the gpio_pins argument
  vector<string> pin_list;
  StringSplit(m_preferences->GetValue(GPIO_PINS_KEY), &pin_list, ",");
  vector<string>::const_iterator iter = pin_list.begin();
  for (; iter != pin_list.end(); ++iter) {
    if (iter->empty()) {
      continue;
    }

    uint16_t pin;
    if (!StringToInt(*iter, &pin)) {
      OLA_WARN << "Invalid value for GPIO pin: " << *iter;
      return false;
    }
    options.gpio_pins.push_back(pin);
  }

  // stop now if no pins are defined; do not initialize or register device
  if (options.gpio_pins.empty()) {
    return true;
  }

  // Process the gpio_slot_offset argument
  char offset[] = m_preferences->GetValue(GPIO_SLOT_OFFSET_KEY);
  if (!StringToInt(offset, &options.start_address)) {
    OLA_WARN << "Invalid value for "<< GPIO_SLOT_OFFSET_KEY << ": " << offset;
    return false;
  }

  // Process the gpio_pwm_frequency argument
  char freq[] = m_preferences->GetValue(GPIO_PWM_FREQUENCY_KEY);
  if (!StringToInt(freq, &options.pwm_frequency)) {
    OLA_WARN << "Invalid value for "<< GPIO_PWM_FREQUENCY_KEY << ": " << freq;
    return false;
  }


  // Initialize the GPIO device and pass it options
  std::auto_ptr<GPIODevice> device(new GPIODevice(this, options));
  if (!device->Start()) {
    return false;
  }

  m_plugin_adaptor->RegisterDevice(device.get());
  m_device = device.release();
  return true;
}

bool GPIOPlugin::StopHook() {
  if (m_device) {
    m_plugin_adaptor->UnregisterDevice(m_device);
    m_device->Stop();
    delete m_device;
    m_device = NULL;
  }
  return true;
}

string GPIOPlugin::Description() const {
  return plugin_description;
}

bool GPIOPlugin::SetDefaultPreferences() {
  bool save = false;

  if (!m_preferences)
    return false;

  save |= m_preferences->SetDefaultValue(GPIO_PINS_KEY,
                                         StringValidator(),
                                         "");
  save |= m_preferences->SetDefaultValue(GPIO_SLOT_OFFSET_KEY,
                                         UIntValidator(1, DMX_UNIVERSE_SIZE),
                                         "1");
  save |= m_preferences->SetDefaultValue(GPIO_PWM_FREQUENCY_KEY,
                                         UIntValidator(PWM_MIN_FREQUENCY,
                                                       PWM_MAX_FREQUENCY),
                                         "1000");

  if (save) {
    m_preferences->Save();
  }

  if (m_preferences->GetValue(GPIO_SLOT_OFFSET_KEY).empty()) {
    return false;
  }

  if (m_preferences->GetValue(GPIO_PWM_FREQUENCY_KEY).empty()) {
    return false;
  }

  return true;
}
}  // namespace gpio
}  // namespace plugin
}  // namespace ola
