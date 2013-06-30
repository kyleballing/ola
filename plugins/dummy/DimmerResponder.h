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
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * DimmerResponder_h
 * Simulates a RDM enabled dimmer with sub devices.
 * Copyright (C) 2013 Simon Newton
 */

#ifndef PLUGINS_DUMMY_DIMMERRESPONDER_H_
#define PLUGINS_DUMMY_DIMMERRESPONDER_H_

#include <map>
#include <memory>
#include "ola/rdm/RDMControllerInterface.h"
#include "ola/rdm/RDMEnums.h"
#include "ola/rdm/UID.h"
#include "plugins/dummy/DimmerRootDevice.h"
#include "plugins/dummy/DimmerSubDevice.h"
#include "plugins/dummy/SubDeviceDispatcher.h"

namespace ola {
namespace plugin {
namespace dummy {

using std::auto_ptr;

class DimmerResponder: public ola::rdm::RDMControllerInterface {
  public:
    DimmerResponder(const ola::rdm::UID &uid,
                    uint16_t number_of_subdevices = 0);
    virtual ~DimmerResponder();

    void SendRDMRequest(const ola::rdm::RDMRequest *request,
                        ola::rdm::RDMCallback *callback);

    const ola::rdm::UID &UID() const { return m_uid; }

  private:
    ola::rdm::UID m_uid;
    SubDeviceDispatcher m_dispatcher;
    auto_ptr<DimmerRootDevice> m_root_device;
    std::map<uint16_t, DimmerSubDevice*> m_sub_devices;
};
}  // namespace dummy
}  // namespace plugin
}  // namespace ola
#endif  // PLUGINS_DUMMY_DIMMERRESPONDER_H_