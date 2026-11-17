#pragma once

#include "configuration.h"

#if MESHCORE_COEXIST

#include "concurrency/OSThread.h"
#include "mesh/MeshTypes.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

/**
 * MeshCore beside Meshtastic on the one radio.
 *
 * Runs the MeshCore companion node (lib/MeshCore) on the radio driver's side channel and shows its traffic through
 * Meshtastic's own interfaces: group messages of MeshCore's Public channel appear in a Meshtastic channel named
 * "MeshCore", senders and advertised nodes as nodes, direct messages as direct messages. Text sent to that channel or
 * to such a node goes out over MeshCore instead of Meshtastic. The MeshCore phone app can connect as well.
 */
class MeshCoreModule : private concurrency::OSThread
{
  public:
    MeshCoreModule();

    /// A packet the phone app or the device UI wants sent. True if it was MeshCore's and has been dealt with.
    bool handleFromPhone(meshtastic_MeshPacket &p);

    /// Whether this node number stands for a MeshCore node
    static bool isMeshCoreNode(NodeNum n);

  protected:
    int32_t runOnce() override;

  private:
    bool started = false;
    bool failed = false;
    uint32_t nextAdvertMs = 0;
    uint32_t nextStatsMs = 0;
    void start();
};

extern MeshCoreModule *meshCoreModule;

#endif
