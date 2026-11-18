#pragma once
// An alert (the message sound and a line on the screen) for a node never seen before that is either heard directly or
// is a router. Part of the MeshCore fork's screen work; nothing of it is compiled without MESHCORE_COEXIST.

#include "MeshTypes.h"

#if MESHCORE_COEXIST
namespace NewNodeAlert
{
/// The node database has just made an entry for a node it never knew (not one coming back from its warm tier)
void created(NodeNum n);

/// A packet from a node; only the first one after created() counts. hops: -1 when the packet cannot say.
void heard(NodeNum n, int8_t hops, bool viaMqtt, float snr);

/// The MeshCore side heard this node (an advert or a message) this many hops away
void heardMeshCore(NodeNum n, uint8_t hops);
} // namespace NewNodeAlert
#endif
