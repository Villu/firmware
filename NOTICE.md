# Notice: what this fork is and where its code comes from

This repository is a personal fork of the [Meshtastic firmware](https://github.com/meshtastic/firmware). Its branch `meshcore-coexist` adds the [MeshCore](https://github.com/meshcore-dev/MeshCore) companion firmware inside Meshtastic, so that a LilyGo T-Deck Plus serves both mesh networks from its one SX1262 radio. Everything it adds is compiled only into the PlatformIO environment `t-deck-tft-meshcore` (`-D MESHCORE_COEXIST=1`).

It is experimental, tested on one device, and not an official build of either project.

Meshtastic® is a registered trademark of Meshtastic LLC. This fork is not affiliated with or endorsed by the Meshtastic project. It is not affiliated with or endorsed by the MeshCore project either.

## Licence

The combined firmware is distributed under the GNU General Public License v3.0, the licence of the Meshtastic firmware (`LICENSE`). The MIT- and zlib-licensed parts listed below keep their own licences and copyright notices.

## Sources

| Source | Licence and copyright | What comes from it |
| --- | --- | --- |
| [meshtastic/firmware](https://github.com/meshtastic/firmware), tag `v2.8.2.d5f1619` | GPL-3.0, the Meshtastic contributors | The whole firmware this fork is based on. Some of its files are modified; `git diff v2.8.2.d5f1619 meshcore-coexist` shows every change. |
| [meshcore-dev/MeshCore](https://github.com/meshcore-dev/MeshCore), tag `companion-v1.18.0` (commit `81dbae5`) | MIT, © 2025 Scott Powell / rippleradios.com (`lib/MeshCore/LICENSE-MeshCore.txt`) | `lib/MeshCore/src/`: the mesh core (`*.cpp`, `*.h`), `helpers/`, and `companion/` (the companion firmware's `MyMesh`, `DataStore` and `NodePrefs`, from `examples/companion_radio`). Unchanged except where marked `MESHCORE_HOSTED`. The packet-score formula in `host/HostTarget.cpp` follows `RadioLibWrapper::packetScoreInt()`. |
| [orlp/ed25519](https://github.com/orlp/ed25519), as shipped with MeshCore | zlib, © 2015 Orson Peters (`lib/MeshCore/src/ed25519/license.txt`) | `lib/MeshCore/src/ed25519/`, `ed_25519.h` |
| [ElectronicCats/CayenneLPP](https://github.com/ElectronicCats/CayenneLPP) 1.6.1 | MIT | A MeshCore dependency, fetched by PlatformIO at build time (not in this repository). |
| [Densaugeo/base64_arduino](https://github.com/Densaugeo/base64_arduino) 1.4.0 | MIT | A MeshCore dependency, fetched by PlatformIO at build time (not in this repository). |
| [jgromes/RadioLib](https://github.com/jgromes/RadioLib) | MIT | Already a Meshtastic dependency. The radio sharing (`src/mesh/SX126xSideChannel.inl`) uses its SX126x command interface and repeats some of its register choices. |
| Semtech application notes AN1200.48 (channel activity detection) and AN1200.13 (time on air) | Documentation | The CAD settings and the airtime formula in `src/mesh/SX126xSideChannel.inl`. |
| Nordic UART Service | UUIDs defined by Nordic Semiconductor | The Bluetooth service the MeshCore phone app connects to (`src/nimble/NimbleBluetooth.cpp`), as in MeshCore's own firmware. |
| [skizzophrenic/SquachWatch-CYD](https://github.com/skizzophrenic/SquachWatch-CYD) | — | Prior art: the firmware in which the author first saw Meshtastic and MeshCore sharing one radio on this device. No code from it is used here. |

## Written for this fork

`src/mesh/SideChannel.h`, `src/mesh/SX126xSideChannel.inl`, `src/modules/MeshCoreModule.*`, `lib/MeshCore/src/MeshCoreHost.h`, `lib/MeshCore/src/target.h`, `lib/MeshCore/src/host/`, and the changes to the Meshtastic files are by Villu Sepman, written with Claude Code (Anthropic's AI coding assistant; the commits carry a `Co-Authored-By` line). They are offered under GPL-3.0, like the firmware they extend.
