> [!NOTE]
> This is a personal fork of the Meshtastic firmware. Its branch `meshcore-coexist` adds MeshCore beside Meshtastic on the LilyGo T-Deck Plus; [NOTICE.md](NOTICE.md) says what it is and where its code comes from. It is experimental, it is not an official Meshtastic build, and it is not affiliated with or endorsed by the Meshtastic project. Meshtastic® is a registered trademark of Meshtastic LLC.

### This fork: which radio settings MeshCore and Meshtastic can share

The radio stays on Meshtastic. About every 80 ms it leaves for one channel-activity scan on MeshCore's settings (10–16 ms) and stays there only when the scan finds a frame. That works when all of these hold:

- **MeshCore at SF7 or SF8.** MeshCore sends a 32-symbol preamble only at SF7/SF8; from SF9 up it uses 16 symbols, too short to be caught by scans that come only now and then. Meshtastic's preamble is always 16 symbols, so the roles cannot be swapped either.
- **A Meshtastic preset other than ShortTurbo, ShortFast or ShortSlow.** A scan has to be well shorter than Meshtastic's preamble (4–16 ms on the Short presets).
- **MeshCore's channel inside the Meshtastic region's band** (its centre frequency plus and minus half its bandwidth).

| MeshCore settings | NarrowFast, NarrowSlow, MediumFast, MediumSlow, LongFast, LongModerate, LongSlow | ShortTurbo, ShortFast, ShortSlow |
| --- | --- | --- |
| SF8 / 62.5 kHz (EU/UK Narrow, 869.618 MHz) | yes | no |
| SF7 / 62.5 kHz (US/Canada 910.525 MHz, CZ, AU/NZ narrow) | yes | no |
| SF9 and up (any bandwidth) | no | no |

Settings that do not fit are refused when the MeshCore app sets them. If the Meshtastic preset is changed to one that does not fit, MeshCore pauses and Meshtastic runs alone until it fits again. Measured on the air only with Meshtastic NarrowFast (region `EU_N_868`) and MeshCore EU/UK Narrow; the other "yes" cells follow from the same timing rule (`sideChannelFits()` in `src/mesh/SX126xSideChannel.inl`) and are untested.

<div align="center" markdown="1">

<img src=".github/meshtastic_logo.png" alt="Meshtastic Logo" width="80"/>
<h1>Meshtastic Firmware</h1>

![GitHub release downloads](https://img.shields.io/github/downloads/meshtastic/firmware/total)
[![CI](https://img.shields.io/github/actions/workflow/status/meshtastic/firmware/main_matrix.yml?branch=master&label=actions&logo=github&color=yellow)](https://github.com/meshtastic/firmware/actions/workflows/ci.yml)
[![CLA assistant](https://cla-assistant.io/readme/badge/meshtastic/firmware)](https://cla-assistant.io/meshtastic/firmware)
[![Fiscal Contributors](https://opencollective.com/meshtastic/tiers/badge.svg?label=Fiscal%20Contributors&color=deeppink)](https://opencollective.com/meshtastic/)
[![Vercel](https://img.shields.io/static/v1?label=Powered%20by&message=Vercel&style=flat&logo=vercel&color=000000)](https://vercel.com?utm_source=meshtastic&utm_campaign=oss)
[![MemBrowse](https://membrowse.com/badge.svg)](https://membrowse.com/public/meshtastic/firmware)

<a href="https://trendshift.io/repositories/5524" target="_blank"><img src="https://trendshift.io/api/badge/repositories/5524" alt="meshtastic%2Ffirmware | Trendshift" style="width: 250px; height: 55px;" width="250" height="55"/></a>

</div>

</div>

<div align="center">
	<a href="https://meshtastic.org">Website</a>
	-
	<a href="https://meshtastic.org/docs/">Documentation</a>
</div>

## Overview

This repository contains the official device firmware for Meshtastic, an open-source LoRa mesh networking project designed for long-range, low-power communication without relying on internet or cellular infrastructure. The firmware supports various hardware platforms, including ESP32, nRF52, RP2040/RP2350, and Linux-based devices.

Meshtastic enables text messaging, location sharing, and telemetry over a decentralized mesh network, making it ideal for outdoor adventures, emergency preparedness, and remote operations.

### Get Started

- 🔧 **[Building Instructions](https://meshtastic.org/docs/development/firmware/build)** - Learn how to compile the firmware from source.
- ⚡ **[Flashing Instructions](https://meshtastic.org/docs/getting-started/flashing-firmware/)** - Install or update the firmware on your device.

Join our community and help improve Meshtastic! 🚀

## Stats

![Alt](https://repobeats.axiom.co/api/embed/8025e56c482ec63541593cc5bd322c19d5c0bdcf.svg "Repobeats analytics image")
