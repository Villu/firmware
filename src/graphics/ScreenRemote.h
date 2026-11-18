#pragma once

#include "configuration.h"

#if HAS_TFT && HAS_WIFI && defined(ARCH_ESP32) && defined(SCREEN_REMOTE_PORT) && SCREEN_REMOTE_PORT
#define HAS_SCREEN_REMOTE 1

/**
 * A development aid: the screen over Wi-Fi. TCP port SCREEN_REMOTE_PORT answers one line per connection:
 *   shot                      "SHOT", width and height (16 bit, little endian), then the pixels as the panel got them
 *   tap X Y [MS]              a press at X,Y for MS milliseconds (120)
 *   drag X0 Y0 X1 Y1 [MS]     press at the first point, move to the second over MS milliseconds (400), release
 *   wake                      count as activity, which keeps the screen on or wakes it
 * No login: anyone on the network sees and works the screen. Call once, after the UI is set up and before its task
 * starts.
 */
void screenRemoteInit();
#endif
