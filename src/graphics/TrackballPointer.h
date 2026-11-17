#pragma once

#include "configuration.h"

#if HAS_TFT && defined(TRACKBALL_POINTER) && defined(INPUTDRIVER_ENCODER_LEFT) && defined(INPUTDRIVER_ENCODER_RIGHT) &&          \
    defined(INPUTDRIVER_ENCODER_UP) && defined(INPUTDRIVER_ENCODER_DOWN) && defined(INPUTDRIVER_ENCODER_BTN)
#define HAS_TRACKBALL_POINTER 1

/**
 * The trackball as a mouse: rolling it moves an arrow over the screen, pressing it is a tap where the arrow points
 * (press, hold and roll is a drag). Replaces the UI's own use of the ball, which steps the focus from one widget to
 * the next. Call once, after the UI is set up and before its task starts.
 */
void trackballPointerInit();
#endif
