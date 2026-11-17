#include "TrackballPointer.h"

#if HAS_TRACKBALL_POINTER

#include "lvgl.h"
#include <Arduino.h>

namespace
{
// Each of the ball's four hall sensors toggles its line as the ball turns: every edge is one step that way. A step is
// worth more pixels the sooner it follows the one before, so a slow roll places the arrow and a flick crosses the screen.
const uint8_t STEP_SLOW = 2, STEP_WALK = 4, STEP_RUN = 7, STEP_FLICK = 11;
const uint32_t HIDE_AFTER_MS = 8000;  // the arrow goes away when the ball rests
const uint32_t ASLEEP_AFTER_MS = 20000; // a press with nothing before it for this long only wakes the screen

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t pendingX = 0, pendingY = 0;
volatile uint32_t lastEdgeUs[2] = {0, 0};

lv_obj_t *arrow = nullptr;

inline int32_t IRAM_ATTR stepPixels(uint8_t axis)
{
    const uint32_t now = micros();
    const uint32_t since = now - lastEdgeUs[axis];
    lastEdgeUs[axis] = now;
    return since < 25000 ? STEP_FLICK : since < 50000 ? STEP_RUN : since < 110000 ? STEP_WALK : STEP_SLOW;
}

void IRAM_ATTR onLeft()
{
    portENTER_CRITICAL_ISR(&mux);
    pendingX -= stepPixels(0);
    portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR onRight()
{
    portENTER_CRITICAL_ISR(&mux);
    pendingX += stepPixels(0);
    portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR onUp()
{
    portENTER_CRITICAL_ISR(&mux);
    pendingY -= stepPixels(1);
    portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR onDown()
{
    portENTER_CRITICAL_ISR(&mux);
    pendingY += stepPixels(1);
    portEXIT_CRITICAL_ISR(&mux);
}

void readPointer(lv_indev_t *indev, lv_indev_data_t *data)
{
    static int32_t x = -1, y = -1;
    static bool visible = false, onlyWaking = false;
    static uint32_t lastUsedMs = 0;

    lv_display_t *disp = lv_indev_get_display(indev);
    if (!disp)
        disp = lv_display_get_default();
    const int32_t width = lv_display_get_horizontal_resolution(disp);
    const int32_t height = lv_display_get_vertical_resolution(disp);
    if (x < 0) {
        x = width / 2;
        y = height / 2;
    }

    portENTER_CRITICAL(&mux);
    const int32_t dx = pendingX, dy = pendingY;
    pendingX = pendingY = 0;
    portEXIT_CRITICAL(&mux);
    const bool pressed = digitalRead(INPUTDRIVER_ENCODER_BTN) == LOW;

    if (dx || dy || pressed) {
        // LVGL counts only presses as activity: a roll has to keep the screen on, and wake it, as well
        if (pressed && !dx && !dy && lv_display_get_inactive_time(disp) > ASLEEP_AFTER_MS)
            onlyWaking = true;
        lv_display_trigger_activity(disp);
        lastUsedMs = millis();
        if (!visible && arrow) {
            lv_obj_remove_flag(arrow, LV_OBJ_FLAG_HIDDEN);
            visible = true;
        }
    } else if (visible && millis() - lastUsedMs > HIDE_AFTER_MS) {
        lv_obj_add_flag(arrow, LV_OBJ_FLAG_HIDDEN);
        visible = false;
    }
    if (!pressed)
        onlyWaking = false;

    x = LV_CLAMP(0, x + dx, width - 1);
    y = LV_CLAMP(0, y + dy, height - 1);
    data->point.x = x;
    data->point.y = y;
    data->state = pressed && !onlyWaking ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

// X outline, . fill; the tip, top left, is where the arrow points
const char *const ARROW_ART[] = {"X           ", "XX          ", "X.X         ", "X..X        ", "X...X       ", "X....X      ",
                                 "X.....X     ", "X......X    ", "X.......X   ", "X........X  ", "X.....XXXXX ", "X..X..X     ",
                                 "X.X X..X    ", "XX  X..X    ", "X    X..X   ", "     X..X   ", "      XX    "};
const int ARROW_W = 12, ARROW_H = sizeof(ARROW_ART) / sizeof(ARROW_ART[0]);
uint32_t arrowPixels[ARROW_W * ARROW_H];
lv_image_dsc_t arrowImage;

lv_obj_t *makeArrow()
{
    for (int row = 0; row < ARROW_H; row++)
        for (int col = 0; col < ARROW_W; col++) {
            const char c = ARROW_ART[row][col];
            arrowPixels[row * ARROW_W + col] = c == 'X' ? 0xFF000000u : c == '.' ? 0xFFFFFFFFu : 0x00000000u;
        }
    arrowImage.header.magic = LV_IMAGE_HEADER_MAGIC;
    arrowImage.header.cf = LV_COLOR_FORMAT_ARGB8888;
    arrowImage.header.w = ARROW_W;
    arrowImage.header.h = ARROW_H;
    arrowImage.header.stride = ARROW_W * 4;
    arrowImage.data_size = sizeof(arrowPixels);
    arrowImage.data = (const uint8_t *)arrowPixels;
    lv_obj_t *img = lv_image_create(lv_layer_sys());
    lv_image_set_src(img, &arrowImage);
    lv_obj_add_flag(img, LV_OBJ_FLAG_HIDDEN);
    return img;
}
} // namespace

void trackballPointerInit()
{
    // The UI made an encoder device of the ball and reads its button too: silence it, or a press would also be an
    // ENTER for whichever widget has the focus. The keyboard keeps the focus group to itself.
    for (lv_indev_t *dev = lv_indev_get_next(nullptr); dev; dev = lv_indev_get_next(dev))
        if (lv_indev_get_type(dev) == LV_INDEV_TYPE_ENCODER)
            lv_indev_enable(dev, false);

    pinMode(INPUTDRIVER_ENCODER_LEFT, INPUT_PULLUP);
    pinMode(INPUTDRIVER_ENCODER_RIGHT, INPUT_PULLUP);
    pinMode(INPUTDRIVER_ENCODER_UP, INPUT_PULLUP);
    pinMode(INPUTDRIVER_ENCODER_DOWN, INPUT_PULLUP);
    pinMode(INPUTDRIVER_ENCODER_BTN, INPUT);
    attachInterrupt(INPUTDRIVER_ENCODER_LEFT, onLeft, CHANGE); // takes the pins over from the UI's handlers
    attachInterrupt(INPUTDRIVER_ENCODER_RIGHT, onRight, CHANGE);
    attachInterrupt(INPUTDRIVER_ENCODER_UP, onUp, CHANGE);
    attachInterrupt(INPUTDRIVER_ENCODER_DOWN, onDown, CHANGE);

    lv_indev_t *pointer = lv_indev_create();
    lv_indev_set_type(pointer, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(pointer, readPointer);
    arrow = makeArrow();
    lv_indev_set_cursor(pointer, arrow);
    LOG_INFO("Trackball: pointer mode");
}

#endif
