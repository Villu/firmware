#include "ScreenRemote.h"

#if HAS_SCREEN_REMOTE

#include "lvgl.h"
#include "lvgl_private.h" // the display's flush callback, to put ours in front of it
#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <lwip/sockets.h>

namespace
{
// A copy of every area the UI sends to the panel: what is on the glass, without reading the panel back
uint8_t *mirror = nullptr;
int32_t mirrorW = 0, mirrorH = 0;
lv_display_flush_cb_t panelFlush = nullptr;

void mirrorFlush(lv_display_t *disp, const lv_area_t *area, uint8_t *px)
{
    if (area->x1 >= 0 && area->y1 >= 0 && area->x2 < mirrorW && area->y2 < mirrorH) {
        const int32_t w = lv_area_get_width(area);
        for (int32_t y = area->y1; y <= area->y2; y++)
            memcpy(mirror + ((size_t)y * mirrorW + area->x1) * 2, px + (size_t)(y - area->y1) * w * 2, (size_t)w * 2);
    }
    panelFlush(disp, area, px);
}

// The finger the server task moves; LVGL's task reads it
volatile int16_t touchX = 0, touchY = 0;
volatile bool touchDown = false;
volatile uint32_t wakeCount = 0;
volatile uint32_t readCount = 0; // how often LVGL has looked at the finger

void readRemote(lv_indev_t *indev, lv_indev_data_t *data)
{
    static uint32_t wakeSeen = 0;
    readCount++;
    const bool down = touchDown;
    if (down || wakeSeen != wakeCount) {
        wakeSeen = wakeCount;
        lv_display_trigger_activity(lv_display_get_default());
    }
    data->point.x = touchX;
    data->point.y = touchY;
    data->state = down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

// Wait until LVGL has looked at the finger this many more times. A press is timed from when LVGL saw it, not from
// when it was set here: while the UI is busy (map tiles) it looks late, and a 700 ms press ended up a short click.
void awaitReads(uint32_t count, uint32_t limitMs)
{
    const uint32_t from = readCount, start = millis();
    while (readCount - from < count && millis() - start < limitMs)
        delay(5);
}

// A sleeping screen is one black cover over everything (and nothing else on the screen is ever all black)
bool looksAsleep()
{
    if (!mirror)
        return false;
    const size_t bytes = (size_t)mirrorW * mirrorH * 2;
    for (size_t i = 0; i < bytes; i += 2 * 53)
        if (mirror[i] || mirror[i + 1])
            return false;
    return true;
}

bool sendAll(int client, const void *data, size_t len)
{
    const uint8_t *at = (const uint8_t *)data;
    while (len) {
        const int sent = send(client, at, len > 2048 ? 2048 : len, 0);
        if (sent <= 0)
            return false;
        at += sent;
        len -= sent;
    }
    return true;
}

void serve(int client)
{
    char line[64];
    size_t have = 0;
    while (have < sizeof(line) - 1) {
        const int got = recv(client, line + have, 1, 0);
        if (got <= 0 || line[have] == '\n')
            break;
        have++;
    }
    line[have] = 0;

    int a = 0, b = 0, c = 0, d = 0, ms = 0;
    if (strncmp(line, "shot", 4) == 0 && mirror) {
        const uint8_t head[8] = {'S', 'H', 'O', 'T', (uint8_t)mirrorW, (uint8_t)(mirrorW >> 8), (uint8_t)mirrorH,
                                 (uint8_t)(mirrorH >> 8)};
        if (sendAll(client, head, sizeof(head)))
            sendAll(client, mirror, (size_t)mirrorW * mirrorH * 2);
    } else if (sscanf(line, "tap %d %d %d", &a, &b, &ms) >= 2) {
        touchX = a;
        touchY = b;
        wakeCount++;
        awaitReads(2, 500); // the place seen, finger up
        touchDown = true;
        awaitReads(2, 3000); // the press seen: the hold counts from here
        delay(ms > 0 ? ms : 120);
        awaitReads(2, 3000); // looked at once more while down, so a long press that is due is sent
        touchDown = false;
        awaitReads(2, 500);
        sendAll(client, "ok\n", 3);
    } else if (sscanf(line, "drag %d %d %d %d %d", &a, &b, &c, &d, &ms) >= 4) {
        const int steps = (ms > 0 ? ms : 400) / 20 + 1;
        touchX = a;
        touchY = b;
        wakeCount++;
        delay(80);
        touchDown = true;
        delay(80);
        for (int i = 1; i <= steps; i++) {
            touchX = a + (c - a) * i / steps;
            touchY = b + (d - b) * i / steps;
            delay(20);
        }
        delay(150); // rest before letting go, so the list does not coast on
        touchDown = false;
        delay(80);
        sendAll(client, "ok\n", 3);
    } else if (strncmp(line, "wake", 4) == 0) {
        // Awake: counting as activity keeps it on. Asleep: only a press on the black cover wakes it, and there a
        // press does nothing else. So this never presses anything on a screen that is on.
        wakeCount++;
        if (looksAsleep()) {
            touchX = mirrorW / 2;
            touchY = mirrorH / 2;
            awaitReads(2, 500);
            touchDown = true;
            awaitReads(3, 3000);
            delay(120);
            touchDown = false;
            awaitReads(2, 500);
            delay(700); // the screen comes back and redraws
        }
        sendAll(client, "ok\n", 3);
    } else {
        sendAll(client, "?\n", 2);
    }
}

void serverTask(void *)
{
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) { // also: no sockets before the network stack is up
            delay(3000);
            continue;
        }
        const int listener = socket(AF_INET, SOCK_STREAM, 0);
        if (listener < 0) {
            delay(3000);
            continue;
        }
        int yes = 1;
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(SCREEN_REMOTE_PORT);
        if (bind(listener, (sockaddr *)&addr, sizeof(addr)) == 0 && listen(listener, 1) == 0) {
            while (WiFi.status() == WL_CONNECTED) {
                fd_set readable;
                FD_ZERO(&readable);
                FD_SET(listener, &readable);
                timeval wait = {2, 0};
                if (select(listener + 1, &readable, nullptr, nullptr, &wait) <= 0)
                    continue;
                const int client = accept(listener, nullptr, nullptr);
                if (client < 0)
                    break;
                timeval limit = {5, 0};
                setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &limit, sizeof(limit));
                setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &limit, sizeof(limit));
                serve(client);
                close(client);
            }
        }
        close(listener);
        delay(1000);
    }
}
} // namespace

void screenRemoteInit()
{
    lv_display_t *disp = lv_display_get_default();
    if (!disp)
        return;
    mirrorW = lv_display_get_horizontal_resolution(disp);
    mirrorH = lv_display_get_vertical_resolution(disp);
    mirror = (uint8_t *)heap_caps_calloc((size_t)mirrorW * mirrorH, 2, MALLOC_CAP_SPIRAM);
    if (mirror && disp->flush_cb) {
        panelFlush = disp->flush_cb;
        lv_display_set_flush_cb(disp, mirrorFlush);
    } else if (mirror) {
        heap_caps_free(mirror);
        mirror = nullptr;
    }

    lv_indev_t *finger = lv_indev_create();
    lv_indev_set_type(finger, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(finger, readRemote);

    xTaskCreate(serverTask, "screenRemote", 4096, nullptr, 1, nullptr);
    LOG_INFO("Screen remote: TCP port %d, %s", SCREEN_REMOTE_PORT, mirror ? "with screenshots" : "no memory for screenshots");
}

#endif
