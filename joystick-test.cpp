// Joystick tester: shows the state of controller 1's four directions and
// fire button in text mode, with a press count for each.
//
// Uses only the portable Rosa calls (text mode, joystick state, timing,
// events), so it could move to rosa/apps unchanged.

#include <cstdio>
#include <cstring>
#include "rocinante.h"
#include "text-mode.h"
#include "events.h"
#include "rosa/apps/launcher/launcher.h"

extern "C" {
int joystick_test_main(int argc, const char **argv);
};

static int register_app_initializer = []() -> int {
    LauncherRegisterApp("Joystick Test", "joystick-test", "", "", "", {}, {}, joystick_test_main);
    return 1;
}();

namespace {

constexpr int ButtonWidth = 7;          // cells, including the brackets
constexpr int ButtonCount = 5;
constexpr uint32_t ExitHoldMillis = 3000;

struct Button
{
    const char *label;      // at most ButtonWidth - 2 characters
    uint8_t bit;
    int column;
    int row;
    char seenLetter;
};

// Laid out as a d-pad on the left and fire on the right; needs 35 x 10 cells.
const Button buttons[ButtonCount] = {
    { "N",    CONTROLLER_NORTH_BIT,  9, 2, 'N' },
    { "S",    CONTROLLER_SOUTH_BIT,  9, 8, 'S' },
    { "W",    CONTROLLER_WEST_BIT,   1, 5, 'W' },
    { "E",    CONTROLLER_EAST_BIT,  17, 5, 'E' },
    { "FIRE", CONTROLLER_FIRE_BIT,  27, 5, 'F' },
};

// Write text centred in a field of ButtonWidth cells, with optional brackets.
void DrawCentered(int row, int column, uint8_t attributes, const char *text, bool brackets)
{
    char cells[ButtonWidth + 1];
    memset(cells, ' ', ButtonWidth);
    cells[ButtonWidth] = '\0';
    if(brackets) {
        cells[0] = '[';
        cells[ButtonWidth - 1] = ']';
    }
    int length = static_cast<int>(strlen(text));
    if(length > ButtonWidth - 2) {
        length = ButtonWidth - 2;
    }
    memcpy(cells + (ButtonWidth - length) / 2, text, length);
    RoTextModeSetLine(row, column, attributes, cells);
}

void DrawButton(const Button& button, bool pressed, unsigned int count)
{
    uint8_t attributes = pressed ? TEXT_INVERSE : TEXT_NO_ATTRIBUTES;
    DrawCentered(button.row, button.column, attributes, button.label, !pressed);

    char countText[ButtonWidth];
    snprintf(countText, sizeof(countText), "%u", count % 100000);
    DrawCentered(button.row + 1, button.column, attributes, countText, false);
}

void DrawStatus(int row, int width, uint8_t state, uint8_t seenMask)
{
    char line[64];
    bool allSeen = true;
    char seen[ButtonCount * 2 + 1];
    for(int i = 0; i < ButtonCount; i++) {
        bool wasSeen = seenMask & buttons[i].bit;
        allSeen = allSeen && wasSeen;
        seen[i * 2] = wasSeen ? buttons[i].seenLetter : '.';
        seen[i * 2 + 1] = ' ';
    }
    seen[ButtonCount * 2] = '\0';

    RoTextModeClearArea(0, width, row, 1, TEXT_NO_ATTRIBUTES);
    if(allSeen) {
        snprintf(line, sizeof(line), " raw 0x%02X  all five buttons seen", state);
    } else {
        snprintf(line, sizeof(line), " raw 0x%02X  seen %s", state, seen);
    }
    RoTextModeSetLine(row, 0, TEXT_NO_ATTRIBUTES, line);
}

};

int joystick_test_main([[maybe_unused]] int argc, [[maybe_unused]] const char **argv)
{
    RoTextMode();
    RoTextModeClearDisplay();

    int width, height;
    RoTextModeGetSize(&width, &height);

    const char *title = "Joystick Test";
    RoTextModeSetLine(0, (width - static_cast<int>(strlen(title))) / 2, TEXT_NO_ATTRIBUTES, title);
    int statusRow = 11;
    int hintRow = 12;
    if(hintRow < height) {
        RoTextModeSetLine(hintRow, 1, TEXT_NO_ATTRIBUTES, "hold FIRE 3 seconds to exit");
    }

    unsigned int counts[ButtonCount] = {0, 0, 0, 0, 0};
    uint8_t seenMask = 0;           // buttons that have been pressed and released
    uint8_t previous = 0xFF;        // force the first draw
    uint32_t fireDownSince = 0;
    bool first = true;

    while(true) {
        uint8_t state = RoGetJoystickState(CONTROLLER_1);

        if(first || (state != previous)) {
            uint8_t before = first ? 0 : previous;
            for(int i = 0; i < ButtonCount; i++) {
                bool pressed = state & buttons[i].bit;
                bool wasPressed = before & buttons[i].bit;
                if(pressed && !wasPressed) {
                    counts[i]++;
                }
                if(!pressed && wasPressed) {
                    seenMask |= buttons[i].bit;
                }
                DrawButton(buttons[i], pressed, counts[i]);
            }
            if(statusRow < height) {
                DrawStatus(statusRow, width, state, seenMask);
            }

            if((state & CONTROLLER_FIRE_BIT) && !(before & CONTROLLER_FIRE_BIT)) {
                fireDownSince = RoGetMillis();
            }
            previous = state;
            first = false;
        }

        if((state & CONTROLLER_FIRE_BIT) && ((RoGetMillis() - fireDownSince) >= ExitHoldMillis)) {
            break;
        }

        RoEvent ev;
        if(RoEventPoll(&ev) && (ev.eventType == RoEvent::KEYBOARD_RAW) &&
            ev.u.keyboardRaw.isPress && (ev.u.keyboardRaw.key == KEYCAP_ESCAPE)) {
            break;
        }

        RoDoHousekeeping();
        RoDelayMillis(1);   // sample about 1000 times a second, fast enough to count contact bounce
    }

    // Wait for fire to be released so the launcher doesn't see it as a selection.
    while(RoGetJoystickState(CONTROLLER_1) & CONTROLLER_FIRE_BIT) {
        RoDoHousekeeping();
        RoDelayMillis(1);
    }

    return 0;
}
