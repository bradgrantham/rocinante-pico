#include <cstdio>
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include "byte_queue.h"
#include "events.h"
#include "events_internal.h"
#include "rocinante.h"

#if 0
void LauncherRegisterApp(const std::string& name, const std::string& exe_name, const std::string& what_to_choose, const std::string& where_to_choose, const std::string& suffix, const std::vector<std::string>& first_args, const std::vector<std::string>& last_args, int (*main)(int argc, const char **argv))
{
}
#endif

void enqueue_press_release(int keycap)
{
    RoEvent ev;

    ev.eventType = RoEvent::KEYBOARD_RAW;
    ev.u.keyboardRaw.key = keycap;

    ev.u.keyboardRaw.isPress = true;
    SystemEventEnqueue(ev);

    ev.u.keyboardRaw.isPress = false;
    SystemEventEnqueue(ev);
}

void enqueue_press_release(int keycap1, int keycap2)
{
    RoEvent ev;

    ev.eventType = RoEvent::KEYBOARD_RAW;

    ev.u.keyboardRaw.key = keycap1;
    ev.u.keyboardRaw.isPress = true;
    SystemEventEnqueue(ev);

    ev.u.keyboardRaw.key = keycap2;
    ev.u.keyboardRaw.isPress = true;
    SystemEventEnqueue(ev);

    ev.u.keyboardRaw.isPress = false;
    ev.u.keyboardRaw.key = keycap2;
    SystemEventEnqueue(ev);

    ev.u.keyboardRaw.isPress = false;
    ev.u.keyboardRaw.key = keycap1;
    SystemEventEnqueue(ev);
}


extern "C" {

int enqueue_serial_input(uint8_t c)
{
    if(c >= 'a' && c <= 'z')
    {
        enqueue_press_release(KEYCAP_A + c - 'a');
        return 1;
    }
    else if(c >= '1' && c <= '9')
    {
        enqueue_press_release(KEYCAP_1_EXCLAMATION + c - '1');
        return 1;
    }
    else if(c >= 'A' && c <= 'Z')
    {
        enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_A + c - 'A');
        return 1;
    }
    else
    {
        switch(c)
        {
            case ' ' : enqueue_press_release(KEYCAP_SPACE); return 1; break;
            case '0' : enqueue_press_release(KEYCAP_0_CPAREN); return 1; break;
            case '!' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_1_EXCLAMATION); return 1; break;
            case '@' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_2_AT); return 1; break;
            case '#' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_3_NUMBER); return 1; break;
            case '$' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_4_DOLLAR); return 1; break;
            case '%' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_5_PERCENT); return 1; break;
            case '^' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_6_CARET); return 1; break;
            case '&' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_7_AMPERSAND); return 1; break;
            case '*' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_8_ASTERISK); return 1; break;
            case '(' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_9_OPAREN); return 1; break;
            case ')' : enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_0_CPAREN); return 1; break;
            case '\n' : enqueue_press_release(KEYCAP_ENTER); return 1; break;
            case '' : enqueue_press_release(KEYCAP_ESCAPE); return 1; break;
            case '' : enqueue_press_release(KEYCAP_BACKSPACE); return 1; break;
            case '\t': enqueue_press_release(KEYCAP_TAB); return 1; break;
            case '-': enqueue_press_release(KEYCAP_HYPHEN_UNDER); return 1; break;
            case '_': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_HYPHEN_UNDER); return 1; break;
            case '[': enqueue_press_release(KEYCAP_OBRACKET_OBRACE); return 1; break;
            case '{': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_OBRACKET_OBRACE); return 1; break;
            case ']': enqueue_press_release(KEYCAP_CBRACKET_CBRACE); return 1; break;
            case '}': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_CBRACKET_CBRACE); return 1; break;
            case '\\': enqueue_press_release(KEYCAP_BACKSLASH_PIPE); return 1; break;
            case '|': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_BACKSLASH_PIPE); return 1; break;
            case ';': enqueue_press_release(KEYCAP_SEMICOLON_COLON); return 1; break;
            case ':': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_SEMICOLON_COLON); return 1; break;
            case '\'': enqueue_press_release(KEYCAP_SINGLEQUOTE_DOUBLEQUOTE); return 1; break;
            case '"': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_SINGLEQUOTE_DOUBLEQUOTE); return 1; break;
            case '`': enqueue_press_release(KEYCAP_GRAVE_TILDE); return 1; break;
            case '~': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_GRAVE_TILDE); return 1; break;
            case ',': enqueue_press_release(KEYCAP_COMMA_LESS); return 1; break;
            case '<': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_COMMA_LESS); return 1; break;
            case '.': enqueue_press_release(KEYCAP_PERIOD_GREATER); return 1; break;
            case '>': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_PERIOD_GREATER); return 1; break;
            case '/': enqueue_press_release(KEYCAP_SLASH_QUESTION); return 1; break;
            case '?': enqueue_press_release(KEYCAP_LEFTSHIFT, KEYCAP_SLASH_QUESTION); return 1; break;
            default:
                return 0;
                break;
        }
    }
}

void CheckEvents(void)
{
    RoEvent ev;
    while(RoEventPoll(&ev))
    {
        switch(ev.eventType) {
            case RoEvent::KEYBOARD_RAW: {
                const struct KeyboardRawEvent raw = ev.u.keyboardRaw;
                printf("keyboard event: %s %d\n", raw.isPress ? "press" : "release", raw.key);
                break;
            }
            default:
                // pass;
                break;
        }
    }
}

const std::map<std::string, std::vector<std::string>> filesystem =
{
    {"/", { "model3.rom" } },
    {"coleco", { "COLECO.ROM", "Smurf - Rescue in Gargamel's Castle (1982).col", "Zaxxon (1982) (Sega).col" } },
};

Status RoFillFilenameList(const char* dirName, uint32_t flags, const char* optionalFilterSuffix, size_t maxNames, char **filenames, size_t* filenamesSize)
{
    Status status;

    const auto iter = filesystem.find(dirName);
    if (iter != filesystem.end()) {

        for (const auto& entry: iter->second) 
        {
            int addToList = 1;

            if (entry[entry.size() - 1] == '/') {                    /* It is a directory */
                // XXX Really should have a way to descend into directories.
                addToList = 0;
            } else if((entry[0] == '.') && (flags & CHOOSE_FILE_IGNORE_DOTFILES)) {
                addToList = 0;
            } else if(optionalFilterSuffix && (strcmp(optionalFilterSuffix, entry.c_str() + entry.size() - strlen(optionalFilterSuffix)) != 0)) {
                addToList = 0;
            }

            if(addToList) {
                if(*filenamesSize > maxNames - 1) {
                    return RO_RESOURCE_EXHAUSTED;
                }
                filenames[(*filenamesSize)++] = strdup(entry.c_str());
            }
        }
        status = RO_SUCCESS;

    } else {

        if(*filenamesSize > maxNames - 1) {
            return RO_RESOURCE_EXHAUSTED;
        }
        filenames[(*filenamesSize)++] = strdup("failed to find dir");
        status = RO_RESOURCE_NOT_FOUND;

    }

    return status;
}

};

