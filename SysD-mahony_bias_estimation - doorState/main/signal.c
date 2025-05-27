// anywhere in your code, when session is ending:
#include "door_state.h"

xEventGroupSetBits(door_event_group, DOOR_EVENT_SESSION_END);

// — or using the helper —
door_signal_session_end();
