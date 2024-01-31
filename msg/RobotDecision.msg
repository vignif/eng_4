# Decision space over a robot's gaze, gesture, speech and the target of these other actions

Header header  

# Gestures
uint8 GESTURE_NOTHING = 0           # Do nothing
uint8 GESTURE_WAVE = 1              # Wave
uint8 GESTURE_ALIVE = 2             # Idle gesture to show robot is active

# Gaze
uint8 GAZE_AHEAD = 0                # Look straight ahead
uint8 GAZE_TARGET = 1               # Look at the target

# Speech
uint8 SPEECH_NOTHING = 0            # Don't say anything
uint8 SPEECH_GREETING_INFORMAL = 1  # An informal greeting like "Hi"
uint8 SPEECH_GREETING_FORMAL = 2    # A more formal greeting like "Good afternoon"
uint8 SPEECH_BECKON_ROBOT = 3       # Try and attract a person, promising an interaction with the robot e.g. "Come and talk with me"
uint8 SPEECH_BECKON_TABLET = 4      # Try and attract a person, promising an interaction with the robot's interface e.g. "Come and see what's on my tablet"
uint8 SPEECH_RECAPTURE = 5          # Try and preemptively recapture someone who is leaving e.g. "Are you leaving already?"


uint8 gesture
uint8 gaze
uint8 speech
bool wait
string target