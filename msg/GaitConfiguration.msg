# This message represents the gait configuration
#

# The type of gait to use
string gaitName

# Path to the logFile
string logFilePath

# True when moving forward, false for reverse walking
bool directionForward

# If true, the controller sends commands to adjust leg
# length and moves legs to starting position
bool prepareForGait

# Lengths given in MM, -1 when not setting leg lengths
float32[] femurLengths
float32[] tibiaLengths

# These contain all parameters for the chosen gait
string[] gaitParameterName
float32[] gaitParameterValue

