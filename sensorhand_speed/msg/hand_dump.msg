# Message definition for dump messages of Otto Bock SensorHand Speed
#
# Institute for Robotics, JKU Linz
# Alexander Reiter
# January 2014

float32 V_open      # open electrode voltage in V
float32 V_close     # close electrode voltage in V
float32 V_bat       # supply voltage in V
uint8 state         # state number of the hand's internal state machine
float32 F_bracket   # bracket force in N
float32 F_pad1      # pad force in N
float32 F_pad2      # pad force in N
float32 F_pad3      # pad force in N
float32 I           # supply current in A
