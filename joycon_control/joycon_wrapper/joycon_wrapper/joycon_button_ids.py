"""
Uint8 button_id values for custom_msgs/JoyconDataButton (right Joy-Con).

IDs2,3,8 match joycon-sdk ``control_button`` when ``all_button_return=True``.
10,11 are reserved for Y/A (not present in control_button bitmask).
"""

# joyconrobotics all_button_return / control_button
BUTTON_B = 2
BUTTON_X = 3
BUTTON_PLUS = 8

# Published only on joycon_data_buttons (edge events)
BUTTON_Y = 10
BUTTON_A = 11
