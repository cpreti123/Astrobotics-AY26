# controller_lib.py
from inputs import get_gamepad
import math
import threading

class XboxController(object):
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.Left = 0
        self.Right = 0
        self.Up = 0
        self.Down = 0
        self.DPadY = 0
        self.DPadX = 0
        self.Select = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def readLeftStick_x(self):
        return -round(self.LeftJoystickX, 2)

    def readLeftStick_y(self):
        return -round(self.LeftJoystickY, 2)

    def readRightStick_x(self):
        return -round(self.RightJoystickX, 2)

    def readRightStick_y(self):
        return -round(self.RightJoystickY, 2)

    def readBtn(self):
        self.Up   = 1 if self.DPadY == -1 else 0
        self.Down = 1 if self.DPadY ==  1 else 0
        self.Left = 1 if self.DPadX == -1 else 0
        self.Right= 1 if self.DPadX ==  1 else 0

        # Note: X/Y swapped in your original mapping (kept as-is)
        return {
            "A": self.A, "Y": self.X, "X": self.Y, "B": self.B,
            "Select": self.Back, "Start": self.Start,
            "Left": self.Left, "Right": self.Right,
            "Up": self.Up, "Down": self.Down,
            "LeftThumb": self.LeftThumb, "RightThumb": self.RightThumb
        }

    def readBumper(self):
        if self.RightBumper:
            return "RB"
        if self.LeftBumper:
            return "LB"
        return None

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_HAT0Y': self.DPadY = event.state
                elif event.code == 'ABS_HAT0X': self.DPadX = event.state
                elif event.code == 'BTN_TR': self.RightBumper = event.state
                elif event.code == 'BTN_TL': self.LeftBumper = event.state
                elif event.code == 'BTN_SOUTH': self.A = event.state
                elif event.code == 'BTN_NORTH': self.Y = event.state
                elif event.code == 'BTN_WEST':  self.X = event.state
                elif event.code == 'BTN_EAST':  self.B = event.state
                elif event.code == 'BTN_SELECT': self.Back = event.state
                elif event.code == 'BTN_START':  self.Start = event.state
                elif event.code == 'BTN_THUMBL': self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR': self.RightThumb = event.state
                elif event.code == 'ABS_Y':  self.LeftJoystickY  = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_X':  self.LeftJoystickX  = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_RY': self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_RX': self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL
