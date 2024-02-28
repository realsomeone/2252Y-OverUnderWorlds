# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       arnaldoalicea                                                #
# 	Created:      2/27/2024, 3:18:05 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain = Brain()
player = Controller()

trackwidth = 12.25 # Faker's measurements
wheelbase = 10
wheeldiam = 2.75
gearatio = 4/3
if brain.sdcard.is_inserted(): # load up pizazz from the SD Card
    brain.screen.draw_image_from_file('PR.png',0,0)
# region brain ports
change = False
lefttop = Motor(Ports.PORT1,GearSetting.RATIO_6_1,False)
leftmid = Motor(Ports.PORT2,GearSetting.RATIO_6_1,False)
leftbak = Motor(Ports.PORT3,GearSetting.RATIO_6_1,False)
rigttop = Motor(Ports.PORT4,GearSetting.RATIO_6_1,True)
rigtmid = Motor(Ports.PORT5,GearSetting.RATIO_6_1,True)
rigtbak = Motor(Ports.PORT6,GearSetting.RATIO_6_1,True)
lefty = MotorGroup(leftbak,leftmid,lefttop) # Motors in 1 side are controlled by MotorGroups
right = MotorGroup(rigtbak,rigtmid,rigttop)
# endregion
# region hybrid functions
def toggle(button,lastState,change):
    """Halt thread until current state toggles between current and not current.
    If bool, provide `lastState` and `change`.
    Used in that case as `while toggle(current,last,change): wait(5)`
    `change` must ideally be a value that is recursive:
    `cont,change = toggle(current,last,change)`

    Args:
        button (Button or Boolean): button (or bool) to check
        lastState (Boolean): should be constant, eg: if False, must maintain False throught whole excecution
        change (Boolean): manages our autonomous toggle, can be 
    """
    if isinstance(button,player.Button):
        lastState = button.pressing() # save current state to variable
        while button.pressing() == lastState: wait(5) # waits for button change
        while button.pressing() != lastState: wait(5) # waits for button change, again
    else: 
        if button == lastState and not change: return True,False # check if conditional has not changed from initial value
        elif button != lastState: return True,True  # condition changed, waiting for 2nd change
        else: return False,False # condition is OG value, continue.
def hold(button,lastState):
    """Halt thread until current state changes.

    Args:
        button (Button or Boolean): conditional to check
        lastState (Boolean): last state of same conditional
    """
    if isinstance(button,player.Button):
        lastState = button.pressing()
        while button.pressing() == lastState: wait(5) # wait for button change
    else:
        if button == lastState: return True # wait for state change
        else: return False
# endregion
# region driver functions
def baseCont():
    lefty.spin(FORWARD) # make sure our robot moves
    right.spin(FORWARD)
    while True: # links joystick positions directly to base motors
        lefty.set_velocity(player.axis3.position()+player.axis1.position()) 
        right.set_velocity(player.axis3.position()-player.axis1.position())
        wait(5)
# endregion
# region autonomous functions
def auton(): # code for autonomous, currently none
    pass
# endregion
# region competitions functions
driver = Event()
def autoF(): # Threads the autonomous code, compliant with competitive requirements
    active = Thread(auton)
    while comp.is_enabled() and comp.is_autonomous(): wait(10) # waits until auton period ends
    active.stop()
def drivF(): # Threads driver period, compliant with competitive requirements
    active = Thread(driver.broadcast)
    while comp.is_enabled() and comp.is_driver_control(): wait(10) # waits until driver period ends
    active.stop()
comp = Competition(drivF,autoF) # initiates our competition format
# endregion