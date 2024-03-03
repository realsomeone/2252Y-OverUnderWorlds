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
lefttop = Motor(Ports.PORT1,GearSetting.RATIO_6_1,True)
leftmid = Motor(Ports.PORT2,GearSetting.RATIO_6_1,True)
leftbak = Motor(Ports.PORT3,GearSetting.RATIO_6_1,True)
rigttop = Motor(Ports.PORT4,GearSetting.RATIO_6_1,False)
rigtmid = Motor(Ports.PORT5,GearSetting.RATIO_6_1,False)
rigtbak = Motor(Ports.PORT6,GearSetting.RATIO_6_1,False)
lefty = MotorGroup(leftbak,leftmid,lefttop) # Motors in 1 side are controlled by MotorGroups
right = MotorGroup(rigtbak,rigtmid,rigttop)
dtmots = MotorGroup(lefty,right)            # all drivetrain motors in a MotorGroup
inert = Inertial(Ports.PORT11)
rL = Rotation(Ports.PORT12,False)
rR = Rotation(Ports.PORT13,True)
intake = Motor(Ports.PORT7,GearSetting.RATIO_18_1,True)
fwing = DigitalOut(brain.three_wire_port.a)
lbwing = DigitalOut(brain.three_wire_port.b)
rbwing = DigitalOut(brain.three_wire_port.c)
elevmot = Motor(Ports.PORT8,GearSetting.RATIO_18_1,False)
ratchetLock = DigitalOut(brain.three_wire_port.d)
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
def hold(button,lastState=0):
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
def intakeIn():
    intake.spin(FORWARD)            # spins intake
    if intaketrigF.pressing():  # check if a button is pressed
        while hold(intaketrigF): wait(5)  # while button is held, wait
        intake.stop()               # stops intake when done
def intakeOut():
    intake.spin(REVERSE)            # spins intake
    if intaketrigR.pressing():  # check if a button is pressed
        while hold(intaketrigR): wait(5) # while button is held, wait
        intake.stop()               # stops intake when done
def FWings():
    if not fwing.value():           # check if wings are expanded
        fwing.set(True)             # expand wings
        if player.buttonR1.pressing():              # check if driver summoned this function
            while hold(player.buttonR1): wait(5)    # if yes, wait for release. Else, end function
            fwing.set(False)
    else:                   # if wings are expanded (auton summoned the function),
        fwing.set(False)    # hide wings
def LBWing():
    if not lbwing.value():          # check if wings are expanded
        lbwing.set(True)            # expand wings
        if player.buttonDown.pressing():            # check if driver summoned this function
            while hold(player.buttonDown): wait(5)  # if yes, wait for release. Else, end function
            lbwing.set(False)
    else:                   # if wings are expanded (auton summoned the function),
        lbwing.set(False)   # hide wings
def RBWing():
    if not rbwing.value():          # check if wings are expanded
        rbwing.set(True)            # expand wings
        if player.buttonB.pressing():               # check if driver summoned this function
            while hold(player.buttonB): wait(5)     # if yes, wait for release. Else, end function
            rbwing.set(False)
    else:                   # if wings are expanded (auton summoned the function),
        rbwing.set(False)   # hide wings
def elev(dir=""):
    if elevmot.velocity() < 5:      # check if elevation is not spinning
        if dir == "up": elevmot.spin(FORWARD)               # lift elevation up
        if dir == "down": elevmot.spin(REVERSE)             # move elevation down
        if elevUp.pressing() or elevDown.pressing():
            while hold(elevUp) or hold(elevDown): wait(5)   # wait until the button is let go
            elevmot.stop()                                  # stop the elevation 
    else:       # if elevation is moving (auton summoned the function)
        if not (elevUp.pressing() or elevDown.pressing()):  # double check if auton summoned the function 
            elevmot.stop()      # stop the elevation
def ratchlock():
    wait(1,SECONDS)             # wait a second, driver should hold the button to trigger the function
    if locktrig.pressing():     # check if driver held the button for a second
        ratchetLock.set(True)   # engage ratchet lock
# endregion
# region driver functions
def baseCont():
    lefty.spin(FORWARD) # make sure our robot moves
    right.spin(FORWARD)
    while True: # links joystick positions directly to base motors
        lefty.set_velocity(player.axis3.position()+player.axis1.position(),PERCENT) 
        right.set_velocity(player.axis3.position()-player.axis1.position(),PERCENT)
        wait(5)
def intakeCont():
    global intaketrigF, intaketrigR     # globalize buttons, makes change easier

    intaketrigF = player.buttonL2       # save buttons to variable
    intaketrigR = player.buttonL1

    intake.set_velocity(100,PERCENT)    # set motor to max vel
    intaketrigF.pressed(intakeIn)       # assign L2 to pickup triball
    intaketrigR.pressed(intakeOut)      # assign L1 to spit out triball
def WingsCont():
    global fwingtrig, lwingtrig, rwingtrig

    fwingtrig = player.buttonR1         # save buttons to variables
    lwingtrig = player.buttonDown
    rwingtrig = player.buttonB

    player.buttonR1.pressed(FWings)     # assign each button to its hybrid function
    player.buttonB.pressed(RBWing)
    player.buttonDown.pressed(LBWing)
def ElevationCont():
    global elevUp, elevDown, locktrig

    elevUp = player.buttonUp            # save buttons to global variables
    elevDown = player.buttonY
    locktrig = player.buttonLeft

    elevUp.pressed(elev,tuple("up"))    # assign hybrid function with arguments to buttons
    elevDown.pressed(elev,tuple("down"))
    locktrig.pressed(ratchlock)
# endregion
# region autonomous functions
def inertCheck(Tdis):
    vel = 0             # current robot velocity (inches/seconds)
    dis = 0             # distance ran by robot
    slowdown = False
    while True:
        # get current acceleration in inches/seconds^2
        accel = inert.acceleration(XAXIS) * 386.1 # type: ignore
        vel += accel * 0.05         # add change to velocity
        dis += vel * 0.05           # calculate distance from our current velocity
        if Tdis - dis < 5 and not slowdown:             # if remaning distance is less than 5 inches
            lefty.set_velocity(lefty.velocity()/3)      # reduce wheel velocity to 1/3 its velocity
            right.set_velocity(right.velocity()/3)
            slowdown = True         # makes sure this dosen't run again
        wait(5)
        if Tdis <= dis:             # check if distance is completed
            break
def inertTCheck(Tturn):
    inert.reset_rotation()          # reset inertial rotation value
    slowdown = False
    while True: 
        amnt = inert.rotation()     # save current rotation amount
        if abs(amnt) > abs(Tturn) - 25 and not slowdown:    # check if theres 25 degress left in turning
            dtmots.set_velocity(dtmots.velocity()/3)    # slowdown base by 1/3
            slowdown = True         # makes sure this dosen't run again
        if abs(amnt) >= abs(Tturn): break               # exit when rotation reaches the threshold given
def odomCheck(dis):
    rL.set_position(0) # reset positions of rotations
    rR.set_position(0)
    slowdown = False
    while True:
        pL = ((rL.position() * math.pi * wheeldiam) / dis) * 100    # convert distances into %s, clearer end condition
        pR = ((rR.position() * math.pi * wheeldiam) / dis) * 100
        fac = (0.25 / dis) * 100    # factor for correction threshold
        slFac = (5 / dis) * 100     # factor for ending slowdown
        if pL - pR > fac:           # check if deviation is enough
            vel = veldec(lefty)     # slow down deviated side
            while pL - pR > fac - ((0.1 / dis) * 100): wait(5)  # wait until deviation is sufficiently small
            lefty.set_velocity(vel,PERCENT)     # restore OG velocity
        elif pR - pL > fac:         # same as other if, but specified for the other side
            vel = veldec(right)     # slow down deviated side
            while pL - pR > fac - ((0.1 / dis) * 100): wait(5)  # wait until deviation is sufficiently small
            right.set_velocity(vel,PERCENT)     # restore OG velocity
        if (pL >= slFac or pR >= slFac) and not slowdown:       # if either side has 5 inches left in its movement
            veldec(dtmots)          # slows down whole base
            slowdown = True         # makes sure this dosen't run again
        if (pL >= 100 or pR >= 100) and slowdown: break # checks for 100% completion
def veldec(motor):
    retval = motor.velocity(PERCENT)    # save OG motor velocity
    if motor.count() > 3:               # check if we need to slow down whole base
        motor.set_velocity(motor.velocity()/3)          # change velocity to 1/3 its original value
    else: motor.set_velocity(motor.velocity() * 0.80)   # reduce velocity by 20%
    return retval           # return OG velocity, used if necessary
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
driver(baseCont)        # these lines make sure the driver functions actually work
driver(intakeCont)
driver(WingsCont)
driver(ElevationCont)
# endregion

fwing.set(False)        # make sure pistons don't expand in start 
rbwing.set(False)
lbwing.set(False)
ratchetLock.set(False)