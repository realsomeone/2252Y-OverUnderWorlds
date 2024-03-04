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
intake = Motor(Ports.PORT7,GearSetting.RATIO_18_1,True)
fwing = DigitalOut(brain.three_wire_port.a)
lbwing = DigitalOut(brain.three_wire_port.b)
rbwing = DigitalOut(brain.three_wire_port.c)
elevmot = Motor(Ports.PORT8,GearSetting.RATIO_18_1,False)
ratchetLock = DigitalOut(brain.three_wire_port.d)
autonOpt = Optical(Ports.PORT12)
catapult = Motor(Ports.PORT9,GearSetting.RATIO_36_1,False)
catsens = Rotation(Ports.PORT13,False)
# endregion
# region hybrid functions
def toggle(button,lastState=True,change=0):
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
        lastState = button.pressing()                   # save current state to variable
        while button.pressing() == lastState: wait(5)   # waits for button change
        while button.pressing() != lastState: wait(5)   # waits for button change, again
    else: 
        if button == lastState and not change: return True,False        # check if conditional has not changed from initial value
        elif button != lastState: return True,True      # condition changed, waiting for 2nd change
        else: return False,False        # condition is OG value, continue.
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
def cataHide():
    if catapult.velocity(PERCENT) < 5 and catsens.angle() > maxrot - 15: 
    # check if catapult is not moving, and in range of holding
        catapult.set_stopping(COAST)    # release HOLD mode
    else:
        catapult.set_stopping(HOLD)     # make catapult HOLD
        catapult.spin(FORWARD)          # move catapult down
        while catsens.angle() < maxrot - 10: wait(5)    # wait until in good range
        catapult.stop()                 # stop catapult
    if hideTogg.pressing():
        hold(hideTogg)
def cataCalibration():
    global maxrot, minrot       # save vars as global variables
    maxrot, minrot = catsens.angle(), catsens.angle()       # set values to current position
    catsens.reset_position()    # reset Rotation
    catapult.spin(FORWARD)      #spin catapult
    while True:
        check = 0       # value making sure we have maxs and mins
        if catsens.angle() < minrot:    # check if our angle is less than minimum 
            minrot = catsens.angle()    # make current angle new minimum
        else: check += 1                # add to our check
        if catsens.angle() > maxrot:    # check if angle is more than maximum
            maxrot = catsens.angle()    # make current angle the new maximum
        else: check += 1                # add to out check
        if check == 2: break            # if both "if"s didn't change variables, break
        wait(5)
    catapult.stop()     # stop our catapult
def cata():
    catapult.set_stopping(COAST)    # makes sure catapult dosent get affected by hiding
    catapult.spin(FORWARD)          # spins catapult
    toggle(cataTogg)                # waits for the toggle
    catapult.stop()                 # stops catapult
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
def cataCont():
    cataCalibration()       # calibrate catapult before giving control
    global cataTogg, hideTogg   # globalize used buttons to access them in functions

    cataTogg = player.buttonR2  # assign actual buttons to these functions
    hideTogg = player.buttonX

    cataTogg.pressed(cata)      # assing each button its hybrid function
    hideTogg.pressed(cataHide)
# endregion
# region autonomous functions
def inertCheck(Tdis):
    vel = 0             # current robot velocity (inches/seconds)
    dis = 0             # distance ran by robot
    slowdown = False
    stabilization = Thread(inertStabil)         # stabilize robot's straights
    while True:
        # get current acceleration in inches/seconds^2
        accel = inert.acceleration(XAXIS) * 386.1 # type: ignore
        vel += accel * 0.05         # add change to velocity
        dis += vel * 0.05           # calculate distance from our current velocity
        if Tdis - dis < 5 and not slowdown:             # if remaning distance is less than 5 inches
            veldec(dtmots)
            slowdown = True         # makes sure this dosen't run again
        wait(5)
        if Tdis <= dis:             # check if distance is completed
            stabilization.stop()    # end stabilization
            break
def inertTCheck(Tturn):
    inert.reset_rotation()          # reset inertial rotation value
    slowdown = False
    while True: 
        amnt = inert.rotation()     # save current rotation amount
        if abs(amnt) > abs(Tturn) - 25 and not slowdown:    # check if theres 25 degress left in turning
            veldec(dtmots)          # slowdown base by 1/3
            slowdown = True         # makes sure this dosen't run again
        if abs(amnt) >= abs(Tturn): break               # exit when rotation reaches the threshold given
def inertStabil():
    inert.reset_rotation()              # reset rotation to have a constant reset point
    while True:
        if inert.rotation() < -5:       # if rotation is too much to the left, 
            og = veldec(right)          # reduce velocity on the right
            while not int(inert.rotation()) == 0: wait(5)   # wait until centered
            right.set_velocity(og,PERCENT)  # restore velocity
        elif inert.rotation() > 5:      # if rotation is too much to the right,
            og = veldec(lefty)          # reduce velocity on the left
            while not int(inert.rotation()) == 0: wait(5)   # wait until centered
            lefty.set_velocity(og,PERCENT)  # restore velocity
        wait(5)
def veldec(motor):
    retval = motor.velocity(PERCENT)    # save OG motor velocity
    if motor.count() > 3:               # check if we need to slow down whole base
        motor.set_velocity(motor.velocity()/3)          # change velocity to 1/3 its original value
    else: motor.set_velocity(motor.velocity() * 0.80)   # reduce velocity by 20%
    return retval           # return OG velocity, used if necessary
def autonDetect():
    if not autonOpt.installed(): return ""  # if sensor is disconnected, return empty
    autonOpt.set_light(LedStateType.ON)     # turn in light, not helpful, but we know when its working
    autonOpt.set_light_power(100)           # set intensity to max
    if autonOpt.is_near_object():           # check if theres an object covering the sensor
        ret = "offen"                       # return to run offensive side auton
    else:
        ret = "defen"                       # return to run defesive side auton
    autonOpt.set_light(LedStateType.OFF)    # turn off light
    return ret                              # return our variable
def move(dis):
    dir = dis / abs(dis)                    # get direction, -1 for backwards or 1 for forwards
    dtmots.set_velocity(80 * dir,PERCENT)   # set current velocity to a stable, precise velocity. multiplied by dir
    dtmots.spin(FORWARD)                    # start to move motors
    inertCheck(dis)                         # wait for exit from this function
    dtmots.stop()                           # stop motors
    wait(10)
def turn(theta):
    dir = theta / abs(theta)                # get direction of turn
    dtmots.set_velocity(60 * dir,PERCENT)   # set velocity to a stable vel. dir determines direction
    lefty.spin(FORWARD)                     # start motors
    right.spin(REVERSE)
    inertTCheck(theta)                      # wait until current rotation meets our needs
    dtmots.stop()                           # stop movement
def auton():
    check = autonDetect()       # check which autonomous should be ran
    dtmots.set_stopping(HOLD)   # set stopping to hold, should make everything more precise
    if check == "offen":    # offensive side auton
        pass
    elif check == "defen":  # defensive side auton
        pass
    else:                   # no auton; only used in emergencies
        pass
# endregion
# region competitions functions
driver = Event()
def autoF(): # Threads the autonomous code, compliant with competitive requirements
    active = Thread(auton)
    while comp.is_enabled() and comp.is_autonomous(): wait(10) # waits until auton period ends
    active.stop()
def drivF(): # Threads driver period, compliant with competitive requirements
    dtmots.set_stopping(COAST)
    active = Thread(driver.broadcast)
    while comp.is_enabled() and comp.is_driver_control(): wait(10) # waits until driver period ends
    active.stop()
comp = Competition(drivF,autoF) # initiates our competition format
driver(baseCont)        # these lines make sure the driver functions actually work
driver(intakeCont)
driver(WingsCont)
driver(ElevationCont)
driver(cataCont)
# endregion

fwing.set(False)        # make sure pistons don't expand in start 
rbwing.set(False)
lbwing.set(False)
ratchetLock.set(False)