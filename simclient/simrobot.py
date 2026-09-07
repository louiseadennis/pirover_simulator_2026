"""
1. User imports simrobot.
2. init() creates a SimulatorClient.
3. Functions such as:
      - forward()
      - stop()
      - setLED()
      - getDistance()
      - irLeft()
   simply call the corresponding SimulatorClient function.
4. SimulatorClient handles UDP communication with Ben_simulator.
5. Sensor results are returned back through simrobot to the user program.

User Program
     ↓
simrobot
     ↓
simclient
     ↓ UDP
Ben_simulator

"""

from .simclient import SimulatorClient

PAN = 1
VERSION = 1
sim = None


def init():
    """initializes the simulator client"""
    try:
        cleanup()
    except:
        pass
    global sim
    sim = SimulatorClient()


def cleanup():
    """stops the simulator client"""
    global sim
    sim.stop()
    sim.cleanup()
    pass


def version():
    """returns the version, in the case of the sim client this always returns 1"""
    return VERSION


def startServos():
    """Has no effect, added to keep compatibility with real robot"""
    pass


def stopServos():
    """Has no effect, added to keep compatibility with real robot"""
    pass


def startServod():
    """Has no effect, added to keep compatibility with real robot"""
    pass


def pinServod():
    """Has no effect, added to keep compatibility with real robot"""
    pass


def stopServod():
    """Has no effect, added to keep compatibility with real robot"""
    pass


def getRobotName():
    global sim
    return sim.getRobotName()


def setServo(servo, degrees):
    """Sets the servo to position in degrees -90 to +90"""
    global sim
    sim.setServo(servo, degrees)


def getDistance():
    """Returns the distance to the nearest reflecting object"""
    global sim
    return sim.getDistance()


def irLeft():
    """Returns state of Left IR Obstacle sensor"""
    global sim
    return sim.irLeft()


def irRight():
    """Returns state of Right IR Obstacle sensor"""
    global sim
    return sim.irRight()


def irCentre():
    """Returns state of Centre IR Obstacle sensor (Pi2Go only)"""
    global sim
    return sim.irCentre()


def irAll():
    """Returns true if any of the Obstacle sensors are triggered"""
    global sim
    return sim.irAll()


def irLeftLine():
    """Returns state of Left IR Line sensor"""
    global sim
    return sim.irLeftLine()


def irRightLine():
    """Returns state of Right IR Line sensor"""
    global sim
    return sim.irRightLine()


def forward(speed):
    """Sets both motors to move forward at speed. 0 <= speed <= 100"""
    global sim
    sim.forward(speed)


def reverse(speed):
    """Sets both motors to reverse at speed. 0 <= speed <= 100"""
    global sim
    sim.reverse(speed)


def spinLeft(speed):
    """Sets motors to turn opposite directions at speed. 0 <= speed <= 100"""
    global sim
    sim.spinLeft(speed)


def spinRight(speed):
    """Sets motors to turn opposite directions at speed. 0 <= speed <= 100"""
    global sim
    sim.spinRight(speed)


def turnForward(left_speed, right_speed):
    """Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100"""
    global sim
    sim.turnForward(left_speed, right_speed)


def turnReverse(left_speed, right_speed):
    """Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100"""
    global sim
    sim.turnReverse(left_speed, right_speed)


def stop():
    """Stops both motors"""
    global sim
    sim.stop()


#======================================================================
# Pi2Go / Pi2Go2 functions
#======================================================================

def getSwitch():
    """Returns the value of the tact switch: True==pressed"""
    global sim
    return sim.getSwitch()


def getLight(sensor):
    """Returns the value 0..1023 for the selected sensor, 0 <= Sensor <= 3"""
    global sim
    return sim.getLight(sensor)


def getLightFL():
    """Returns the value 0..1023 for Front-Left light sensor"""
    global sim
    return sim.getLightFL()


def getLightFR():
    """Returns the value 0..1023 for Front-Right light sensor"""
    global sim
    return sim.getLightFR()


def getLightBL():
    """Returns the value 0..1023 for Back-Left light sensor"""
    global sim
    return sim.getLightBL()


def getLightBR():
    """Returns the value 0..1023 for Back-Right light sensor"""
    global sim
    return sim.getLightBR()


def setLED(LED, red, green, blue):
    """Sets selected RGB LED. Pi2Go uses 0..7, Pi2Go2 uses 0..9."""
    global sim
    sim.setLED(LED, red, green, blue)


def setAllLEDs(red, green, blue):
    """Sets all LEDs to required RGB."""
    global sim
    sim.setAllLEDs(red, green, blue)


def getLED(LED):
    """Gets the RGB value of the specified LED."""
    global sim
    return sim.getLED(LED)


def getAllLEDs():
    """Gets RGB values for all LEDs on the connected robot."""
    global sim
    return sim.getAllLEDs()


#======================================================================
# Pi2Go2 wheel encoders
#======================================================================

def getEncoderLeft():
    """Returns left wheel encoder count on Pi2Go2"""
    global sim
    return sim.getEncoderLeft()


def getEncoderRight():
    """Returns right wheel encoder count on Pi2Go2"""
    global sim
    return sim.getEncoderRight()


def resetEncoders():
    """Resets both Pi2Go2 encoder counts"""
    global sim
    sim.resetEncoders()
