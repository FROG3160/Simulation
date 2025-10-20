import math
from navx import AHRS
from wpimath.geometry import Rotation2d


class FROGNavXGyro:
    """Gyro class that creates and instance of the NavX gyro and uses it to get AHRS data,
    converting it for use by the swerve drivetrain.  All swerve calculations use radians
    with CCW rotation being positive, and field-oriented driving uses moving forward and
    moving left as positive."""

    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.starting_angle = 0.0
        self.offset = 0
        # self.field_heading = 360-242
        # self.gyro.reset()
        self.gyro.setAngleAdjustment(self.offset)

    def getAngleCCW(self):
        # returns gyro heading
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return -self.gyro.getAngle()

    def getRoll(self):
        return self.gyro.getRoll()

    def getPitch(self):
        return self.gyro.getPitch()

    def setOffset(self, offset):
        self.offset = offset

    def getDegreesPerSecCCW(self):
        return -self.gyro.getRate()

    def getRadiansPerSecCCW(self):
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngleCCW())

    def resetGyro(self, on_red: bool):
        # sets yaw reading to 0
        if on_red:
            self.setAngleAdjustment(180)
        else:
            self.setAngleAdjustment(0)
        self.gyro.reset()

    def getAngleConstrained(self):
        angle = self.getAngle()
        return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

    def setAngleAdjustment(self, angle):
        self.gyro.setAngleAdjustment(angle)

    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()
