# Shooter's motor component

"""
How to control the fly wheel of the shooter with the talonFX motor.
(look into swerve drive as a reference for a fly wheel(2025 - reefscape))
 - velocity control
    - vary speed

 - speed is a function of the distance
"""

from phoenix6.hardware import TalonFX


# Code
class Shooter:
    def __init__(self):
        self.flywheel_motor = TalonFX(6)  # Assuming ID 6 for the flywheel motor

    def set_flywheel_speed(self, speed_rpm):
        """
        Set the flywheel speed in RPM.
        :param speed_rpm: Desired speed in revolutions per minute.
        """
        self.flywheel_motor.set_velocity(speed_rpm)

    def stop_flywheel(self):
        """
        Stop the flywheel motor.
        """
        self.flywheel_motor.set_velocity(0)


# This will need an excute function and an init function (both are needed)
