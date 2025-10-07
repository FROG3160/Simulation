from phoenix6.hardware import TalonFX


class myComponent:
    def __init__(self):
        self.motor = TalonFX(5)

    def set_height(self, height):
        self.motor.set_position(height)


#
