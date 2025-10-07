from phoenix6.hardware import TalonFX


class MyComponent:
    def __init__(self):
        # initialized the component.

        self.name = "motor"
        self.motor = TalonFX(0)
        # info methods about motor

    def set_position(self, position):
        self.motor.set_position()
