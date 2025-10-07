from phoenix6.hardware import talonFX
class MyComponent:
	def __init__(self):
		#initialized the component.

		self.name = "motor"
		self.motor = talonFX(0)
        #info methods about motor
		
	def set_position(self, position):
    self.motor.set_position(position)