from phoenix6.hardware import TalonFX
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
from phoenix6.configs import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    MotionMagicConfigs,
)

class MyComponent:
    def __init__(self):
    # initialize the component.
        self.name = "motor"
        self.motor = FROGTalonFX(
            id: int=1,
            motor_config: FROGTalonFXConfig = FROGTalonFXConfig(
            feedback_config=FROGFeedbackConfig(),
            slot1gains=Slot1Configs()
                .with_k_a(1)
                .with_k_p(1)
                .with_k_s(1)
                .with_k_v(1)
                .with_k_i(1), # Need to edit the values once i found out what they should be.
            )
            parent_nt="Arm",
            motor_name="motor",
    # Configure the motor
        )



    def reset_position(self):
        self.motor.set_position(0)
        self._enable_software_limits()

    def _enable_software_limits(self):
        self.motor.config.with_software_limit_switch(
            SoftwareLimitSwitchConfigs()
            .with_reverse_soft_limit_enable(True)
            .with_reverse_soft_limit_threshold(0.0)
            .with_forward_soft_limit_enable(True)
            .with_forward_soft_limit_threshold(1)
        )
    #Set motor range of motion.

    # Set home/starting Position.

    # Have two set positions for motor to go to.
