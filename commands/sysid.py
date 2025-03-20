from commands2 import Command
from wpilib import RobotController, Timer
from utils.mathutils import clamp
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine
from subsystems.drivetrain import Drivetrain, RomiDrivetrain
import commands2

class RomiSysId(commands2.Subsystem):
    def __init__(self, drivetrain: RomiDrivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.sysIdLog = SysIdRoutineLog()

        self.sysIdConfig = SysIdRoutine.Config(
            rampRate=0.25,
            timeout=10.0,
        )

        self.sysIdMechanism = SysIdRoutine.Mechanism(
            drive=self.drive_callback,
            log=self.log_data
        )

        self.sysIdRoutine = SysIdRoutine(self.sysIdConfig, self.sysIdMechanism)

    def drive_callback(self, volts: float):
        """Drives the Romi with a given voltage for SysId."""
        self.drivetrain.tank_drive_volts(volts, volts)

    def log_data(self, log: SysIdRoutineLog):
        """Correctly logs motor data for SysId."""
        timestamp = Timer.getFPGATimestamp()

        # Logging left motor
        log.motor("left_motor")\
            .voltage(self.drivetrain.left_motor.get())\
            .position(self.drivetrain.get_left_distance())\
            .velocity(self.drivetrain.get_left_encoder_rate())

        # Logging right motor
        log.motor("right_motor")\
            .voltage(self.drivetrain.right_motor.get())\
            .position(self.drivetrain.get_right_distance())\
            .velocity(self.drivetrain.get_right_encoder_rate())

        # Logging gyro data (optional but helpful for characterization)
        log.motor("gyro")\
            .position(self.drivetrain.get_gyro_angle())\
            .velocity(self.drivetrain.get_gyro_rate())

    def getSysIdRoutine(self):
        return self.sysIdRoutine