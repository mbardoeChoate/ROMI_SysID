
from commands2 import Subsystem, Command
from typing import Callable


from commands2.sysid import SysIdRoutine
from romi import RomiGyro
import wpimath.units
from wpilib.sysid import SysIdRoutineLog

import wpilib
import wpilib.drive
from wpilib import RobotController, DataLogManager
from wpimath.units import volts
from wpilib.drive import DifferentialDrive
from wpiutil.log import DoubleLogEntry, StringLogEntry

from utils.mathutils import clamp


class RomiDrivetrain(Subsystem):
    kCountsPerRevolution = 1440.0

    def __init__(self):
        super().__init__()
        self.setName("RomiDrivetrain")
        self.left_motor = wpilib.Spark(0)
        self.right_motor = wpilib.Spark(1)
        self.left_motor.setInverted(False)
        self.right_motor.setInverted(True)

        self.left_encoder = wpilib.Encoder(4, 5)
        self.right_encoder = wpilib.Encoder(6, 7)

        # For ease of characterization, we will output the number of wheel rotations
        # (vs. actual distance)

        self.left_encoder.setDistancePerPulse(1/ self.kCountsPerRevolution)
        self.right_encoder.setDistancePerPulse(1 / self.kCountsPerRevolution)
        self.reset_encoders()
        self.drive=DifferentialDrive(self.left_motor, self.right_motor)

        self.gyro = RomiGyro()

        self.log = DataLogManager.getLog()
        self.leftPositionLog = DoubleLogEntry(self.log, "/drive/leftPosition")
        self.rightPositionLog = DoubleLogEntry(self.log, "/drive/rightPosition")
        self.leftVelocityLog = DoubleLogEntry(self.log, "/drive/leftVelocity")
        self.rightVelocityLog = DoubleLogEntry(self.log, "/drive/rightVelocity")
        self.motorVoltageLog = DoubleLogEntry(self.log, "/drive/appliedVoltage")
        self.testTypeLog = StringLogEntry(self.log, "/sysid/testType")

        # Tell SysId how to plumb the driving voltage to the motors.
        def drive(voltage: volts) -> None:
            self.left_motor.setVoltage(voltage)
            self.right_motor.setVoltage(voltage)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(recordState=self.logSysId),
            SysIdRoutine.Mechanism(drive, self.recordState, self),
        )

    def logSysId(self, state: wpilib.sysid.State):
        self.testTypeLog.append(SysIdRoutineLog.stateEnumToString(state))

    def recordState(self, thing: SysIdRoutineLog):
        # Log encoders
        self.leftPositionLog.append(self.left_encoder.getDistance())
        self.rightPositionLog.append(self.right_encoder.getDistance())
        self.leftVelocityLog.append(self.left_encoder.getRate())
        self.rightVelocityLog.append(self.right_encoder.getRate())
        # You may want to log battery voltage here too
        self.motorVoltageLog.append(self.left_motor.get() * RobotController.getBatteryVoltage())

    def reset_encoders(self):
        self.left_encoder.reset()
        self.right_encoder.reset()

    def get_left_distance(self):
        return self.left_encoder.getDistance()

    def get_right_distance(self):
        return self.right_encoder.getDistance()

    def get_left_encoder_rate(self):
        return self.left_encoder.getRate()

    def get_right_encoder_rate(self):
        return self.right_encoder.getRate()

    def tank_drive_volts(self, left_volts, right_volts):
        battery_voltage = RobotController.getBatteryVoltage()
        left_speed = clamp(left_volts / battery_voltage, -1, 1)
        right_speed = clamp(right_volts / battery_voltage, -1, 1)
        self.left_motor.set(left_speed)
        self.right_motor.set(right_speed)


    def stop(self):
        self.left_motor.set(0)
        self.right_motor.set(0)

    def get_gyro_angle(self):
        return wpimath.units.degreesToRadians(-self.gyro.getAngleZ())

    def get_gyro_rate(self):
        return self.gyro.getRateZ()

    def arcadeDrive(self, fwd: float, rot: float) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(fwd, rot, True)

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        sys_id_routine.motor("drive-left").voltage(
            self.left_motor.get() * RobotController.getBatteryVoltage()
        ).position(self.left_encoder.getDistance()).velocity(
            self.left_encoder.getRate()
        )
        # Record a frame for the right motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        sys_id_routine.motor("drive-right").voltage(
            self.right_motor.get() * RobotController.getBatteryVoltage()
        ).position(self.right_encoder.getDistance()).velocity(
            self.right_encoder.getRate()
        )
    def arcadeDriveCommand(
        self, fwd: Callable[[], float], rot: Callable[[], float]
    ) -> Command:
        """Returns a command that drives the robot with arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """

        # A split-stick arcade command, with forward/backward controlled by the left
        # hand, and turning controlled by the right.
        return self.run(lambda: self.drive.arcadeDrive(fwd(), rot()))

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)


