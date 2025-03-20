
from commands2 import Subsystem
from romi import RomiGyro
import wpimath.units
import commands2
import wpilib
import wpilib.drive
from wpilib import RobotController
import romi
from wpilib.drive import DifferentialDrive

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


class Drivetrain(commands2.Subsystem):
    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591

    def __init__(self) -> None:
        super().__init__()

        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = wpilib.Spark(0)
        self.rightMotor = wpilib.Spark(1)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)
        self.rightEncoder = wpilib.Encoder(6, 7)

        # Set up the differential drive controller
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotor, self.rightMotor)

        # Set up the RomiGyro
        self.gyro = romi.RomiGyro()

        # Set up the BuiltInAccelerometer
        self.accelerometer = wpilib.BuiltInAccelerometer()

        # # Use inches as unit for encoder distances
        # self.leftEncoder.setDistancePerPulse(
        #     (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        # )
        # self.rightEncoder.setDistancePerPulse(
        #     (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        # )

        # For ease of characterization, we will output the number of wheel rotations
        # (vs. actual distance)

        self.leftEncoder.setDistancePerPulse(1/ self.kCountsPerRevolution)
        self.rightEncoder.setDistancePerPulse(1 / self.kCountsPerRevolution)
        self.resetEncoders()

        self.rightMotor.setInverted(True)
        self.drive.setDeadband(0)


    def arcadeDrive(self, fwd: float, rot: float) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(fwd, rot, True)

    def tankDrive(self, leftSpeed: float, rightSpeed: float) -> None:
        self.drive.tankDrive(leftSpeed, rightSpeed)

    def tankDriveVolts(self, leftVolts: float, rightVolts: float) -> None:
        self.leftMotor.setVoltage(leftVolts)
        self.rightMotor.setVoltage(rightVolts)
        self.drive.feed()

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getLeftEncoderCount(self) -> int:
        return self.leftEncoder.get()

    def getRightEncoderCount(self) -> int:
        return self.rightEncoder.get()

    def getLeftDistance(self) -> float:
        return self.leftEncoder.getDistance()

    def getRightDistance(self) -> float:
        return self.rightEncoder.getDistance()

    def getLeftEncoderRate(self) -> float:
        return self.leftEncoder.getRate()

    def getRightEncoderRate(self) -> float:
        return self.rightEncoder.getRate()

    def getAverageDistanceInch(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return (self.getLeftDistance() + self.getRightDistance()) / 2.0

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the Romi along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the Romi along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the Romi along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the Romi around the X-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the Romi around the Y-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the Romi around the Z-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleZ()

    def resetGyro(self) -> None:
        """Reset the gyro"""
        self.gyro.reset()

