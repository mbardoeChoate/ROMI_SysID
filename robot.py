import time
import wpilib
import wpimath
import ntcore
from wpilib import TimedRobot, Timer, RobotController, XboxController
from romi import RomiGyro
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain

class QuasistaticVoltageSupplier:
    def __init__(self, ramp_rate):
        self.ramp_rate = ramp_rate
        self.current_voltage = 0.0
        self.counter = 0

    def get(self):
        # Simple rate limiter
        self.counter += 1
        if self.counter == 10:
            self.current_voltage = self.ramp_rate * (Timer.getFPGATimestamp() - Robot.start_time)
            self.counter = 0
        return self.current_voltage

class Robot(TimedRobot):
    def __init__(self):
        super().__init__(period=0.005)  # 5ms loop
        self.stick = None
        self.left_encoder_position = None
        self.right_encoder_position = None
        self.left_encoder_rate = None
        self.right_encoder_rate = None
        self.gyro_angle_radians = None
        self.gyro_angle_rate = None
        self.voltage_supplier = None

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.sysIdTelemetryEntry = self.nt_instance.getEntry("/SmartDashboard/SysIdTelemetry")
        self.sysIdVoltageCommandEntry = self.nt_instance.getEntry("/SmartDashboard/SysIdVoltageCommand")
        self.sysIdTestTypeEntry = self.nt_instance.getEntry("/SmartDashboard/SysIdTestType")
        self.sysIdRotateEntry = self.nt_instance.getEntry("/SmartDashboard/SysIdRotate")
        self.sysIdTestEntry = self.nt_instance.getEntry("/SmartDashboard/SysIdTest")
        self.sysIdWrongMechEntry = self.nt_instance.getEntry("/SmartDashboard/SysIdWrongMech")
        self.sysIdOverflowEntry = self.nt_instance.getEntry("/SmartDashboard/SysIdOverflow")

        self.counter = 0
        self.start_time = 0
        self.motor_voltage = 0
        self.number_array = [0] * 9
        self.entries = []

        self.drivetrain = Drivetrain()
        self.gyro = RomiGyro()

    def robotInit(self):
        self.stick = XboxController(0)

        # Reverse sign of gyro angle to match coordinate convention
        self.gyro_angle_radians = lambda: -1 * wpimath.units.degreesToRadians(self.gyro.getAngleZ())
        self.gyro_angle_rate = lambda: self.gyro.getRateZ()

        self.left_encoder_position = self.drivetrain.getLeftDistance
        self.left_encoder_rate = self.drivetrain.getLeftEncoderRate
        self.right_encoder_position = self.drivetrain.getRightDistance
        self.right_encoder_rate = self.drivetrain.getRightEncoderRate

        self.nt_instance.setUpdateRate(0.010)

    def robotPeriodic(self):
        SmartDashboard.putNumber("Left Encoder Position", self.left_encoder_position())
        SmartDashboard.putNumber("Left Encoder Rate", self.left_encoder_rate())
        SmartDashboard.putNumber("Right Encoder Position", self.right_encoder_position())
        SmartDashboard.putNumber("Right Encoder Rate", self.right_encoder_rate())
        SmartDashboard.putNumber("Motor Voltage", self.motor_voltage)

    def autonomousInit(self):
        print("Robot in autonomous mode")
        self.drivetrain.resetEncoders()
        self.gyro.reset()

        self.start_time = Timer.getFPGATimestamp()
        self.voltage_supplier = self.createVoltageSupplier()
        self.counter = 0

    def createVoltageSupplier(self):
        test = self.sysIdTestEntry.getString(None)
        if test not in ["Drivetrain", "Drivetrain (Angular)"]:
            self.sysIdWrongMechEntry.setBoolean(True)
        else:
            req_voltage = self.sysIdVoltageCommandEntry.getDouble(0)
            test_type = self.sysIdTestTypeEntry.getString(None)

            if test_type == "Quasistatic":
                return QuasistaticVoltageSupplier(req_voltage).get
            elif test_type == "Dynamic":
                return lambda: req_voltage

        return lambda: 0.0

    def autonomousPeriodic(self):
        now = Timer.getFPGATimestamp()

        left_position = self.left_encoder_position()
        left_rate = self.left_encoder_rate()
        right_position = self.right_encoder_position()
        right_rate = self.right_encoder_rate()
        gyro_angle = self.gyro_angle_radians()
        gyro_angle_rate = self.gyro_angle_rate()

        left_motor_volts = self.motor_voltage
        right_motor_volts = self.motor_voltage

        battery = RobotController.getBatteryVoltage()
        self.motor_voltage = clamp(self.voltage_supplier(), -battery, battery)

        self.drivetrain.tankDriveVolts(
            (-1 if self.sysIdRotateEntry.getBoolean(False) else 1) * self.motor_voltage,
            self.motor_voltage
        )

        self.number_array = [
            now, left_motor_volts, right_motor_volts,
            left_position, right_position, left_rate,
            right_rate, gyro_angle, gyro_angle_rate
        ]

        self.entries.extend(self.number_array)
        self.counter += 1

    def teleopInit(self):
        print("Robot in operator control mode")

    def teleopPeriodic(self):
        self.drivetrain.arcadeDrive(-self.stick.getLeftY(), self.stick.getRightX())

    def disabledInit(self):
        elapsed_time = Timer.getFPGATimestamp() - self.start_time
        print("Robot disabled")
        self.drivetrain.tankDriveVolts(0, 0)

        self.sysIdOverflowEntry.setBoolean(len(self.entries) >= 36_000)

        data = ",".join(map(str, self.entries))
        self.sysIdTelemetryEntry.setString(data)
        self.entries.clear()

        print(f"Collected: {self.counter} in {elapsed_time} seconds")

    def disabledPeriodic(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

if __name__ == "__main__":
    wpilib.run(Robot)
