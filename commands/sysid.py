import os
import ntcore
from commands2 import Command
from wpilib import RobotController, Timer
from utils.mathutils import clamp
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine
from subsystems.drivetrain import Drivetrain, RomiDrivetrain
import commands2
import csv

class RomiSysId(commands2.Subsystem):
    def __init__(self, drivetrain: RomiDrivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.sysIdLog = SysIdRoutineLog("sysid-test-state-Drivetrain")

        # Define log directory and file
        self.log_dir = "/Users/matthewbardoe/logs"
        self.log_filename = os.path.join(self.log_dir, "sysid_data.csv")

        # Ensure the logs directory exists
        os.makedirs(self.log_dir, exist_ok=True)

        self.sysIdConfig = SysIdRoutine.Config(
            rampRate=0.25,
            timeout=10.0,
        )

        # Initialize NetworkTables
        # Initialize NetworkTables for SysId
        self.nt_inst = ntcore.NetworkTableInstance.getDefault()
        self.sysIdTable = self.nt_inst.getTable("SmartDashboard")  # SmartDashboard is required

        self.sysIdMechanism = SysIdRoutine.Mechanism(
            drive=self.drive_callback,
            subsystem=self.drivetrain,
            log=self.log_data
        )

        self.sysIdRoutine = SysIdRoutine(self.sysIdConfig, self.sysIdMechanism)
        self.log_filename = "/Users/matthewbardoe/logs/sysid_data.csv"
        with open(self.log_filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                "timestamp",
                "left_motor_volts", "right_motor_volts",
                "left_motor_position", "right_motor_position",
                "left_motor_velocity", "right_motor_velocity",
                "gyro_angle", "gyro_velocity"
            ])

    def drive_callback(self, volts: float):
        """Drives the Romi with a given voltage for SysId."""
        self.drivetrain.tank_drive_volts(volts, volts)

    def log_data(self, log: SysIdRoutineLog):
        """Correctly logs motor data for SysId and prints debug info."""
        timestamp = Timer.getFPGATimestamp()

        print(f"Logging Data at {timestamp}")  # Debug Print

        # Logging left motor
        log.motor("left_motor") \
            .voltage(self.drivetrain.left_motor.get()) \
            .position(self.drivetrain.get_left_distance()) \
            .velocity(self.drivetrain.get_left_encoder_rate())

        # Logging right motor
        log.motor("right_motor") \
            .voltage(self.drivetrain.right_motor.get()) \
            .position(self.drivetrain.get_right_distance()) \
            .velocity(self.drivetrain.get_right_encoder_rate())

        # Logging gyro data
        log.motor("gyro") \
            .position(self.drivetrain.get_gyro_angle()) \
            .velocity(self.drivetrain.get_gyro_rate())

        # Send to NetworkTables
        self.sysIdTable.getEntry("timestamp").setDouble(timestamp)
        self.sysIdTable.getEntry("left_motor_volts").setDouble(self.drivetrain.left_motor.get())
        self.sysIdTable.getEntry("right_motor_volts").setDouble(self.drivetrain.right_motor.get())
        self.sysIdTable.getEntry("left_motor_position").setDouble(self.drivetrain.get_left_distance())
        self.sysIdTable.getEntry("right_motor_position").setDouble(self.drivetrain.get_right_distance())
        self.sysIdTable.getEntry("left_motor_velocity").setDouble(self.drivetrain.get_left_encoder_rate())
        self.sysIdTable.getEntry("right_motor_velocity").setDouble(self.drivetrain.get_right_encoder_rate())
        self.sysIdTable.getEntry("gyro_position").setDouble(self.drivetrain.get_gyro_angle())
        self.sysIdTable.getEntry("gyro_velocity").setDouble(self.drivetrain.get_gyro_rate())

        with open(self.log_filename, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp,
                self.drivetrain.left_motor.get(),
                self.drivetrain.get_left_distance(),
                self.drivetrain.get_left_encoder_rate(),
                self.drivetrain.right_motor.get(),
                self.drivetrain.get_right_distance(),
                self.drivetrain.get_right_encoder_rate(),
                self.drivetrain.get_gyro_angle(),
                self.drivetrain.get_gyro_rate()
            ])

        print("SysId Data Logged Successfully!")  # Debug Print

    def getSysIdRoutine(self):
        return self.sysIdRoutine