import os
import wpilib
from wpilib import Joystick
import commands2
from subsystems.drivetrain import RomiDrivetrain
from commands.sysid import RomiSysId
from commands2.sysid import SysIdRoutine

os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"




class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.drivetrain = RomiDrivetrain()
        self.joystick = Joystick(0)
        self.sysId = RomiSysId(self.drivetrain)

        # Define SysId Commands
        self.sysIdQuasistaticForward = self.sysId.getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward)
        self.sysIdQuasistaticBackward = self.sysId.getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse)
        self.sysIdDynamicForward = self.sysId.getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward)
        self.sysIdDynamicBackward = self.sysId.getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse)

        # SmartDashboard selection
        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption("Quasistatic Forward", self.sysIdQuasistaticForward)
        self.chooser.addOption("Quasistatic Backward", self.sysIdQuasistaticBackward)
        self.chooser.addOption("Dynamic Forward", self.sysIdDynamicForward)
        self.chooser.addOption("Dynamic Backward", self.sysIdDynamicBackward)
        wpilib.SmartDashboard.putData("SysId Mode", self.chooser)

    def autonomousInit(self):
        self.sysIdCommand = self.chooser.getSelected()
        if self.sysIdCommand:
            self.sysIdCommand.schedule()


    def teleopPeriodic(self):
        forward = self.joystick.getY()
        rotation = self.joystick.getX()
        self.drivetrain.arcadeDrive(forward, rotation)
