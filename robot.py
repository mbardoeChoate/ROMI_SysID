

from commands2 import CommandScheduler, TimedCommandRobot
import os
from robotcontainer import SysIdRoutineBot

os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"
# Use the following command in terminal to start the ROMI control
# python3 -m robotpy sim --ws-client



class MyRobot(TimedCommandRobot):


    def robotInit(self) -> None:
        """This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.robot = SysIdRoutineBot()

        self.robot.configureBindings()

        self.autonomous_command = self.robot.getAutonomousCommand()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.autonomous_command.schedule()

    def teleopInit(self) -> None:
        self.autonomous_command.cancel()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        """This function is called periodically during test mode."""
        pass


