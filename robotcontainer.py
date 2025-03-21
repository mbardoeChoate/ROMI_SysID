from commands2 import Command
from commands2.button import JoystickButton
from commands2.sysid import SysIdRoutine

from wpilib import Joystick  # Import Joystick
from subsystems.drivetrain import RomiDrivetrain

class SysIdRoutineBot:
    """This class is where the bulk of the robot should be declared. Since Command-based is a
    'declarative' paradigm, very little robot logic should actually be handled in the Robot
    periodic methods (other than the scheduler calls). Instead, the structure of the robot
    (including subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive = RomiDrivetrain()

        # Use a Joystick at port 0
        self.joystick = Joystick(0)  # Updated to Joystick at port 0

        # Configure button bindings
        self.configureBindings()

    def configureBindings(self) -> None:
        """Use this method to define bindings between conditions and commands. These are useful for
        automating robot behaviors based on button and sensor input.

        Should be called during Robot.robotInit.
        """

        # Control the drive with split-stick arcade controls
        self.drive.setDefaultCommand(
            self.drive.arcadeDriveCommand(
                lambda: -self.joystick.getY(),  # Use Joystick Y-axis for forward/backward
                lambda: -self.joystick.getZ(),  # Use Joystick Z-axis for turning
            )
        )

        # Bind SysId tests to joystick buttons
        JoystickButton(self.joystick, 1).whileTrue(  # Button 1 (Trigger)
            self.drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        JoystickButton(self.joystick, 2).whileTrue(  # Button 2
            self.drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        JoystickButton(self.joystick, 3).whileTrue(  # Button 3
            self.drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        JoystickButton(self.joystick, 4).whileTrue(  # Button 4
            self.drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

    def getAutonomousCommand(self) -> Command:
        """Use this to define the command that runs during autonomous.

        Scheduled during Robot.autonomousInit.
        """

        # Do nothing
        return self.drive.run(lambda: None)