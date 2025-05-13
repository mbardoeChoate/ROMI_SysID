# ROMI SysID
## Purpose

While it is true that the two motors on the ROMI are from the same company and produced in the same way to the same 
specifications it is possible (likely) that the way that the two motors and the physical nature of the robot cause the 
wheels to spin at slightly different speeds when given the same voltage by the robot. This means that the robot will likely 
not move exactly straight when given the same voltage to both motors. 

To deal with issues like this, which could ruin autonomous routines, or really any request of the robot. Is to *characterize* 
the motors and how they respond when given requests of certain voltages. WPILib has a separate program called *SysId* which 
allows us to take log files that are properly formatted and that program does statistics on the output of the log, and determines
a mathematical equation that can be used to estimate what voltage is needed to make the robot act in a specific way. In
our case the model is:

Voltage= K_s * sign(velocity) +K_v * velocity+K_a * acceleration

In this model $Ks$ stands for how much voltage it takes to get the motor to move at all. $Kv$ is how 
much more voltage we would need to increase the velocity by 1, and $Ka$ is the amount of voltage we need to increase the 
acceleration by 1. The ROMI is quite small, and we will simplify the model to assume that $Ka$ is zero.

#### Beyond the ROMI

Characterization is something that is very helpful in programming your robot to respond in the way that you hope it would. 
Models like the the one above are often referred to as *Open Loop Control*. Open loop control indicates that we are using 
information about the systems dynamics to create the response that we would like. Soon we will learn about *Closed Loop Control*. 
All of this is important in many systems that we want to work precisely, quickly, and predictably within our robot. 



## Challenges

The purpose of this project is to create a project that allows the ROMI to demonstrate how SysId works in FRC. 
There are several challenges to this. The first is that the logging system on the ROMI seems to behave very differently
from a typical FRC robot. I have had difficulty getting the logging to work in the expected way when running the 
SysIdRoutines. You can see this in the drivetrain code where there is a `log` method that should save the required
information to the log, but instead we are using several other methods. 

## Run the ROMI

To run the ROMI use the following code in the terminal

```
python3 -m robotpy sim --ws-client
```

Then when the simulator window opens move the Keyboard(0) to Joystick(0). I have had difficulty with the axis being 
1 and 2 instead of 0 and 1. But this can be reset in the dialog. 

## Run the tests

By hitting buttons `z`, `x`, `c`, and `v` the four basic tests should run, and the information from the encoders are 
saved to the log file. This information is not visible in network tables because the information is only saved in the log.

After you have done the tests. The log file should be in a folder named logs in the projects home directory. You can 
upload this file to SysId application. There will be a tab in the log for Drive and the information there should be what 
is needed to make the SysId work. Read the instruction on the wpilib website to find `Ks`, `Kv`, and `Ka`. The `Ka` did not
come out as significant when I did my ROMI (bad floor, and somewhat expected for such a low mass robot).

## Implementing the values

Once you have the values for both the right and left motors you can create use those values to send voltage requests to the 
motors based on the number of rotations that you want the wheels to spin. WPILib has many classes that with this. 

* `SimpleFeedForwardMeters` - Is instantiated with values from SysId and can be used to determine the right voltage for a speed.
* `DifferentialDriveKinematics` - Is instantiated with the track width, then you can give it the forward and rotation speeds and find out
what speeds the wheels need to turn to attain that motion.
* `ChassisSpeeds` - is a wrapper that contains the information about how fast we would like the robot to go.

By putting these classes together we can recreate our arcade drive so that each motor is set to a speed that will hopefully
align with the desired motion, instead of assuming that both motors work exactly the same.

