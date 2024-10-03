// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.util.Controller;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.FRONT_CAMERA;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(FRONT_CAMERA);
    public static final Swerve SWERVE = new Swerve();

    public static final BuiltInAccelerometer ROBORIO_ACCELEROMETER = new BuiltInAccelerometer();


    private final Controller driveController = new Controller(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser("Teared down Crown"));

        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        SWERVE.setDefaultCommand(
                SWERVE.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,

                        () -> -driveController.getRawAxis(Controller.Axis.RIGHT_X),
                        () -> driveController.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        driveController.getButton(Controller.Inputs.START).whileTrue(SWERVE.resetGyro());
        driveController.getButton(Controller.Inputs.BACK).whileTrue(SWERVE.lockSwerve());

    }


    private void setupCharacterization(GenericSubsystem subsystem) {
        driveController.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        driveController.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /*
    - Swerve - VERY COOL!

-Intake - an arm that goes down
Horizontal Conveyor - controlled by the same intake motor.
Vertical conveyor - controllde by diff motor

3 beam breaks.

color sensor to know which box you have

2 falcons controlling a turret

adjustable hood - pretty much an arm - control the angle.
Shooter - one motor going whoop

climbing - exists I guess.
    */
}
