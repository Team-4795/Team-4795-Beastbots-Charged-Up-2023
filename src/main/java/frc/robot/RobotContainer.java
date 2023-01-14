// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;

import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivebase m_drivebase = new Drivebase();
  private final XboxController driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivebase.setDefaultCommand(new ArcadeDrive(
      m_drivebase,
      () -> driverController.getLeftY(),
      () -> driverController.getRightX()
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
try {
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/Path.wpilib.json");
  Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      m_drivebase::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        DrivebaseConstants.ksVolts,
        DrivebaseConstants.kvVoltSecondsPerMeter,
        DrivebaseConstants.kaVoltSecondsSquaredPerMeter
      ),
      DrivebaseConstants.kDriveKinematics,
      m_drivebase::getWheelSpeeds,
      new PIDController(DrivebaseConstants.kPDriveVel, 0, 0),
      new PIDController(DrivebaseConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drivebase::tankDriveVolts,
      m_drivebase
    );

    // Reset odometry to the starting pose of the trajectory.
    m_drivebase.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivebase.tankDriveVolts(0, 0));

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + "paths/output/Path.wpilib.json", ex.getStackTrace());
    }

    return null;
  }
}
