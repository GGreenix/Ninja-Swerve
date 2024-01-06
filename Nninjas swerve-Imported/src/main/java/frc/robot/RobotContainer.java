// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CheckCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  Swerve _swerve;
  PS4Controller _controller;
  JoystickButton _oneBtn = new JoystickButton(_controller, 0);
  JoystickButton _twoBtn = new JoystickButton(_controller, 1);
  JoystickButton _threeBtn = new JoystickButton(_controller, 2);
  JoystickButton _fourBtn = new JoystickButton(_controller, 3);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _swerve = new Swerve();
    _controller = new PS4Controller(0);
    CommandScheduler.getInstance().registerSubsystem(_swerve);
    _swerve.setDefaultCommand(null
      // new DefaultDriveCommand(
      // this._swerve,
      // () -> 0,
      // () -> 0,
      // () -> 0,
      // () -> false
      // )
      );
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    _oneBtn.whenPressed(
      new CheckCommand(this._swerve,0)
      // _swerve.getAutonomousCommand(new PathPoint(new Translation2d(0,0),new Rotation2d(0)))
      );
      _twoBtn.whenPressed(
      new CheckCommand(this._swerve,1)
      // _swerve.getAutonomousCommand(new PathPoint(new Translation2d(0,0),new Rotation2d(0)))
      );
      _threeBtn.whenPressed(
      new CheckCommand(this._swerve,2)
      // _swerve.getAutonomousCommand(new PathPoint(new Translation2d(0,0),new Rotation2d(0)))
      );
      _fourBtn.whenPressed(
      new CheckCommand(this._swerve,3)
      // _swerve.getAutonomousCommand(new PathPoint(new Translation2d(0,0),new Rotation2d(0)))
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
