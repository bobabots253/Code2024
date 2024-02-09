// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Hook;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  //The rest of buttons
   private final Arm arm = Arm.getInstance();
  private final Hook hook = Hook.getInstance();


  private static RobotContainer instance = null;
  private static final XboxController driverController = new XboxController(Constants.OIConstants.driverController);
  private static final XboxController operatorController = new XboxController(Constants.OIConstants.operatorController);
  private static final Trigger driver_A = new JoystickButton(driverController, 1),
      driver_B = new JoystickButton(driverController, 2), driver_X = new JoystickButton(driverController, 3),
      driver_Y = new JoystickButton(driverController, 4), driver_LB = new JoystickButton(driverController, 5),
      driver_RB = new JoystickButton(driverController, 6), driver_VIEW = new JoystickButton(driverController, 7),
      driver_MENU = new JoystickButton(driverController, 8);
  private static final Trigger operator_A = new JoystickButton(operatorController, 1),
      operator_B = new JoystickButton(operatorController, 2), operator_X = new JoystickButton(operatorController, 3),
      operator_Y = new JoystickButton(operatorController, 4), operator_LB = new JoystickButton(operatorController, 5),
      operator_RB = new JoystickButton(operatorController, 6), operator_VIEW = new JoystickButton(operatorController, 7),
      operator_MENU = new JoystickButton(operatorController, 8);
  private static final POVButton operator_DPAD_UP = new POVButton(operatorController, 0),
  operator_DPAD_RIGHT = new POVButton(operatorController, 90), operator_DPAD_DOWN = new POVButton(operatorController, 180),
  operator_DPAD_LEFT = new POVButton(operatorController, 270);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    bindOI();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    arm.setDefaultCommand(stowArm());
    hook.setDefaultCommand(stowArm());
  }

  public Command stowArm() {
    return new RunCommand(() -> arm.setArmState(States.ArmPos.STOW), arm);
  }

  public Command scoreArm(){
    return new RunCommand(() -> arm.setArmState(States.ArmPos.SCORE), arm);
  }

  public Command stowHook(){
    return new RunCommand(() -> hook.setHookState(States.HookPos.STOW), hook);
  }

  public Command scoreHook(){
    return new RunCommand(() -> hook.setHookState(States.HookPos.OPEN), hook);
  }

//   public Command stowHook() {
//     return new RunCommand(() -> hook.setHookState(States.HookPos.STOW), hook);
//   }

  public void bindOI(){

    driver_X
        .onTrue(stowArm());
    
    driver_Y
        .onTrue(scoreArm());

    driver_A
        .onTrue(stowHook());
    
    driver_B
        .onTrue(scoreHook());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("TestAuto");
  }
}
