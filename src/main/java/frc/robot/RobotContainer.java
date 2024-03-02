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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HookConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Hook;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive;
  public final Arm arm;
  public final Hook hook;
  

  private static RobotContainer instance = null;
  private static final XboxController operatorController = new XboxController(Constants.OIConstants.operatorController);
  private static final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);


  public static final Trigger driver_A = new JoystickButton(driverController, 1),
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
  private static final POVButton driver_DPAD_UP = new POVButton(driverController, 0),
    driver_DPAD_RIGHT = new POVButton(driverController, 90), driver_DPAD_DOWN = new POVButton(driverController, 180),
    driver_DPAD_LEFT = new POVButton(driverController, 270);

  public static RobotContainer getInstance() {
      if(instance == null) instance = new RobotContainer();
      return instance;
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    arm = Arm.getInstance();
    hook = Hook.getInstance();
    m_robotDrive = new DriveSubsystem();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    // arm.setDefaultCommand(stowArm());
    // hook.setDefaultCommand(stowHook());
  }

  public Command goToPositionBezier(Pose2d target) {
    Rotation2d bezierControlPointHeading = Rotation2d.fromRadians(
      Math.atan2(
        target.getY() - m_robotDrive.getPose().getY(),
        target.getX() - m_robotDrive.getPose().getX()
      )
    );
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(
          m_robotDrive.getPose().getX(),
          m_robotDrive.getPose().getY(),  bezierControlPointHeading
        ),
        new Pose2d(target.getX(), target.getY(), bezierControlPointHeading.times(-1))
    );

    return AutoBuilder.followPath(
      new PathPlannerPath(
        bezierPoints, 
        Constants.AutoConstants.pathConstraints,
        new GoalEndState(0.0, target.getRotation())
      )
    );
  }

  /**
   * Create and return a command to follow a path that takes the robot to the target pose.
   *
   * @return The current alliance of the robot.
   */
  public Command goToPosition(Pose2d target, double goalEndVelocity, double rotationDelayDistance) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = Constants.AutoConstants.pathConstraints;
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
            target,
            constraints,
            goalEndVelocity, // Goal end velocity in meters/sec
            rotationDelayDistance // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    return pathfindingCommand;
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
    driver_DPAD_RIGHT.whileTrue(
      new RunCommand(() -> arm.setOpenLoop(.05), arm)
      ).onFalse(new InstantCommand(() -> arm.setOpenLoop(0)));

    driver_DPAD_LEFT.whileTrue(
      new RunCommand(() -> arm.setOpenLoop(-.05), arm)
      ).onFalse(new InstantCommand(() -> arm.setOpenLoop(0)));

    driver_DPAD_UP.whileTrue(
      new RunCommand(() -> hook.setOpenLoop(0.1), hook)
      ).onFalse(new InstantCommand(() -> hook.setOpenLoop(0)));

    driver_DPAD_DOWN.whileTrue(
      new RunCommand(() -> hook.setOpenLoop(-0.1), hook)
      ).onFalse(new InstantCommand(() -> hook.setOpenLoop(0)));

    driver_RB
      .whileTrue(
        new RunCommand(() -> hook.setHookState(States.HookPos.OPEN), hook)
      );
    driver_LB
      .whileTrue(
        new RunCommand(() -> hook.setHookState(States.HookPos.STOW), hook)
      );

    driver_X.whileTrue(
      new RunCommand(() -> goToPosition(
        new Pose2d(
          PathPlannerConstants.ampX, 
          PathPlannerConstants.ampY, 
          Rotation2d.fromDegrees(PathPlannerConstants.ampDegrees)
        ), 
        0, 1.
        ), m_robotDrive)
    );

    operator_Y
      .whileTrue(
        new ParallelCommandGroup(
          new RunCommand(() -> {
            arm.setArmState(States.ArmPos.SCORE);
            }, arm),
          new SequentialCommandGroup(
            new WaitCommand(HookConstants.delay),
            new RunCommand(() -> {
              hook.setHookState(States.HookPos.SCORE);
            }, hook
            )
          )
        )
      );

    operator_X
      .whileTrue(
       new RunCommand(() -> {
        arm.setArmState(States.ArmPos.STOW); 
        hook.setHookState(States.HookPos.STOW);
       }, arm, hook)
      );
    operator_B
      .whileTrue(
       new RunCommand(() -> {
        SmartDashboard.putNumber("B button Working", 1);
         double rotate = Robot.getAprilTagOffset();
         m_robotDrive.drive(0,0, rotate, true, false);

       }, m_robotDrive)
      );
    operator_RB
      .whileTrue(
       new RunCommand(() -> {
        arm.setArmState(States.ArmPos.CLIMB_UP); 
       }, arm)
      );
    operator_LB
      .whileTrue(
       new RunCommand(() -> {
        arm.setArmState(States.ArmPos.CLIMB_DOWN); 
       }, arm)
      );
  }

  public Command stowArm() {
    return new RunCommand(() -> arm.setArmState(States.ArmPos.STOW), arm);
  }

  public Command scoreArm(){
    return new RunCommand(() -> arm.setArmState(States.ArmPos.SCORE), arm);
  }

  public Command stowHook() {
    return new RunCommand(() -> hook.setHookState(States.HookPos.STOW), hook);
  }

  public Command score() {
    return new RunCommand(() -> {
        arm.setArmState(States.ArmPos.SCORE); 
        hook.setHookState(States.HookPos.SCORE);
       }, arm, hook);
  }

  public void bindOI(){

    // driver_X
    //     .onTrue(stowArm());
    
    driver_Y
        .onTrue(scoreArm());

 
  }

  /**
   * Returns the current alliance, with false indicating blue and true indicating red.
   * If there is no alliance, blue alliance is assumed. 
   *
   * @return The current alliance of the robot.
   */
  public boolean getAlliance() {
     var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getPathPlannerCommand1() {
  //   return new PathPlannerAuto("Simple Auto Part 1");
  // }

  // public Command getPathPlannerCommand2(){
  //   return new PathPlannerAuto("Simple Auto Part 2");
  // }


}
