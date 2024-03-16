// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Auto;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Eyes;
import frc.utils.CoordinateSpace;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;
  private static double aprilTagOffset = 0;
  public static synchronized double getAprilTagOffset(){
    return aprilTagOffset;
  }
  public static synchronized void setAprilTagOffset(double value){
    aprilTagOffset = value;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Thread visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    // PhotonVision testing
    // m_robotContainer = new RobotContainer();
    // CameraServer.startAutomaticCapture();
    // CvSink cvSink = CameraServer.getVideo();

    // // set up AprilTag detector
    // AprilTagDetector detector = new AprilTagDetector();
    // AprilTagDetector.Config config = new AprilTagDetector.Config();
    // // set config parameters, e.g. config.blah = 5;
    // detector.setConfig(config);
    // detector.addFamily("tag16h5");

    // // Set up Pose Estimator
    // AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(
    //   kDefaultPeriod, 
    //   kDefaultPeriod, 
    //   kDefaultPeriod, 
    //   kDefaultPeriod, 
    //   kDefaultPeriod
    //   );
    // AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Mat mat = new Mat();
    // Mat graymat = new Mat();

    // while (!Thread.interrupted()) {
    //   // grab image from camera
    //   long time = cvSink.grabFrame(mat);
    //   if (time == 0) {
    //     continue;  // error getting image
    //   }

    //   // convert image to grayscale
    //   Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_BGR2GRAY);
      
    //   // run detection
    //   for (AprilTagDetection detection : detector.detect(graymat)) {
    //     // filter by property

    //     // run pose estimator
    //     Transform3d pose = PoseEstimator.estimate(detection);
    //   }
    // }
    m_robotContainer = RobotContainer.getInstance();
  }

  void apriltagVisionThreadProc() {
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag16h5", 0);
  
    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();

    CoordinateSpace coordinateSpace = new CoordinateSpace(VisionConstants.cameraWidth, VisionConstants.cameraHeight);
    // Set the resolution
    camera.setResolution((int)coordinateSpace.width, (int)coordinateSpace.height);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("detect", VisionConstants.cameraWidth, VisionConstants.cameraHeight);

    // Mats are very memory expensive. Lets reuse this Mat.
    Mat mat = new Mat();
    Mat grayMat = new Mat();
    ArrayList<Integer> tags = new ArrayList<>();

    //
    Scalar outlineColor = new Scalar(0, 255, 0);
    Scalar xColor = new Scalar(0, 0, 255);

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);
      var reticalSize = 20;
      AprilTagDetection[] detections = detector.detect(grayMat);
      tags.clear();
      AprilTagDetection selectedTag = null;
      for (AprilTagDetection detection : detections) {

        // From mentors (John & Lauren): 
        //    1. How do you adjust the robots position using these variables?
        //    2. How do you know when you're centered against the april tag?
        //    3. What happens if the camera isn't mounted perfectly center? (see 4)
        //    4. What variables affect alignment? (offset from center, mount angle...etc) -- write all of these 
        //       as constant variables; with GOOD names (no 'i' or 'j' or 'mhm'. For example, use 'mountHeightInMeters' instead.)
        //    5. How often does detection need to run? Can it only run while a button is pressed?
        //    6. What is _smallest_ resolution you can use to detect april tags within your specification? (also, choose a max distance
        //       for this to work). Smaller reoslutions evaluate exponentially faster.
        //    7. How can april tag detection be part of a pre-comp system check? Who is responsible for writing/checking that? For this
        //       step, please make sure there is more than one person who can do either.
        //    8. Please agree on a branching strategy. Forks are unnecessary, and you should have a single branch that is only for
        //       competition. 
        //        a. For competition, we recommend 'competition/<year>', so for this year it is 'competition/2024'. All active work MUST
        //           NOT happen on this branch -- only merge in working code to this branch
        //        b. Make personal branches for experiments. We recommend '<Name>/<experiment>'. This should share with others what you
        //           are trying to do. For example, 'john/photon-vision-with-webcam'.
        //        c. When you branch is ready, push it to the repository and merge it into the competition branch using 
        //           'git checkout <competition branch> && git merge <your branch>'. 
        //            For example, 'git checkout competition/2024 && git merge john/photon-vision-with-webcam'
        //    8. Think about how to pick which detection you want if more than one is on screen?

        // The following variables are the most useful values
        var tagID = detection.getId();
        var centerX = detection.getCenterX();
        var centerY = detection.getCenterY();

        tags.add(tagID);

        /// Draw debug rectangle

        for (var corner = 0; corner <= 3; corner++) {
          var nextCorner = (corner + 1) % 4;
          var pt1 = new Point(detection.getCornerX(corner), detection.getCornerY(corner));
          var pt2 = new Point(detection.getCornerX(nextCorner), detection.getCornerY(nextCorner));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // Draw debug retical
        Imgproc.line(mat, new Point(centerX - reticalSize, centerY), new Point(centerX + reticalSize, centerY), xColor, 2);
        Imgproc.line(mat, new Point(centerX, centerY - reticalSize), new Point(centerX, centerY + reticalSize), xColor, 2);
        Imgproc.putText(mat, Integer.toString(tagID), new Point (centerX + reticalSize, centerY), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
        selectedTag = detection;
      }

      double commandedRotation;
      if (selectedTag != null) {
        final double proportionalError = Eyes.widthOffset(selectedTag, coordinateSpace) / coordinateSpace.width;
        commandedRotation = Eyes.rotationRate(proportionalError);
      } else {
        commandedRotation = 0;
      }
      setAprilTagOffset(commandedRotation);
      SmartDashboard.putNumber("Vision Commanded Rotation", commandedRotation);
      SmartDashboard.putString("tag", tags.toString());
      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    detector.close();
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.anniversaryDance();
    // m_autonomousCommand = Auto.anniversaryDance();
    // m_autonomousCommand = Auto.ScoreAutoOneNoteAmp();
    // m_autonomousCommand = Auto.driveTime(1,1 ,1 ,1 ); // replace with actual values
    // m_autonomousCommand = null;
    // m_autonomousCommand = Auto.driveAutoCommand();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
