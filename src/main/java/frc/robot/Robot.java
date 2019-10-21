/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.Date;
import java.util.UUID;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.Xbox;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.loops.Looper;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchScorer;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.LimelightProcessor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Drivetrain driveTrain;
  private Lift lift;
  private HatchScorer hatchScorer;
  private CargoIntake cargoIntake;
  private Climber climber;
  private LimelightProcessor limelightProcessor;

  private Superstructure s;
  private SubsystemManager subsystems;

  private Looper enabledLooper = new Looper();
  private Looper disabledLooper = new Looper();

  private DriverStation ds = DriverStation.getInstance();
  private BadLog log;

  private Xbox driver;
  private Xbox coDriver;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    log = BadLog.init("/home/lvuser/badlog/" + 
        (Constants.kIsUsingCompBot ? ds.getMatchNumber() : new Date().toLocaleString()) + ".bag");
    BadLog.createValue("Start Time", new Date().toString());
    BadLog.createValue("Match Number", "" +  ds.getMatchNumber());

    s = Superstructure.getInstance();
    cargoIntake = CargoIntake.getInstance();
    hatchScorer = HatchScorer.getInstance();
    driveTrain = Drivetrain.getInstance();
    lift = Lift.getInstance();
    climber = Climber.getInstance();
    limelightProcessor = LimelightProcessor.getInstance();
    
    subsystems = new SubsystemManager(Arrays.asList(s, cargoIntake, hatchScorer, 
        driveTrain, lift, climber));

    driver = new Xbox(0);
    coDriver = new Xbox(1);
    driver.setDeadband(0.2);
    coDriver.setDeadband(0.1);

    enabledLooper.register(limelightProcessor);

    subsystems.registerEnabledLoops(enabledLooper);
    subsystems.registerDisabledLooper(disabledLooper);

    log.finishInitialization();
  }

  public void allPeriodic() {
    subsystems.outputTelemetery();
    enabledLooper.outputTelemetry();
    if (ds.isEnabled()) {
      log.updateTopics();
      log.log();
    }    
    SmartDashboard.putBoolean("Enabled", ds.isEnabled());
  }

  public void teleopConfig() {
    lift.configForTeleopSpeed();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    try {
      disabledLooper.stop();
      enabledLooper.start();
      teleopConfig();
    } catch (Throwable t) {
      throw t;
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    try {
      driver.update();
      allPeriodic();
      twoControllerMode();
    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    try {
      disabledLooper.stop();
      enabledLooper.start();
      teleopConfig();
    } catch (Throwable t) {
      throw t;
    }
  }
  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    try {
      driver.update();
      coDriver.update();
      twoControllerMode();
      allPeriodic();
    } catch (Throwable t) {
      throw t;
    }
  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void disabledInit() {
    try {
      enabledLooper.stop();
      disabledLooper.start();
      subsystems.stop();
    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try {
      allPeriodic();
    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void testInit() {
    s.toggleCompressor();  
  }

  @Override
  public void testPeriodic() {
    if (s.getCompressorState()) {
      s.toggleCompressor();
    }
  }

  public void twoControllerMode() {
    double driveXInput = driver.getX(Hand.kLeft);
    double driveYInput = -driver.getY(Hand.kLeft);

    driveTrain.setOpenLoop(new Translation2d(driveXInput, driveYInput));

    double liftInput = -driver.getY(Hand.kRight);

    if (Math.abs(liftInput) != 0) {
      lift.setOpenLoop(liftInput);
    } else if (lift.isOpenLoop()) {
      lift.lockHeight();
    }

    double climbInput = -coDriver.getY(Hand.kLeft);

    climber.setOpenLoop(climbInput);

    if (coDriver.bButton.wasActivated()) {
      climber.toggleVaccum();
    }

    if (driver.aButton.wasActivated()) {
      s.resetLiftState();
    } else if (driver.xButton.wasActivated()) {
      if (cargoIntake.getCargoSensor()) {
        lift.setTargetHeight(Constants.kLowLevelCargoHeight);
      } else {
        lift.setTargetHeight(Constants.kMidLevelHatchRocketHeight);
      }
    } else if (driver.yButton.wasActivated()) {
      if (cargoIntake.getCargoSensor()) {
        lift.setTargetHeight(Constants.kMidLevelCargoHeight);
      } else {
        lift.setTargetHeight(Constants.kHighLevelHatchRocketHeight);
      }
    } else if (driver.POV0.wasActivated()) {
      lift.setTargetHeight(Constants.kCargoShipHeight);
    } else if (driver.POV180.wasActivated()) {
      lift.setTargetHeight(Constants.kHighLevelCargoHeight);
    }

    if (driver.bButton.isBeingPressed()) {
      driveTrain.setVisionSteer(new Translation2d(limelightProcessor.generateSteer(), driveYInput));
    }

    if (driver.backButton.wasActivated()) {
      s.toggleIntakeState();
    }

    if (driver.startButton.wasActivated()) {
      cargoIntake.toggleBumperState();
    }

    if (driver.rightBumper.wasActivated()) {
      hatchScorer.toggleIntakeState();
    }

    if (driver.leftBumper.wasActivated()) {
      hatchScorer.toggleStowState();
    }


    limelightProcessor.toggleCamMode(driver.bButton.isBeingPressed());

    cargoIntake.setIntakeSpeed(driver.getTriggerAxis(Hand.kLeft) 
        - driver.getTriggerAxis(Hand.kRight));
  }
}
