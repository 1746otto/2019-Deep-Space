/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.Xbox;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.loops.Looper;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchScorer;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;

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
  private Superstructure s;
  private SubsystemManager subsystems;

  private Looper enabledLooper = new Looper();
  private Looper disabledLooper = new Looper();

  private DriverStation ds = DriverStation.getInstance();

  private Xbox driver;
  private Xbox coDriver;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    s = Superstructure.getInstance();
    cargoIntake = CargoIntake.getInstance();
    hatchScorer = HatchScorer.getInstance();
    driveTrain = Drivetrain.getInstance();
    lift = Lift.getInstance();
    subsystems = new SubsystemManager(Arrays.asList(s, cargoIntake, hatchScorer, driveTrain, lift));

    driver = new Xbox(0);
    coDriver = new Xbox(1);
    driver.setDeadband(0.1);
    coDriver.setDeadband(0.1);

    subsystems.registerEnabledLoops(enabledLooper);
    subsystems.registerDisabledLooper(disabledLooper);
  }

  public void allPeriodic() {
    subsystems.outputToSmartDashboard();
    enabledLooper.outputToSmartDashboard();
    SmartDashboard.putBoolean("Enabled", ds.isEnabled());
  }

  public void teleopConfig() {
    lift.setCurrentLimit(Constants.kLiftCurrentLimit + 5);
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
    double driveYInput = driver.getY(Hand.kLeft);

    driveTrain.setOpenLoop(new Translation2d(driveXInput, driveYInput));

    double liftInput = driver.getY(Hand.kRight);

    if (Math.abs(liftInput) != 0) {
      lift.setOpenLoop(liftInput);
    } else if (lift.isOpenLoop()) {
      lift.lockHeight();
    }

    if (driver.aButton.wasActivated()) {
      lift.setTargetHeight(0.0);
      s.resetLiftRequest();
    } else if (driver.xButton.wasActivated()) {
      if (cargoIntake.getCargoSensor()) {
        lift.setTargetHeight(Constants.kHighLevelCargoHeight);
        lift.lockHeight();
      } else {
        lift.setTargetHeight(Constants.kHighLevelHatchRocketHeight);
        lift.lockHeight();
      }
    } else if (driver.yButton.wasActivated()) {
      if (cargoIntake.getCargoSensor()) {
        lift.setTargetHeight(Constants.kMidLevelCargoHeight);
        lift.lockHeight();
      } else {
        lift.setTargetHeight(Constants.kMidLevelHatchRocketHeight);
        lift.lockHeight();
      }
    } else if (driver.POV0.wasActivated()) {
      lift.setTargetHeight(Constants.kCargoShipHeight);
      lift.lockHeight();
    } else if (driver.POV180.wasActivated()) {
      lift.setTargetHeight(Constants.kLowLevelCargoHeight);
      lift.lockHeight();
    }

    if (driver.backButton.wasActivated()) {
      s.toggleIntakeState();
    }

    if (driver.startButton.wasActivated()) {
      cargoIntake.toggleBumperState();
    }

    if (driver.leftBumper.wasActivated()) {
      hatchScorer.toggleIntakeState();
    }

    if (driver.rightBumper.wasActivated()) {
      hatchScorer.toggleStowState();
    }

    cargoIntake.setIntakeSpeed(driver.getTriggerAxis(Hand.kRight) 
        - driver.getTriggerAxis(Hand.kLeft));

  }
}
