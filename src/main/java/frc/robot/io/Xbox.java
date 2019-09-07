package frc.robot.io;

import frc.robot.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Xbox extends XboxController {
  private static final double PRESS_THRESHOLD = 0.4;
  private double DEAD_BAND = 0.1;
  private boolean rumbling = false;
  public ButtonCheck aButton, bButton, xButton, yButton, startButton, backButton, leftBumper,
      rightBumper, leftCenterClick, rightCenterClick, leftTrigger, rightTrigger, POV0, POV90,
      POV180, POV270;
  public static final int A_BUTTON = 1;
  public static final int B_BUTTON = 2;
  public static final int X_BUTTON = 3;
  public static final int Y_BUTTON = 4;
  public static final int LEFT_BUMPER = 5;
  public static final int RIGHT_BUMPER = 6;
  public static final int BACK_BUTTON = 7;
  public static final int START_BUTTON = 8;
  public static final int LEFT_CENTER_CLICK = 9;
  public static final int RIGHT_CENTER_CLICK = 10;
  public static final int LEFT_TRIGGER = -2;
  public static final int RIGHT_TRIGGER = -3;
  public static final int POV_0 = -4;
  public static final int POV_90 = -5;
  public static final int POV_180 = -6;
  public static final int POV_270 = -7;

  public void setDeadband(double db) {
    DEAD_BAND = db;
  }

  public Xbox(int port) {
    super(port);
    aButton = new ButtonCheck(A_BUTTON);
    bButton = new ButtonCheck(B_BUTTON);
    xButton = new ButtonCheck(X_BUTTON);
    yButton = new ButtonCheck(Y_BUTTON);
    startButton = new ButtonCheck(START_BUTTON);
    backButton = new ButtonCheck(BACK_BUTTON);
    leftBumper = new ButtonCheck(LEFT_BUMPER);
    rightBumper = new ButtonCheck(RIGHT_BUMPER);
    leftCenterClick = new ButtonCheck(LEFT_CENTER_CLICK);
    rightCenterClick = new ButtonCheck(RIGHT_CENTER_CLICK);
    leftTrigger = new ButtonCheck(LEFT_TRIGGER);
    rightTrigger = new ButtonCheck(RIGHT_TRIGGER);
    POV0 = new ButtonCheck(POV_0);
    POV90 = new ButtonCheck(POV_90);
    POV180 = new ButtonCheck(POV_180);
    POV270 = new ButtonCheck(POV_270);
  }

  @Override
  public double getX(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return Util.deadBand(getRawAxis(0), DEAD_BAND);
    } else {
      return Util.deadBand(getRawAxis(4), DEAD_BAND);
    }
  }

  @Override
  public double getY(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return Util.deadBand(getRawAxis(1), DEAD_BAND);
    } else {
      return Util.deadBand(getRawAxis(5), DEAD_BAND);
    }
  }

  @Override
  public double getTriggerAxis(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return Util.deadBand(getRawAxis(2), PRESS_THRESHOLD);
    } else {
      return Util.deadBand(getRawAxis(3), PRESS_THRESHOLD);
    }
  }

  public void rumble(double rumblesPerSecond, double numberOfSeconds) {
    if (!rumbling) {
      RumbleThread r = new RumbleThread(rumblesPerSecond, numberOfSeconds);
      r.start();
    }
  }

  public boolean isRumbling() {
    return rumbling;
  }

  public class RumbleThread extends Thread {
    public double rumblesPerSec = 1;
    public long interval = 500;
    public double seconds = 1;
    public double startTime = 0;

    public RumbleThread(double rumblesPerSecond, double numberOfSeconds) {
      rumblesPerSec = rumblesPerSecond;
      seconds = numberOfSeconds;
      interval = (long) (1 / (rumblesPerSec * 2) * 1000);
    }

    public void run() {
      rumbling = true;
      startTime = Timer.getFPGATimestamp();
      try {
        while ((Timer.getFPGATimestamp() - startTime) < seconds) {
          setRumble(RumbleType.kLeftRumble, 1);
          setRumble(RumbleType.kRightRumble, 1);
          sleep(interval);
          setRumble(RumbleType.kLeftRumble, 0);
          setRumble(RumbleType.kRightRumble, 0);
          sleep(interval);
        }
      } catch (InterruptedException e) {
        rumbling = false;
        e.printStackTrace();
      }
      rumbling = false;
    }
  }

  public class ButtonCheck {
    boolean buttonCheck = false;
    boolean buttonActive = false;
    boolean activationReported = false;
    boolean longPressed = false;
    boolean longPressActivated = false;
    boolean hasBeenPressed = false;
    boolean longReleased = false;
    private double buttonStartTime = 0;
    private double longPressDuration = 0.25;
    private int buttonNumber;

    public void setLongPressDuration(double seconds) {
      longPressDuration = seconds;
    }

    public ButtonCheck(int id) {
      buttonNumber = id;
    }

    public void update() {
      if (buttonNumber > 0) {
        buttonCheck = getRawButton(buttonNumber);
      } else {
        switch (buttonNumber) {
          case LEFT_TRIGGER:
            buttonCheck = getTriggerAxis(Hand.kLeft) > 0;
            break;
          case RIGHT_TRIGGER:
            buttonCheck = getTriggerAxis(Hand.kRight) > 0;
            break;
          case POV_0:
            buttonCheck = (getPOV() == 0);
            break;
          case POV_90:
            buttonCheck = (getPOV() == 90);
            break;
          case POV_180:
            buttonCheck = (getPOV() == 180);
            break;
          case POV_270:
            buttonCheck = (getPOV() == 270);
            break;
          default:
            buttonCheck = false;
            break;
        }
      }
      if (buttonCheck) {
        if (buttonActive) {
          if (((Timer.getFPGATimestamp() - buttonStartTime) > longPressDuration)
              && !longPressActivated) {
            longPressActivated = true;
            longPressed = true;
            longReleased = false;
          }
        } else {
          buttonActive = true;
          activationReported = false;
          buttonStartTime = Timer.getFPGATimestamp();
        }
      } else {
        if (buttonActive) {
          buttonActive = false;
          activationReported = true;
          if (longPressActivated) {
            hasBeenPressed = false;
            longPressActivated = false;
            longPressed = false;
            longReleased = true;
          } else {
            hasBeenPressed = true;
          }
        }
      }
    }

    /**
     * Returns true once the button is pressed regardless of activation time. Only returns true once
     * per button press, and is reset after release.
     */
    public boolean wasActivated() {
      if (buttonActive && !activationReported) {
        activationReported = true;
        return true;
      }
      return false;
    }

    /**
     * Returns true if the button was held down for 0.25s or less and released. Only returns true
     * once per button press.
     */
    public boolean shortReleased() {
      if (hasBeenPressed) {
        hasBeenPressed = false;
        return true;
      }
      return false;
    }

    /**
     * If button has been held down for more than 0.25s and is still being pressed. Returns false
     * once the button is depressed.
     */
    public boolean longPressed() {
      if (longPressed) {
        longPressed = false;
        return true;
      }
      return false;
    }

    /**
     * Returns true once button is let go after 0.25s, resets after every release.
     */
    public boolean longReleased() {
      if (longReleased) {
        longReleased = false;
        return true;
      }
      return false;
    }

    /**
     * Returns true once button is released.
     */
    public boolean wasReleased() {
      return shortReleased() || longReleased();
    }

    /**
     * Returns true if a button is currently pressed.
     */
    public boolean isBeingPressed() {
      return buttonActive;
    }
  }

  public void update() {
    aButton.update();
    bButton.update();
    xButton.update();
    yButton.update();
    startButton.update();
    backButton.update();
    leftBumper.update();
    rightBumper.update();
    leftCenterClick.update();
    rightCenterClick.update();
    leftTrigger.update();
    rightTrigger.update();
    POV0.update();
    POV90.update();
    POV180.update();
    POV270.update();
  }
}
