/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Written by Æclipse and PizzaLovers007 and Olaf_Underhaven                  */
/* (c) 2014                                                                   */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;

public class Cyborgs2014 extends IterativeRobot {

    private static final double Kp = 0.1;
    private static final double Ki = 0;
    private static final double SLOW_SPEED = 0.5;
    private static final double FAST_SPEED = 0.95;
    private static final double SHOOT_DISTANCE = 48;

    private final DriverStationLCD lcd = DriverStationLCD.getInstance();
    private boolean hasChangedSpeed;
    private boolean isKickingHigh, isKickingLow;
    private boolean isSwitchingControls;
    private boolean trippDriving;
    private boolean isHotGoal;
    private double sens;
    private double autoTimeStart;
    private double kickerTimeStart;
//    private Encoder encFrontLeft, encFrontRight, encBackLeft, encBackRight;
    private Joystick controller;
//    private PIDController pidFrontLeft, pidFrontRight, pidBackLeft, pidBackRight;
    private RobotDrive drive;
    private SpeedController motorFrontLeftOne, motorFrontRightTwo, motorBackLeftThree, motorBackRightFour;
    private SpeedController shoulderLeftSix, shoulderRightFive, intakeLeftEight, intakeRightNine;
    private SpeedController kickerLeftTen, kickerRightSeven;
    private Watchdog malachi;
    private AxisCamera camera;
    private DigitalInput limit;
    private DigitalInput ultrasonic;

    public void robotInit() {
        joystickInit();
        SpeedControllerInit();
        robotDriveInit();
        sensorInit();
//        encoderInit();
//        ultra = new Ultrasonic(9, 10, Ultrasonic.Unit.kInches);
//        ultra.setEnabled(true);
        malachi = getWatchdog();
        malachi.setExpiration(3);
        malachi.feed();
        hasChangedSpeed = false;
        sens = SLOW_SPEED;
        autoTimeStart = 0;
        resetLCD();
        trippDriving = true;
        isSwitchingControls = false;
        isHotGoal = false;
        isKickingHigh = false;
        isKickingLow = false;
        //camera = AxisCamera.getInstance("10.33.35.11");
    }

//    public void encoderInit() {
//        encFrontLeft = new Encoder(1, 2, true, CounterBase.EncodingType.k4X);
//        encFrontRight = new Encoder(3, 4, false, CounterBase.EncodingType.k4X);
//        encBackLeft = new Encoder(5, 6, true, CounterBase.EncodingType.k4X);
//        encBackRight = new Encoder(7, 8, false, CounterBase.EncodingType.k4X);
//        encFrontLeft.setDistancePerPulse(0.025132);
//        encFrontRight.setDistancePerPulse(0.025132);
//        encBackLeft.setDistancePerPulse(0.025132);
//        encBackRight.setDistancePerPulse(0.025132);
//        encFrontLeft.start();
//        encFrontRight.start();
//        encBackLeft.start();
//        encBackRight.start();
//        pidFrontLeft = new PIDController(1, 0, 0, encFrontLeft, motorFrontLeft);
//        pidFrontRight = new PIDController(1, 0, 0, encFrontRight, motorFrontRight);
//        pidBackLeft = new PIDController(1, 0, 0, encBackLeft, motorBackLeft);
//        pidBackRight = new PIDController(1, 0, 0, encBackRight, motorBackRight);
//    }
    public void sensorInit() {
        limit = new DigitalInput(14);
        ultrasonic = new DigitalInput(6);
    }

    public void joystickInit() {
        controller = new Joystick(1);
    }

    public void robotDriveInit() {
        drive = new RobotDrive(motorFrontLeftOne, motorFrontRightTwo, motorBackLeftThree, motorBackRightFour);
    }

    public void SpeedControllerInit() {
        motorFrontLeftOne = new Talon(1);
        motorFrontRightTwo = new Talon(2);
        motorBackLeftThree = new Talon(3);
        motorBackRightFour = new Talon(4);
        shoulderLeftSix = new Talon(5);
        shoulderRightFive = new Talon(10);
        kickerRightSeven = new Talon(7); 
        kickerLeftTen = new Talon(8);
        intakeLeftEight = new Talon(9);
        intakeRightNine = new Talon(6);
        
        
        
    }

    public void disabledContinuous() {
    }

    public void disabledInit() {
        resetLCD();
    }

    public void disabledPeriodic() {
    }

    public void autonomousInit() {
        malachi.setEnabled(true);
        malachi.feed();
        autoTimeStart = Timer.getFPGATimestamp();
        resetLCD();
    }

    public void autonomousPeriodic() {
        malachi.feed();
        // ultra.ping();
        double autoTimeCurr = Timer.getFPGATimestamp() - autoTimeStart;
//        if (!limit.get()) {
//            shoulderLeft.set(0);
//            shoulderRight.set(0);
//        } else {
//            shoulderLeft.set(0.55);
//            shoulderRight.set(-0.55);
//        }
        if (autoTimeCurr <= 1) {
            shoulderLeftSix.set(-0.9);
            shoulderRightFive.set(0.9);
        } else {
            shoulderLeftSix.set(0);
            shoulderRightFive.set(0);
        }
        if (autoTimeCurr <= 0.5) {
            lcd.println(DriverStationLCD.Line.kUser1, 1, "== 0          ");
            motorFrontLeftOne.set(0.2);
            motorFrontRightTwo.set(-0.2);
            motorBackLeftThree.set(-0.2);
            motorBackRightFour.set(0.2);
        } else if (autoTimeCurr <= 2.7) {
            lcd.println(DriverStationLCD.Line.kUser1, 1, "<= 2");
            motorFrontLeftOne.set(0.5);
            motorFrontRightTwo.set(-0.5);
            motorBackLeftThree.set(-0.5);
            motorBackRightFour.set(0.5);

        } else if (autoTimeCurr <= 3.0) {
            lcd.println(DriverStationLCD.Line.kUser1, 1, "<= 2.8");
            motorFrontLeftOne.set(0);
            motorFrontRightTwo.set(-0);
            motorBackLeftThree.set(0);
            motorBackRightFour.set(-0);
            kickerLeftTen.set(1);
            kickerRightSeven.set(1);
// Dat math though
// Also add the camera code here
// If high goal lights are on, shoot
// Otherwise wait for a few seconds until it switches
        } else if (autoTimeCurr <= 3.25) {
            lcd.println(DriverStationLCD.Line.kUser1, 1, "<= 3.05");
            kickerLeftTen.set(-0.1);
            kickerRightSeven.set(-0.1);

        } else {
            lcd.println(DriverStationLCD.Line.kUser1, 1, "3.05+   ");
            kickerLeftTen.set(0);
            kickerRightSeven.set(0);
        }
        lcd.println(DriverStationLCD.Line.kUser2, 1, "" + autoTimeCurr);
        lcd.updateLCD();
    }

    public void teleopInit() {
        malachi.setEnabled(false);
        resetLCD();
//        encFrontLeft.reset();
//        encFrontRight.reset();
//        encBackLeft.reset();
//        encBackRight.reset();
//        pidFrontLeft.reset();
//        pidFrontRight.reset();
//        pidBackLeft.reset();
//        pidBackRight.reset();
//        encFrontLeft.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
//        encFrontRight.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
//        encBackLeft.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
//        encBackRight.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
//        pidFrontLeft.enable();
//        pidFrontRight.enable();
//        pidBackLeft.enable();
//        pidBackRight.enable();
    }

    public void teleopPeriodic() {

        lcd.println(DriverStationLCD.Line.kUser6, 1, "Limit: " + limit.get());
        lcd.updateLCD();

        //kThrottle = X rotation
        //sens = [0,1.0], the speed of the robot
        // Drive
//        pidFrontLeft.setInputRange(-sens, sens);
//        pidFrontRight.setInputRange(-sens, sens);
//        pidBackLeft.setInputRange(-sens, sens);
//        pidBackRight.setInputRange(-sens, sens);
        double x = getDeadZone(controller.getAxis(Joystick.AxisType.kX), 0.25) * sens;
        double y = getDeadZone(-controller.getAxis(Joystick.AxisType.kY), 0.25) * sens;
        double r = getDeadZone(controller.getAxis(Joystick.AxisType.kThrottle), 0.25) * sens;
//        pidFrontLeft.setSetpoint(-(x + y + r));
//        pidFrontRight.setSetpoint(-x + y - r);
//        pidBackLeft.setSetpoint(-(-x + y + r));
//        pidBackRight.setSetpoint(x + y - r);
        motorFrontLeftOne.set(x + y + r);
        motorFrontRightTwo.set(x - y + r);
        motorBackLeftThree.set(-x + y + r);
        motorBackRightFour.set(-x - y + r);

        // Lift
        if (trippDriving) {
            if (controller.getRawButton(6)) { // Arms Down
                shoulder(1);
            } else if (controller.getRawButton(5) && limit.get()) { // Arms Up
                shoulder(-1);
            } else {
                shoulder(0);
            }
        } else {
//            if (limit.get() == true) {
//                shoulderLeft.set(-getDeadZone(controller.getAxis(Joystick.AxisType.kZ), 0.25));
//            }
//            else
//            {
//                shoulderLeft.set(0);
//                shoulderRight.set(0);
//            }
//            shoulderRight.set(getDeadZone(controller.getAxis(Joystick.AxisType.kZ), 0.25));

            if (controller.getAxis(Joystick.AxisType.kZ) > 0.8) {
                shoulder(1);
            } else if (controller.getAxis(Joystick.AxisType.kZ) < -0.8 && limit.get()) {
                shoulder(-1);
            } else {
                shoulder(0);
            }
        }

        // Pickup
        if (trippDriving) {
            if (controller.getRawButton(1)) {
                intake(-1);
            } else if (controller.getRawButton(2)) {
                intake(1);
            } else {
                intake(0);
            }
        } else {
            if (controller.getRawButton(5)) {
                intake(-1);
            } else if (controller.getRawButton(6)) {
                intake(1);
            } else {
                intake(0);
            }
        } //2'1"   to    7'6"
        //dead 3'3" to 5'7"

        // Increase Speed
        if (controller.getRawButton(9) && !hasChangedSpeed) {
            sens = (sens == FAST_SPEED) ? SLOW_SPEED : FAST_SPEED;
            hasChangedSpeed = true;
        } else if (!controller.getRawButton(9) && hasChangedSpeed) {
            hasChangedSpeed = false;
        }

        //High Kicker
        if (!isKickingHigh && !isKickingLow) {
            if (trippDriving) {
                if (controller.getAxis(Joystick.AxisType.kZ) < -0.8) {
                    isKickingHigh = true;
                    kickerTimeStart = Timer.getFPGATimestamp();
                } else if (controller.getAxis(Joystick.AxisType.kZ) > 0.8) {
                    isKickingLow = true;
                    kickerTimeStart = Timer.getFPGATimestamp();
                }
            } else {
                if (controller.getRawButton(1)) {
                    isKickingHigh = true;
                    kickerTimeStart = Timer.getFPGATimestamp();
                } else if (controller.getRawButton(2)) {
                    isKickingLow = true;
                    kickerTimeStart = Timer.getFPGATimestamp();
                }
            }
        } else if (isKickingHigh) {
            kickHigh();
        } else if (isKickingLow) {
            kickLow();
        }

        //Switch controls
        if (!isSwitchingControls && controller.getRawButton(7)) {
            trippDriving = !trippDriving;
            isSwitchingControls = true;
        } else if (!controller.getRawButton(7)) {
            isSwitchingControls = false;
        }

        if (trippDriving) {
            lcd.println(DriverStationLCD.Line.kUser1, 1, "Drive Tripp       ");
        } else {
            lcd.println(DriverStationLCD.Line.kUser1, 1, "Drive Max/Terrence");
        }
        lcd.println(DriverStationLCD.Line.kUser2, 1, "X: " + controller.getAxis(Joystick.AxisType.kX));
        lcd.println(DriverStationLCD.Line.kUser3, 1, "Y: " + controller.getAxis(Joystick.AxisType.kY));
        lcd.println(DriverStationLCD.Line.kUser4, 1, ultrasonic.get() ? "Eject Ball" : "              ");
        lcd.println(DriverStationLCD.Line.kUser5, 1, "Max Speed: " + sens + " ");
        //ultrasonic
        lcd.updateLCD();
    }

    private void resetLCD() {
        String blankLine = "";
        for (int i = 0; i < DriverStationLCD.kLineLength; i++) {
            blankLine += " ";
        }
        lcd.println(DriverStationLCD.Line.kUser1, 1, blankLine);
        lcd.println(DriverStationLCD.Line.kUser2, 1, blankLine);
        lcd.println(DriverStationLCD.Line.kUser3, 1, blankLine);
        lcd.println(DriverStationLCD.Line.kUser4, 1, blankLine);
        lcd.println(DriverStationLCD.Line.kUser5, 1, blankLine);
        lcd.println(DriverStationLCD.Line.kUser6, 1, blankLine);
        lcd.updateLCD();
    }

    private double getDeadZone(double num, double dead) {
        if (num < 0) {
            if (num < -dead) {
                return num;
            } else {
                return 0;
            }
        } else {
            if (num > dead) {
                return num;
            } else {
                return 0;
            }
        }
    }

    private void kickHigh() {
        double kickerTimeCurr = Timer.getFPGATimestamp();
        if (kickerTimeCurr - kickerTimeStart <= 0.3) {
            kickerLeftTen.set(-1.2);
            kickerRightSeven.set(-1.2);
        } else if (kickerTimeCurr - kickerTimeStart <= 0.55) {
            kickerLeftTen.set(0.15);
            kickerRightSeven.set(0.15);
        } else {
            isKickingHigh = false;
            kickerLeftTen.set(0);
            kickerRightSeven.set(0);
        }
    }

    private void kickLow() {
        double kickerTimeCurr = Timer.getFPGATimestamp();
        if (kickerTimeCurr - kickerTimeStart <= 0.5) {
            kickerLeftTen.set(-1.2);
            kickerRightSeven.set(-1.2);
        } else if (kickerTimeCurr - kickerTimeStart <= 0.7) {
            kickerLeftTen.set(0.25);
            kickerRightSeven.set(0.25);
        } else {
            isKickingLow = false;
            kickerLeftTen.set(0);
            kickerRightSeven.set(0);
        }
    }

    public void intake(int mode) {
        if (mode == -1) {
            intakeLeftEight.set(1);
            intakeRightNine.set(-1);
        } else if (mode == 1) {
            intakeLeftEight.set(-1);
            intakeRightNine.set(1);
        } else {
            intakeLeftEight.set(0);
            intakeRightNine.set(0);
        }
    }

    public void shoulder(int mode) {
        if (mode == -1) {
            shoulderLeftSix.set(-0.7);
            shoulderRightFive.set(0.7);
        } else if (mode == 1) {
            shoulderLeftSix.set(0.7);
            shoulderRightFive.set(-0.7);
        } else {
            shoulderLeftSix.set(0);
            shoulderRightFive.set(0);
        }
    }
}
