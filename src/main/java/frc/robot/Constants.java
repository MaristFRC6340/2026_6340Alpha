// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    //
    public static final int kDriverControllerPort = 0;
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // speed multiplier :D
    public static final double SPEED_CONTROL = 0.5;
  }

  public static class TurretConstants {

    public static final int kRotationMotorID = 31;

    private static final Slot0Configs kSlot0Configs = new Slot0Configs()
    .withKA(0)
    .withKG(.3)
    .withKS(.0)
    .withKV(0)
    .withKP(15)
    .withKI(0)
    .withKD(0);

    public static final TalonFXConfiguration kRotationConfig = new TalonFXConfiguration()
    .withSlot0(kSlot0Configs);

    public static final double flywheelSpeed = 0.5;

  }

  public static class SwerveConstants {
    public static final double kPX = 2.75;
    public static final double kPY = 2.75;
    public static final double kPTheta = 2.5;
    public static final double kXTolerance = 0;
    public static final double kYTolerance = 0;
    public static final double kThetaTolerance = 0;
    public static double kStoredRadius = 3.9527559/2; // to be configured later??
    public static double kDrivebaseRadius = .409;
  }

  public static final class VisionConstants{
    public static final  double maximumAmbiguity = 0.25;
    public static final double debouncetimeMilli = 15;

    public static final class CameraTemplate{
      public static final String name = "tempalte"; // tempalte?
      public static  Rotation3d robotToCamTransform = new Rotation3d(0, Units.degreesToRadians(18), 0);
      public static  Translation3d robotToCamTranslation=new Translation3d(Units.inchesToMeters(-4.628),
      Units.inchesToMeters(-10.687),
      Units.inchesToMeters(16.129));
      public static  Matrix<N3, N1> singleTagStdDevs= VecBuilder.fill(2, 2, 8);
      public static  Matrix<N3, N1> multiTagStdDevsMatrix= VecBuilder.fill(0.5, 0.5, 1); 
    }

    
    }
   

    public static final double leftAlignmentX = .2435737274077523; //meters
    public static final double leftAlignmentY = 0.275;
    public static final double rightAlignmentX = .2435737274077523;
    public static final double rightAlignmentY = 0.623;
    public static final double troughAlignmentTheta = -1.860;
    public static final double troughAlignmentX = .379;
    public static final double troughAlignmentY = .748;
    public static final double thetaAlignment = -Math.PI/2; //degrees
    public static double maxAlignmentDistance = 1.5;

    public static final double xTolerance = .05;
    public static final double yTolerance = .05;
    public static final double thetaTolerance = .05;


  }

