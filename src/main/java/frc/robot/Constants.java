// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.SPI;

import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final double kJoystickDeadzone = 0.2;
  
  public static class SwerveConstants {
    // public static final double kMaxSpeed = 10;
    // public static final double kMaxAngularSpeed = 2.5 * Math.PI;

    public static final double kMaxSpeed = 10;
    public static final double kMaxAngularSpeed = 3 * Math.PI;


    public static final Translation2d kModulePosFrontLeft = new Translation2d(0.2921+0.00635, 0.2921+0.00635);
    public static final Translation2d kModulePosFrontRight = new Translation2d(0.2921+0.00635, -0.2921-0.00635);
    public static final Translation2d kModulePosBackLeft = new Translation2d(-0.2921-0.00635, 0.2921+0.00635);
    public static final Translation2d kModulePosBackRight = new Translation2d(-0.2921-0.00635, -0.2921-0.00635);

    public static final double kModuleEncoderOffsetFrontLeft = 2.736; // 0.445 rotations
    public static final double kModuleEncoderOffsetFrontRight = 0.319; // 0.061 rotations
    public static final double kModuleEncoderOffsetBackLeft = 4.564; // 0.721 rotations
    public static final double kModuleEncoderOffsetBackRight = 5.188; // 0.825 rotations
    

    public static final AHRS kNavX = new AHRS(SPI.Port.kMXP);

    public static final double kDriveBaseRadius = new Translation2d().getDistance(kModulePosBackLeft) + 0.05;

    public static class AutonConstants {
      public static final double kMaxSpeed = 0.25;
      public static final double kMaxAcceleration = .5;

      public static final double kMaxAngularSpeed = 1 * Math.PI;
      public static final double kMaxAngularAcceleration = 1 * Math.PI;

      public static final PIDConstants kTranslationPID = new PIDConstants(1, 0, 0);
      public static final PIDConstants kRotationPID = new PIDConstants(1, 0, 0);

      public static final ReplanningConfig kReplanningConfig = new ReplanningConfig(false, false);

      public static final Boolean kAutoMirror = false; //Crescendo Field isn't symmetric, so won't work

      public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
        kTranslationPID, 
        kRotationPID, 
        kMaxSpeed, 
        kDriveBaseRadius, 
        kReplanningConfig, 
        Robot.kDefaultPeriod
      );
    }
  }

  public static class ModuleConstants {

      public static final double kWheelRadius = 0.0508;
  
      // L1 MK4 gear ratios
      public static final double kSwerveDriveGearRatio = 8.14; // 8.14 motor rotations = 1 revolution
      public static final double kSwerveTurnGearRatio = 12.8; // 12.8 motor rotations = 1 revolution
      
      // public static final double kNeoTicksPerRevolution = 42;
      // public static final double kNeoTicksPerMotorRadian = kNeoTicksPerRevolution / (2 * Math.PI);
      // public static final double kNeoTicksPerWheelRadian = kNeoTicksPerMotorRadian * kSwerveDriveGearRatio;
      // public static final double kNeoTicksPerTurnWheelRadian = kSwerveTurnGearRatio / (2 * Math.PI);
      
      public static final double kKrakenTicksPerRevolution = 2000;
      public static final double kKrakenTicksPerMotorRadian = kKrakenTicksPerRevolution / (2 * Math.PI);
      public static final double kKrakenTicksPerWheelRadian = kKrakenTicksPerMotorRadian * kSwerveDriveGearRatio;
      public static final double kKrakenTicksPerTurnWheelRadian = kKrakenTicksPerMotorRadian * kSwerveTurnGearRatio;
      
      public static final double kDrivePositionConversionFactor = (kWheelRadius*2) * Math.PI / kSwerveDriveGearRatio; // rotations -> meters (1 motor turn x x 2pi*wheel radius / 8.14 motor turns)
      public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60.0; // rpm -> m/s 

      // FOR RELATIVE ENCODERS
      public static final double kTurnPositionConversionFactor = (2 * Math.PI) / kSwerveTurnGearRatio; // rotations -> radians (1 motor turn x 2pi / 12.8 motor turns)
      public static final double kTurnVelocityConversionFactor = kTurnPositionConversionFactor / 60.0; // rpm -> rad/s
  
      public static final double kModuleTurnPositionConversionFactor = (2 * Math.PI);
      
      public static final double kDriveP = 1;
      public static final double kDriveI = 0;
      public static final double kDriveD = 0.05;
  
      public static final double kTurnP = 1.5;
      public static final double kTurnI = 0;
      public static final double kTurnD = 0;
    
  }

  public static class ShooterConstants {
    public static final int kLeftShooterMotorPort = 55;
    public static final int kRightShooterMotorPort = 9;
    public static final int kFeederMotorPort = 6;
    public static final int kWinchMotorPort = 56;
    public static final int kBarMotorPort = 78;

    public static final double kShooterP = 1;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0.05;
    
    public static final double kFeederP = 2;
    public static final double kFeederI = 0;
    public static final double kFeederD = 0;
    
    public static final double kBarP = 0;
    public static final double kBarI = 0;
    public static final double kBarD = 0;
    
    public static final double kWinchP = 0;
    public static final double kWinchI = 0;
    public static final double kWinchD = 0;
    
    public static final int kFeederSensorChannel = 0;
    
    public static final double kPivotWinchInitialRadius = 0;
    public static final double kPivotWinchFinalRadius = 0;
    public static final double kPivotWinchAverageRadius = (kPivotWinchInitialRadius + kPivotWinchFinalRadius) / 2;
    
    public static final double kBarWinchInitialRadius = 0;
    public static final double kBarWinchFinalRadius = 0;
    public static final double kBarWinchAverageRadius = (kBarWinchInitialRadius + kBarWinchFinalRadius) / 2;
    
    public static final double kBarGearRatio = 0;
    public static final double kWinchGearRatio = 0;
    
    public static final double kBarPositionConversionFactor = kBarWinchAverageRadius * (2 * Math.PI) / kBarGearRatio; 
    public static final double kWinchPositionConversionFactor = kPivotWinchAverageRadius * (2 * Math.PI) / kWinchGearRatio;
    
  }
  
  public static class IntakeConstants {
    public static final int kIntakeMotorPort = 56;
    public static final int kIntakeSensorChannel = 0;

  }

  public static class DriveTeamConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorJoystickPort = 1;
  }

  public static class VisionConstants {

    public static final Map<String, Transform3d> kVisionSources = new HashMap<>(){{
      put("Camera1", new Transform3d(
        0.2921,
        0.3683,
        0.3048,
        new Rotation3d())
      );
      //put("CameraTwo", new Transform3d());
    }};

      public static final Transform3d kCameraPos = new Transform3d(
        0.2921, 
        0.3683, 
        0.3048, 
        new Rotation3d()
      );

    public static final AprilTagFields kApriltagLayout = AprilTagFields.k2024Crescendo;
  }

}