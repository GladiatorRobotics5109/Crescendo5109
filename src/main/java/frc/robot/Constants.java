// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveConstants {
    public static final boolean kFieldRelative = true;

    public static final double kMaxSpeed = 15; // m/s
    public static final double kMaxAngularSpeed = 3 * Math.PI;

    // used for holonomic only
    public static final double kMaxAngularAcceleration = Math.PI;
    
    // bot constants
    public static final Translation2d kModulePosFrontLeft = new Translation2d(0.2921, 0.2921);
    public static final Translation2d kModulePosFrontRight = new Translation2d(0.2921, -0.2921);
    public static final Translation2d kModulePosBackLeft = new Translation2d(-0.2921, 0.2921);
    public static final Translation2d kModulePosBackRight = new Translation2d(-0.2921, -0.2921);
    
    // Swerve Module Constants
    public static final double kWheelRadius = 0.0508;
    
    // SDS MK4 L1 gear ratios
    public static final double kSwerveDriveGearRatio = 8.14;
    public static final double kSwerveTurnGearRatio = 12.8;
    
    // neo constants
    public static final double kNeoTicksPerMotorRevolution = 42;
    public static final double kNeoTicksPerMotorRadian = kNeoTicksPerMotorRevolution / (2 * Math.PI);
    public static final double kNeoTicksPerWheelRadian = kNeoTicksPerMotorRadian * kSwerveDriveGearRatio;
    // TODO: test this
    // public static final double kNeoTicksPerTurnWheelRadian = kNeoTicksPerMotorRadian * kSwerveTurnGearRatio;
    // should be?
    public static final double kNeoTicksPerTurnWheelRadian = kSwerveTurnGearRatio / (2 * Math.PI);
    
    // kraken constants
    public static final double kKrakenTicksPerMotorRevolution = 2000;
    public static final double kKrakenTicksPerMotorRadian = kKrakenTicksPerMotorRevolution / (2 * Math.PI);
    public static final double kKrakenTicksPerWheelRadian = kKrakenTicksPerMotorRadian * kSwerveDriveGearRatio;
    public static final double kKrakenTicksPerTurnWheelRadian = kKrakenTicksPerMotorRadian * kSwerveTurnGearRatio;
    
    public static final AHRS kNavX = new AHRS(SPI.Port.kMXP);

    // more pid controllers found in swerve module classes

    // used for holonomic only
    public static final PIDController kPidControllerHolonomicX = new PIDController(0.3, 0, 0);
    public static final PIDController kPidControllerHolonomicY = new PIDController(0.3, 0, 0);
    public static final ProfiledPIDController kPidControllerHolonomicRot = new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(kMaxAngularSpeed, kMaxAngularAcceleration));
  }

  public static class DriveTeamConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadzone = 0.1;
  }
}