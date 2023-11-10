// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static final double kTurnMotorGearRatio = 12.8;

  public static final double kJoystickDeadzone = 0.1;

  public static class SwerveConstants {
    public static final double kMaxSpeed = 15;
    public static final double kMaxAngularSpeed = 3 * Math.PI;
    
    // TODO: select right CAN ids for motors
    public static final SwerveModule kFLModule = new SwerveModule(new Translation2d(0.2921, 0.2921), "frontLeft", 0, 0, 0);
    public static final SwerveModule kFRModule = new SwerveModule(new Translation2d(0.2921, -0.2921), "frontRight", 0, 0, 0);
    public static final SwerveModule kBLModule = new SwerveModule(new Translation2d(-0.2921, 0.2921), "backLeft", 0, 0, 0);
    public static final SwerveModule kBRModule = new SwerveModule(new Translation2d(-0.2921, -0.2921), "backRight", 0, 0, 0);

    public static final AHRS kNavX = new AHRS(SPI.Port.kMXP);
  }

  public static class DriveTeamConstants {
    public static final int kDriverControllerPort = 0;
  }
}