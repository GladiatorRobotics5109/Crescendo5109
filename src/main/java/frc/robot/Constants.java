// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

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

    public static final double kWheelRadius = 0.0508;

    // l1 mk4 gear ratios
    public static final double kSwerveDriveGearRatio = 8.14 / 1;
    public static final double kSwerveTurnGearRatio = 4 / 15; // think this is right for l1

    public static final double kNeoTicksPerMotorRadian = 42 / (2 * Math.PI);
    public static final double kNeoTicksPerWheelRadian = kNeoTicksPerMotorRadian * 8.14;
    public static final double kNeoTicksPerTurnWheelRadian = 12.8 / (2 * Math.PI);

    public static final double kKrakenTicksPerMotorRadian = 2000 / (2 * Math.PI);
    public static final double kKrakenTicksPerWheelRadian = kKrakenTicksPerMotorRadian * 8.14;
    public static final double kKrakenTicksPerTurnWheelRadian = kSwerveTurnGearRatio * 2000 / (2 * Math.PI);

    public static final AHRS kNavX = new AHRS(SPI.Port.kMXP);
  }

  public static class DriveTeamConstants {
    public static final int kDriverControllerPort = 0;
  }
}