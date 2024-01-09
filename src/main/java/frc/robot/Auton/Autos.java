// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Autos {
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  private static SendableChooser<String> m_autoChooser = new SendableChooser<String>();

  private static SwerveSubsystem m_swerve;

  public static void init() {
    // PUT THIS ON CONSTRUCTOR OF EACH COMMAND LATER
    // load trajectory from pathweaver json
    // String trajectoryJSONPath = "paths/YourPath.wpilib.json";

    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONPath);
    //   Common.currentAutonTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } 
    // catch (IOException e) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONPath, e.getStackTrace());
    // }

    m_autoChooser.setDefaultOption("Default Auto", Constants.AutoConstants.kDefaultAuto);
    // m_autoChooser.addOption() as necessary

    SmartDashboard.putData("Auto Choice", m_autoChooser);
  }

  public static void SetSweve(SwerveSubsystem swerve) {
    m_swerve = swerve;
  }

  public static Command getCurrentAutoCommand() {
    switch (m_autoChooser.getSelected()) {
      case Constants.AutoConstants.kDefaultAuto:
        return AutonFactory.getDefaultAutoCommand(m_swerve);
      default:
        return AutonFactory.getDefaultAutoCommand(m_swerve);
    }
  }
}
