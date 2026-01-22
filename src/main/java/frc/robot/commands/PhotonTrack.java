// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Photon;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.Photon;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PhotonTrack extends Command {

  Photon objPhoton;

  private final CommandSwerveDrivetrain objSwerve;
  private final double dMaxSpeed;
  private final double dMaxAngularRate;
 
  private final DoubleSupplier dsDriverLeftX;
  private final DoubleSupplier dsDriverLeftY;
 
  private double dCmdLeftX, dCmdLeftY;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  


  /** Creates a new PhotonTrack. */
  public PhotonTrack(CommandSwerveDrivetrain objSwerve_in,double dMaxSpeed_in, double dMaxAngularRate_in, DoubleSupplier dsDriverLeftX_in, DoubleSupplier dsDriverLeftY_in, Photon objPhoton_in) {
    // Use addRequirements() here to declare subsystem dependencies.

    objPhoton = objPhoton_in;
    objSwerve = objSwerve_in;
    dMaxSpeed = dMaxSpeed_in;
    dMaxAngularRate = dMaxAngularRate_in;
    dsDriverLeftX = dsDriverLeftX_in;
    dsDriverLeftY = dsDriverLeftY_in;
    addRequirements(objSwerve);
  }

  private SwerveRequest.FieldCentric drive  = new SwerveRequest.FieldCentric()
  .withDeadband(0.02).withRotationalDeadband(0.02); // Add a 2% deadband 10% is CTRE Value
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dCmdLeftX = dsDriverLeftX.getAsDouble() * dMaxSpeed;
    dCmdLeftY = dsDriverLeftY.getAsDouble() * dMaxSpeed;

    objSwerve.setControl(
      drive.withVelocityX(dCmdLeftY).withVelocityY(dCmdLeftX).withRotationalRate()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
