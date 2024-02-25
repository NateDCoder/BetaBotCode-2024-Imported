// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPieceLeft extends SequentialCommandGroup {
    /** Creates a new SimpleTwoPieceAuton. */
    public FourPieceLeft(Shooter m_shooter, Intake m_intake, AutoShoot m_autoshoot, Command path, Command path2,
            Command path3) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.parallel(
                        Commands.run(() -> m_shooter.setPivotAngle()),
                        Commands.startEnd(() -> m_shooter.setShooterVelocity(), () -> m_shooter.stopShooter()),
                        Commands.startEnd(() -> m_intake.intakeMotorPower(Constants.INTAKE_POWER),
                                () -> m_intake.intakeMotorPower(0)),
                        Commands.race(Commands.run(() -> m_autoshoot.targetAll(7)), new WaitCommand(2)),
                        Commands.sequence(
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot),
                                Commands.deadline(
                                        path,
                                        new HandleAutonShoot(m_intake, m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 190)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot),
                                Commands.deadline(
                                        path2,
                                        new HandleAutonShoot(m_intake, m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 190)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot),
                                Commands.deadline(
                                        path3,
                                        new HandleAutonShoot(m_intake, m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 190)),
                                Commands.race(new WaitCommand(2),
                                        Commands.run(() -> m_shooter.targetAngle = m_autoshoot
                                                .getTargetTagAngles(7)[0])),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot))));
    }
}
