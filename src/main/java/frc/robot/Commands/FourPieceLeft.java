// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.TeamSelector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPieceLeft extends SequentialCommandGroup {
    /** Creates a new SimpleTwoPieceAuton. */
    public FourPieceLeft(Shooter m_shooter, Intake m_intake, AutoShoot m_autoshoot, CommandSwerveDrivetrain m_drivetrain, Command path, Command path2,
            Command path3) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.runOnce(() -> m_drivetrain.seedFieldRelative(TeamSelector.getTeamColor()?m_drivetrain.getStartingPath("4-2a Red Piece"):m_drivetrain.getStartingPath("4-2a Piece"))),
                Commands.parallel(
                        Commands.run(() -> m_shooter.setPivotAngle()),
                        Commands.startEnd(() -> m_shooter.setShooterVelocity(), () -> m_shooter.stopShooter()),
                        Commands.race(Commands.run(() -> m_autoshoot.targetAuton(TeamSelector.getTeamColor()?4:7)), new WaitCommand(2)),
                        Commands.sequence(
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot),
                                Commands.deadline(
                                        Commands.race(Commands.sequence(path, new WaitCommand(1.25)), Commands.run(() -> m_shooter.targetAngle = 190)),
                                        new HandleAutonShoot(m_intake, m_shooter)),
                                        
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot),
                                Commands.deadline(
                                        Commands.race(Commands.sequence(path2, new WaitCommand(0.5)), Commands.run(() -> m_shooter.targetAngle = 190)),
                                        new HandleAutonShoot(m_intake, m_shooter)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot),
                                Commands.deadline(
                                        Commands.race(Commands.sequence(path3, new WaitCommand(0.5)), Commands.run(() -> m_shooter.targetAngle = 190)),
                                        new HandleAutonShoot(m_intake, m_shooter)),
                                new WaitCommand(1),
                                Commands.race(new WaitCommand(2),
                                        Commands.run(() -> m_shooter.targetAngle = m_autoshoot
                                                .getTargetTagAngles(TeamSelector.getTeamColor()?4:7)[0])),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot))));
    }
}
