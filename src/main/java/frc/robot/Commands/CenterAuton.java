// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.TeamSelector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAuton extends SequentialCommandGroup {
    /** Creates a new SimpleTwoPieceAuton. */
    Intake m_intake;

    public CenterAuton(Shooter m_shooter, Intake m_intake, AutoShoot m_autoshoot,
            CommandSwerveDrivetrain m_drivetrain, Command path, Command path2,
            Command path3, Command path4, Command path5, Command path6) {
        this.m_intake = m_intake;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.runOnce(
                        () -> m_drivetrain.seedFieldRelative(TeamSelector
                                .getTeamColor() ? m_drivetrain.getStartingPath("Center-1 Red")
                                        : m_drivetrain.getStartingPath(
                                                "Center-1"))),
                Commands.parallel(
                        Commands.run(() -> m_shooter.setPivotAngle()),
                        Commands.runEnd(() -> m_shooter.setShooterVelocity(),
                                () -> m_shooter.stopShooter()),
                        Commands.sequence(
                                Commands.deadline(
                                        path,
                                        new HandleAutonShoot(m_intake,
                                                m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 187)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot,
                                        m_drivetrain),
                                Commands.deadline(
                                        path2,
                                        new HandleAutonShoot(m_intake,
                                                m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 187)),
                                new EitherCommand(Commands.deadline(
                                        path6,
                                        new HandleAutonShoot(m_intake,
                                                m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 187)),
                                        Commands.sequence(Commands.deadline(
                                                path3,
                                                new HandleAutonShoot(
                                                        m_intake,
                                                        m_shooter),
                                                Commands.run(() -> m_shooter.targetAngle = 187)),
                                                new AlignAndShoot(
                                                        m_shooter,
                                                        m_intake,
                                                        m_autoshoot,
                                                        m_drivetrain),
                                                Commands.deadline(
                                                        path4,
                                                        new HandleAutonShoot(
                                                                m_intake,
                                                                m_shooter),
                                                        Commands.run(() -> m_shooter.targetAngle = 187))),
                                        () -> isNote()).getCommand(),
                                Commands.deadline(
                                        path5,
                                        new HandleAutonShoot(m_intake,
                                                m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 187)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot,
                                        m_drivetrain))));
    }

    boolean isNote() {
        SmartDashboard.putBoolean("Is there a note 8", m_intake.irSensor.get());
        return m_intake.irSensor.get();
    }
}
