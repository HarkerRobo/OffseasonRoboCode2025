package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import harkerrobolib.util.MathUtil;

public class ElevatorManual extends Command {

    double initialPose;

    public ElevatorManual() {
        addRequirements(Elevator.getInstance());
    }

    @Override
    public void initialize()
    {
        initialPose = Elevator.getInstance().getPosition();
    }

    @Override
    public void execute() 
    {
        Elevator.getInstance().moveToPosition(initialPose);
    }

    public boolean isFinished() {
        return false;
    }
}