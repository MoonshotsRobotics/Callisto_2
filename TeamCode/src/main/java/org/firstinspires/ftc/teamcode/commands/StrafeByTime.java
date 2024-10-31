package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.Constants;

public class StrafeByTime extends CommandBase {

    // REFERENCES
    private Callisto robot;
    private Mecanum mecanum;

    // ASSETS
    private double speed;
    protected Action action;

    protected boolean finished = false;

    // TIMER
    protected Timing.Timer timer;

    // FTC DASHBOARD
    private FtcDashboard dashboard;


    public StrafeByTime(Callisto robot, double timeout, double speed) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.dashboard = FtcDashboard.getInstance();
        this.speed = speed;

        timer = new Timing.Timer((long)timeout);

        addRequirements(mecanum);

    }

    // Initialize method to build the trajectory
    @Override
    public void initialize() {
        // only start the timer once the command is run and not built
        timer.start();

        // Build the trajectory from the current pose to the target pose

    }

    // The execute method keeps updating the trajectory following
    @Override
    public void execute() {
        mecanum.drive(0, speed,0);
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        // Stop the drive if interrupted or completed
        mecanum.stop();
    }
}