package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.arcrobotics.ftclib.util.Timing;


@Config
class RobotConstantsLIFTY {
    public static double kP = 0.75;
    public static double kI = 0.005;
    public static double kD = 0.01;
}

public class RaiseLift extends CommandBase {
    private final Callisto robot;
    private final Lift lift;
    private final PIDController pid;
    private final int targetPosition;
    private final double TOLERANCE = 5; // acceptable error in ticks
    private final double TIMEOUT = 4.0; // seconds
    protected Timing.Timer timer;

    public RaiseLift(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.lift = robot.lift;
        this.targetPosition = targetPosition;
        pid = new PIDController(RobotConstantsLIFTY.kP, RobotConstantsLIFTY.kI, RobotConstantsLIFTY.kD);
        pid.setSetPoint(targetPosition);
        pid.setTolerance(TOLERANCE);

        timer = new Timing.Timer((long) TIMEOUT);
        //TODO: Figure out if we need this line ;)
        lift.motor1.setTargetPosition(targetPosition);

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        timer.start();

    }

    @Override
    public void execute() {
        double currentPosition = lift.motor1.getCurrentPosition();
        double power = pid.calculate(currentPosition);
        power = Math.max(Math.min(power, 0.75), -0.75); // constrain power between -0.5 and 0.5

        lift.motor1.set(power);

        if(pid.atSetPoint()) {
            lift.motor1.stopMotor();
        }

        // Telemetry for debugging
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Current Position", currentPosition);
        robot.telemetry.addData("Motor Power", power);
    }

    @Override
    public boolean isFinished() {
        return lift.motor1.atTargetPosition() && timer.done();
                //pid.atSetPoint() || timer.seconds() > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        lift.motor1.stopMotor();

    }
}