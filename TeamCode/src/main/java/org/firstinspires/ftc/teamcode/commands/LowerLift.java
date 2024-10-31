
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LowerLift extends CommandBase {
    // REFERENCES
    private Callisto robot;
    private Lift lift;
    private final GamepadEx player2;
    // ASSETS
    private boolean controller = true;
    private boolean isNegative = false;
    private int targetPosition;
    private final double TIMEOUT = 1.5;
    protected Timing.Timer timer;



    public LowerLift(Callisto robot, int targetPosition) {
        //TODO: Auto timeout this command
        this.robot = robot;
        this.lift = robot.lift;
        this.targetPosition = targetPosition;
        this.player2 = robot.player2;

        lift.motor1.setTargetPosition(500);

        timer = new Timing.Timer((long) TIMEOUT);

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if ((Math.abs(lift.basket.getPosition() - 0.4) > 0.1)){
            lift.levelBasket();
        }
        timer.start();
        lift.motor1.setRunMode(Motor.RunMode.PositionControl);
    }


    @Override
    public void execute() {
//        // Set the target position based on button press
//        if (player2.gamepad.dpad_down) {
//            targetPosition = Constants.LOW_HEIGHT;
//        } else if (player2.gamepad.dpad_left) {
//            targetPosition = Constants.MID_HEIGHT;
//        } else if (player2.gamepad.dpad_up) {
//            targetPosition = Constants.HIGH_HEIGHT;
//        }

        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Position", lift.motor1.getCurrentPosition());

        lift.motor1.set(-.25);


       /* // Move towards the target position
        if (!lift.motor1.atTargetPosition()) {
            lift.motor1.set(isNegative ? -0.5 : 0.5);
            if (lift.motor1.getCurrentPosition() <= 25) {
                isNegative = true;

            }*/

    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return Math.abs(lift.motor1.getCurrentPosition() - targetPosition) < 10 && timer.done();
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        // Stop the drive if interrupted or completed
        lift.motor1.stopMotor();
        lift.motor1.resetEncoder();
    }
}
 