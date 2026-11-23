package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class GamePadLesson extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        double speedForward = -gamepad1.left_stick_y / 2.0;
        double xLeftMinusRight = gamepad1.left_stick_x - gamepad1.right_stick_x;
        double sumTriggers = gamepad1.right_trigger + gamepad1.left_trigger;

        telemetry.addData(" left x", gamepad1.left_stick_x);
        telemetry.addData(" left y", speedForward);

        telemetry.addData("a button", gamepad1.a);
        telemetry.addData("b button", gamepad1.b);

        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("right y", gamepad1.right_stick_y);

        telemetry.addData("x difference (left - right)", xLeftMinusRight);
        telemetry.addData("sum of triggers", sumTriggers);
    }

    /*
    1. add telemtery for right joystick
    2. add b button
    3. x differences
    4. add sum of both triggers


     */
}
