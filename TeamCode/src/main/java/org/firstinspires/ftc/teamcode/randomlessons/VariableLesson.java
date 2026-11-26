package org.firstinspires.ftc.teamcode.randomlessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class VariableLesson extends OpMode {
    @Override
    public void init() {
        int teamNumber = 23014;
        double motorSpeed = 0.75;
        boolean clawClosed = true;
        String name = "Tartan Highlanders";
        int motorAngle = 90;

        telemetry.addData("team number", teamNumber);

        telemetry.addData("motor speed", motorSpeed);

        telemetry.addData("claw closed",clawClosed);

        telemetry.addData("name", name);

        telemetry.addData("motor angle", motorAngle);


    }

    @Override
    public void loop() {

        /*
        1.change the string variable name to team name
        2. new int variable called motor angle and store angle between 0 to 180 display init
         */
    }
}
