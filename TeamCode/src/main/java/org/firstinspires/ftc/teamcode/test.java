package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class test extends OpMode {
    @Override
    public void init() {
        telemetry.addData("hello", "world");
    }

    static int counter=0;
    @Override
    public void loop(){
        if(counter<10) {
            telemetry.addData("count", "" + counter);
        }
        counter++;
    }
}