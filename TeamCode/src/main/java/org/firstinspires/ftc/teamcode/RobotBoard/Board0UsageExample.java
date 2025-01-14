package org.firstinspires.ftc.teamcode.RobotBoard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Board0UsageExample", group="TeleOp")
public class Board0UsageExample extends LinearOpMode {
    Board0 board = new Board0();
    public void runOpMode(){
        board.setDriveState(Board0.drivingDirection.FORWARD);
        board.setDriveSpeed(0.5);
        board.setDriveTime(1000);
        board.stateMachinesAct(Board0.stateMachineAct.DRIVE);
        //Should drive straight for 1 second at half speed
    }
}
