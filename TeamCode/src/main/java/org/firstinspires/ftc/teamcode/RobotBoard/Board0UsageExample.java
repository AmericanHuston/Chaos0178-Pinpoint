package org.firstinspires.ftc.teamcode.RobotBoard;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name= "Board0UsageExample", group="TeleOp")
public class Board0UsageExample extends LinearOpMode {
    Board0 board = new Board0();
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()) {
            board.setDriveState(Board0.drivingDirection.FORWARD);
            board.setDriveSpeed(0.5);
            board.setDriveTime(1000);
            board.stateMachinesAct(Board0.stateMachineAct.DRIVE);
            //Should drive straight for 1 second at half speed
            board.setClawState(Board0.clawPositions.CLAW_OPEN);
            board.stateMachinesAct(Board0.stateMachineAct.CLAW);
            sleep(1000);
            board.setClawState(Board0.clawPositions.CLAW_CLOSED);
            board.stateMachinesAct(Board0.stateMachineAct.CLAW);
            board.setArmState(Board0.armStates.BELOW_BAR);
            board.stateMachinesAct(Board0.stateMachineAct.ARM);
            board.setArmState(Board0.armStates.RESTING);
            board.stateMachinesAct(Board0.stateMachineAct.ARM);
            requestOpModeStop();
        }
    }
}
