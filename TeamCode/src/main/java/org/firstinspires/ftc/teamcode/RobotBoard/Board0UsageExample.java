package org.firstinspires.ftc.teamcode.RobotBoard;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name= "Board0UsageExample", group="TeleOp")
public class Board0UsageExample extends LinearOpMode {
    Board0 board = new Board0();

    public void runOpMode(){
        board.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            // defiantly doing stuff, I need to recommit
            board.setDriveState(Board0.drivingDirection.FORWARD);
            board.setDriveSpeed(0.5);
            board.setDriveTime(1000);
            board.stateMachinesThink(Board0.stateMachineAct.DRIVE);
            board.stateMachinesAct(Board0.stateMachineAct.DRIVE);
            //Should drive straight for 1 second at half speed
            board.setClawState(Board0.clawPositions.CLAW_OPEN);
            board.stateMachinesThink(Board0.stateMachineAct.CLAW);
            board.stateMachinesAct(Board0.stateMachineAct.CLAW);
            sleep(1000);
            board.setClawState(Board0.clawPositions.CLAW_CLOSED);
            board.stateMachinesThink(Board0.stateMachineAct.CLAW);
            board.stateMachinesAct(Board0.stateMachineAct.CLAW);
            board.setArmState(Board0.armStates.BELOW_BAR);
            board.stateMachinesThink(Board0.stateMachineAct.ARM);
            board.stateMachinesAct(Board0.stateMachineAct.ARM);
            sleep(1000);
            board.setArmState(Board0.armStates.RESTING);
            board.stateMachinesThink(Board0.stateMachineAct.ARM);
            board.stateMachinesAct(Board0.stateMachineAct.ARM);
            requestOpModeStop();
        }
    }
}