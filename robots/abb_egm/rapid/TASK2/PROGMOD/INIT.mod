MODULE INIT
!Task to check for motion suppervision trigger
!Restart execution of the program when collision is solved.
!This is a SEMISTATIC Task, so it will start automatically.

!Interrupts
VAR intnum iMotionSup;

!Main procedure
PROC main()
CONNECT iMotionSup WITH CollisionDet;
ISignalDO USER_MOTIONSUP_ON,1,iMotionSup;

!//Initialization
SetDO USER_START_OUTPUT, 0;
SetDO USER_RESET_MOTION, 0;

!//Instructions to run the ABB Server
TPWrite "ABB Server:";
TPWrite "1 - Turn motors on.";
TPWrite "2 - Press PP to main in the controller.";
TPWrite "3 - Press Play to run server.";

!//Infinite trivial loop
WHILE TRUE DO
ENDWHILE

ENDPROC

!//Trap routine triggered with USER_MOTIONSUP_ON, a digital signal conencted to the system signal MotSupTrigg
TRAP CollisionDet
    TPWrite "INIT: Collision Detected.";
	!//Here we need to wait until robot backtracks;
	!//For lack of a better strategy, we will just wait for a fix amount of time.
	WaitTime 3.0;
	
	!//We clear the current path, to remove any residual motions in the path queue.
	ClearPath;
	
	!//Set digital output that will trigger the restart of both SERVER and LOGGER.
	SetDO USER_START_OUTPUT, 1;
    TPWrite "INIT: Restart SERVER and LOGGER.";
	
	!//Wait until it is restarted.
	!//For lack of a better strategy, we will just wait for a fix amount of time.
	WaitTime 2.0;
	
	!//Set digital output that will trigger the Error handler in Server that will restart the motion of the robot.
 	SetDO USER_RESET_MOTION, 1;
ENDTRAP
ENDMODULE
