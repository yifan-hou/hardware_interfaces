MODULE SERVER

!////////////////
!GLOBAL VARIABLES
!////////////////

!//To modify the default values go to method Initialize
PERS tooldata currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.8,[0,0,40],[1,0,0,0],0,0,0]];    
PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];   
PERS speeddata currentSpeed;
PERS zonedata currentZone;

!// Clock Synchronization
PERS bool startLog:=TRUE;
PERS bool startRob:=TRUE;

!// Mutex between logger and changing the tool and work objects
PERS bool frameMutex:=FALSE;

!//PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
VAR num instructionCode;
VAR string idCode;
CONST num MAX_PARAMS := 15;
VAR num params{MAX_PARAMS};
VAR num specialParams{5};
VAR num nParams;
PERS string ipController:= "192.168.1.98";
!"192.168.1.98";   Real Controller
!"192.168.180.128" VmWare
!"127.0.0.1";      Local Host
PERS num serverPort:= 5000;
PERS num loggerPort:= 5001;

VAR num remainingBytes;
VAR string remainingString;

VAR num messageState;
CONST num MESSAGE_NONE := 0;
CONST num MESSAGE_MIDDLE := 1;
CONST num MESSAGE_COMPLETE := 2;
CONST num MESSAGE_BAD := 3;

!//Logger sampling rate
PERS num loggerWaitTime:= 0.01;
!PERS num loggerWaitTime:= 0.1; This is adequate for virtual server

!//Motion of the robot
VAR robtarget cartesianTarget;
VAR jointtarget jointsTarget;
VAR bool moveComplete; !True when program pointer leaves a Move instruction.

!//Correct Instruction Execution and possible return values
VAR num ok;
CONST num SERVER_BAD_MSG :=  0;
CONST num SERVER_OK := 1;
CONST num SERVER_COLLISION := 2;
CONST num SERVER_BAD_IK := 3;
CONST num SERVER_BAD_FK := 4;
CONST num SERVER_TRAJ_TOO_LONG := 5;
	
!//Error Handler
VAR errnum ERR_MOTIONSUP := -1;

!//Interrupt to trap the digital output that signals the need to restart the motion of the robot.
VAR intnum iMotionReset;
VAR intnum iCollisionSensor;

!// Inverse and forward kinematic results
VAR jointtarget ik_result_j;
VAR robtarget fk_result_c;

!// Upper and lower joint bounds, in degrees
VAR num upper_joint_limits{6};
VAR num lower_joint_limits{6};

VAR tooldata tempTool;
VAR wobjdata tempWObj;
VAR zonedata tempZone;
VAR speeddata tempSpeed;

!// 
CONST num MAX_TRAJ_LENGTH := 50;
VAR num trajLength;
VAR robtarget trajPt{MAX_TRAJ_LENGTH};
VAR num trajTime{MAX_TRAJ_LENGTH};
VAR zonedata trajZone{MAX_TRAJ_LENGTH};
VAR bool trajOk;
VAR num idx;

!//Other
TASK PERS tooldata calibkinect:=[TRUE,[[-32.9885,0.584218,225.962],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata newvac:=[TRUE,[[-33.5757,-0.325804,225.926],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata stocopt:=[TRUE,[[-78.3527,-3.0397,157.523],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS wobjdata wobjstocopt:=[FALSE,TRUE,"",[[500.8,-14,145],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
TASK PERS tooldata toolshieldcan:=[TRUE,[[-156.443,4.27817,210.91],[1,0,0,0]],[1,[-145,-9.34,218.485],[1,0,0,0],0,0,0]];
TASK PERS tooldata sidd:=[TRUE,[[-144.459,-7.69289,215.816],[1,0,0,0]],[-1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS wobjdata wobjshieldcan:=[FALSE,TRUE,"",[[675.4,214.8,189.1],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
TASK PERS tooldata nishanttest:=[TRUE,[[-81.401,2.516,159.598],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata calibTool:=[TRUE,[[-0.228667,0.269561,182.166],[1,0,0,0]],[0.2,[0,0,0],[1,0,0,0],0,0,0]];
CONST robtarget calib1:=[[599.35,-573.63,1.21],[0.000618842,-0.00183075,0.951211,-0.308535],[-1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib2:=[[461.90,-573.12,1.11],[0.000622551,-0.0018337,0.951211,-0.308534],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib3:=[[331.04,-572.22,1.01],[0.0006239,-0.00182736,0.951213,-0.30853],[-1,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib4:=[[213.90,-571.36,0.65],[0.000623773,-0.00182709,0.951213,-0.30853],[-1,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib5:=[[74.79,-571.08,0.44],[0.000624911,-0.00182578,0.951212,-0.308531],[-1,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib6:=[[-71.53,-570.16,0.39],[0.000627229,-0.00182302,0.951212,-0.308531],[-2,-3,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib7:=[[633.01,41.56,335.98],[0.449198,-0.63276,-0.615056,-0.139795],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib8:=[[496.48,278.30,375.38],[0.357565,0.832103,0.419119,0.0639613],[0,-1,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib9:=[[433.50,177.84,365.22],[0.112886,0.870163,0.299071,0.375007],[0,-2,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget curpos:=[[612.07,11.89,243.18],[0.633169,-0.720998,-0.0564776,0.275807],[-1,0,-3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[808.5,-612.86,0.59],[0.7084,0.0003882,-0.0003882,0.7058]]];
TASK PERS wobjdata wobjCelllPhone:=[FALSE,TRUE,"",[[491.9,-69.4,152.3],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
TASK PERS tooldata tool2:=[TRUE,[[0.868263,-1.61762,162.034],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS wobjdata locatorWobj:=[FALSE,TRUE,"",[[608,11.9,99.4],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

!// For IK. If getting no solution for an IK, check the configuration of the robot, and add it to this list
CONST num num_common_cfgs := 2;
CONST confdata common_configs{num_common_cfgs} := [[0,0,0,0], [0,0,-2,1]];

!////////////////
!LOCAL METHODS
!////////////////

!Method to parse the message received from a PC through the socket
! Loads values on:
! - instructionCode.
! - idCode: 3 digit identifier of the command. 
! - nParams: Number of received parameters.
! - params{nParams}: Vector of received params.
PROC ParseMsg(string msg)
	!Local variables
	VAR bool auxOk;
	VAR num ind:=1;
	VAR num newInd;
	VAR num length;
	VAR string subString;
    
    length := StrLen(msg);
    
    TPWrite msg;
    
    ! If this is the beginning of a message, we'll get our instruction code and id code first
    IF messageState = MESSAGE_NONE THEN
    
    	!Find Instruction code
    	newInd := StrMatch(msg,ind," ");
        ! Our instruction code should take up 79 characters or less (string is max 80 characters).
        ! If we couldn't find a space, that means our instruction code is far too long. Bad message.
        IF newInd > length THEN
            messageState := MESSAGE_BAD;
            RETURN;
        ENDIF
        
        ! Get the substring and save the instruction code
    	subString := StrPart(msg,ind,newInd - ind);
    	auxOk:= StrToVal(subString, instructionCode);
        IF auxOk = FALSE THEN
            ! If we couldn't convert the substring to our instruction code, then something is wrong. Bad message.
            messageState := MESSAGE_BAD;
            RETURN;
        ENDIF
        
        ! Skip over the space, and onto the id code
        ind := newInd + 1;
        
        !Find id code
    	newInd := StrMatch(msg,ind," ");
        ! The number of characters used by our id code and instruction code should be less than or equal to 78. If not, bad message
        IF newInd > length THEN
            messageState := MESSAGE_BAD;
            RETURN;
        ENDIF
        
        ! Get the substring and save the id code
    	idCode := StrPart(msg,ind,newInd - ind);
      
        ! Skip over the space, and onto the parameters
        ind := newInd + 1;
    ENDIF
    
    ! Now onto the parameters
    WHILE TRUE DO
        ! Check if this is the end character
        IF StrMemb(msg, ind, "#") THEN
            ! If so, we're done with our message!
            messageState := MESSAGE_COMPLETE;
            RETURN;
        ENDIF
        
        ! Otherwise, let's get our parameter
        newInd := StrMatch(msg, ind, " ");
        
        ! If we couldn't find a space, our message continues...
        IF newInd > length THEN
            messageState := MESSAGE_MIDDLE;
            remainingString := StrPart(msg, ind, newInd - ind);
            remainingBytes := 80 - StrLen(remainingString);
            
            ! If we are not ready to read any more bytes, then this is a very bad message that has no spaces for 80 characters.
            IF remainingBytes = 0 THEN
                messageState := MESSAGE_BAD;
            ENDIF
            
            RETURN;
        ENDIF
        
        ! If there are too many parameters, this is a bad message
        IF nParams >= MAX_PARAMS THEN
            messageState := MESSAGE_BAD;
            RETURN;
        ENDIF
        
        nParams := nParams + 1;
        
        subString := StrPart(msg,ind,newInd - ind);
    	auxOk:= StrToVal(subString, params{nParams});
        IF auxOk = FALSE THEN
            ! If we couldn't convert the substring to our parameter then something is wrong. Bad message.
            messageState := MESSAGE_BAD;
            RETURN;
        ENDIF
        
        ind := newInd + 1;
    ENDWHILE
ENDPROC

!Handshake between server and client:
! - Creates socket.
! - Waits for incoming TCP connection.
PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	TPWrite "SERVER: Server waiting for incomming connections ...";
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
		IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
			TPWrite "SERVER: Problem serving an incomming connection.";
			TPWrite "SERVER: Try reconnecting.";
		ENDIF
		 !//Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "SERVER: Connected to IP " + clientIP;
	
    ! Initialize our socket receiving logic
    remainingBytes := 80;
    remainingString := "";
    messageState := MESSAGE_NONE;
    nParams := 0;
    
ENDPROC

!//Parameter initialization
!// Loads default values for
!// - Tool.
!// - WorkObject.
!// - Zone.
!// - Speed.
!// Gets joint bounds so they can be used later
PROC Initialize()
	VAR string path;

    !// NOTE: We set the current tool to have a mass of 0.8kg and center of mass at 40mm
	currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.8,[0,0,40],[1,0,0,0],0,0,0]];    
	currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
	currentSpeed := [100, 50, 5000, 1000];
	currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
    SetDO vacuum,0;
	SetDO solenoid_passthrough,0;
	
    ! trajectory length starts at 0
    trajLength := 0;
    trajOk := FALSE;
    
	!// Get all of the joint bounds for later use
	FOR i FROM 1 TO 6 DO
		path := "MOC/ARM/rob1_" + NumToStr(i,0);
		ReadCfgData path, "upper_joint_bound", upper_joint_limits{i};
		ReadCfgData path, "lower_joint_bound", lower_joint_limits{i};
		
		!// The joint limits are in radians, so convert these to degrees
		upper_joint_limits{i} := upper_joint_limits{i} * 180.0 / pi;
		lower_joint_limits{i} := lower_joint_limits{i} * 180.0 / pi;
	ENDFOR

ENDPROC

PROC main()
ENDPROC

!/////////////////
!//Main procedure
!/////////////////
PROC main2()
	!//Local variables
	VAR string receivedString;
	VAR string sendString;
	VAR string addString;
	VAR bool connected;  ! //Client connected
	VAR bool reconnected;! //Reconnection During the iteration
	VAR robtarget cartesianPose;
	VAR jointtarget jointsPose;
	VAR clock timer;
	VAR num quatMag;
    VAR confdata curConf;
    VAR num jointMag;
    VAR bool ik_solution_found;
    VAR num minMag;
    VAR jointtarget best_ik_result_j;
	
	!//Book error number for error handler
	BookErrNo ERR_MOTIONSUP;
	
	!//Configuration of interrupt that traps digital output that signals the need to restart the motion of the robot.
	!//SetDO USER_RESET_MOTION, 0;
    SetDO USER_START_OUTPUT, 0;
    SetDO USER_RESET_MOTION, 0;
	CONNECT iMotionReset WITH resetMotion;
	ISignalDO USER_RESET_MOTION,1,iMotionReset;
    
    !// Set up a interrupt to occur when the collision sensor is triggered
    CONNECT iCollisionSensor WITH colSensorReset;
    ISignalDI Collision_Sensor, 0, iCollisionSensor;
	
	!// We are not currently changing the frame
	frameMutex:= FALSE;
    
    !// Set up Motion Supervision by editing the configuration
    !// Note that we do this because if motion Supervision is disabled manually 
    !// in the configuration parameters, it cannot be turned on by MotionSup
    WriteCfgData "MOC/MOTION_SUP/rob1","path_col_detect_on",TRUE;
    WriteCfgData "MOC/MOTION_SUP/rob1","jog_col_detect_on",TRUE;
    WriteCfgData "MOC/MOTION_SUP/rob1", "path_col_detect_level", 80;
    WriteCfgData "MOC/MOTION_SUP/rob1", "jog_col_detect_level", 80;
    WriteCfgData "MOC/MOTION_SUP/rob1","collision_detection_memory",0.1;

	!//Motion configuration
	!MotionSup \On \TuneValue:=40;
	ConfL \Off;
	ConfJ \Off;
	!SingArea \Wrist;
	SingArea \Off;
	moveComplete:= TRUE;
	
	!//Timer synchronization with Logger
	startRob:=TRUE;
	WaitUntil startLog \PollRate:=0.01;
	ClkStart timer;

	!//Initialization of WorkObject, Tool, Speed and Zone, 
	!// and get the joint bounds from the current configuration
	Initialize;
    

	
 	!//Socket connection
	connected:=FALSE;
	ServerCreateAndConnect ipController,serverPort;	
	connected:=TRUE;
	
	!//Infinite Loop
	WHILE TRUE DO
	    !//Initialization of program flow variables
		ok:=SERVER_OK;              !//Correctness of executed instruction.
		reconnected:=FALSE;         !//Has communication dropped after receiving a command?
		addString := "";    		!//String to add to the reply.

		!//Wait for a command
		!// 'receivedString' can only hold at most 80 characters. If there was anything remaining from 
        !// a previous message, make sure we concatenate that, and only read characters from the socket 
        !// until our concatenated string is 80 characters.
        SocketReceive clientSocket \Str:=receivedString \ReadNoOfBytes:=remainingBytes \Time:=WAIT_MAX;
        receivedString := remainingString + receivedString;
        ParseMsg receivedString;
        
        !// If after parsing the string we have a complete message, then let's execute it
        IF messageState = MESSAGE_COMPLETE THEN
            TPWrite "Num Params: " \Num:=nParams;
            TPWrite "Instruction: " \Num:=instructionCode;
            TPWrite "ID Code: " + idCode;
            
            IF nParams > 0 THEN
                addString := "";
                
                FOR i FROM 1 TO nParams DO
                    addString := addString + NumToStr(params{i},2) + " ";
                ENDFOR
                
                TPWrite addString;
                
                addString := "";
            ENDIF
            
    		!//Execution of the command
    		TEST instructionCode
    		CASE 0: !Ping
    			!Message Check
    			IF nParams = 0 THEN
    				ok := SERVER_OK;
    			ELSE
    				ok := SERVER_BAD_MSG;
    			ENDIF
    		CASE 1: !Set Cartesian Coordinates
    			IF nParams = 7 THEN
    				cartesianTarget :=[[params{1},params{2},params{3}],
    						[params{4},params{5},params{6},params{7}],
    						[0,0,0,0],
    						[9E9,9E9,9E9,9E9,9E9,9E9]];
    				ok := SERVER_OK;
    				moveComplete := FALSE;
                    SetDO Moving,1;
    				MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    SetDO Moving,0;
    				moveComplete := TRUE;
    			ELSEIF nParams = 8 THEN
    				!If there's an extra parameter, do a cartesian move as a joint move instead
    				cartesianTarget :=[[params{1},params{2},params{3}],
    						[params{4},params{5},params{6},params{7}],
    						[0,0,0,0],
    						[9E9,9E9,9E9,9E9,9E9,9E9]];
    				ok := SERVER_OK;
    				moveComplete := FALSE;
                    SetDO Moving,1;
    				MoveJ cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    SetDO Moving,0;
    				moveComplete := TRUE;
    			ELSE
    				ok := SERVER_BAD_MSG;
    			ENDIF	
    		CASE 2: !Set Joint Coordinates
    			IF nParams = 6 THEN
    				jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
    							[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
    				ok := SERVER_OK;
    				moveComplete := FALSE;
                    SetDO Moving,1;
    				MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    SetDO Moving,0;
    				moveComplete := TRUE;
    			ELSE
    				ok :=SERVER_BAD_MSG;
    			ENDIF
    		CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
    			IF nParams = 0 THEN
    				cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);
    				addString := NumToStr(cartesianPose.trans.x,2) + " ";
    				addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
    				addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
    				addString := addString + NumToStr(cartesianPose.rot.q1,4) + " ";
    				addString := addString + NumToStr(cartesianPose.rot.q2,4) + " ";
    				addString := addString + NumToStr(cartesianPose.rot.q3,4) + " ";
    				addString := addString + NumToStr(cartesianPose.rot.q4,4); !End of string	
    				ok := SERVER_OK;
    			ELSE
    				ok :=SERVER_BAD_MSG;
    			ENDIF
    		CASE 4: !Get Joint Coordinates
    			IF nParams = 0 THEN
    				jointsPose := CJointT();
    				addString := NumToStr(jointsPose.robax.rax_1,2) + " ";
    				addString := addString + NumToStr(jointsPose.robax.rax_2,2) + " ";
    				addString := addString + NumToStr(jointsPose.robax.rax_3,2) + " ";
    				addString := addString + NumToStr(jointsPose.robax.rax_4,2) + " ";
    				addString := addString + NumToStr(jointsPose.robax.rax_5,2) + " ";
    				addString := addString + NumToStr(jointsPose.robax.rax_6,2); !End of string
    				ok := SERVER_OK;
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
    		CASE 6: !Specify Tool
    			IF nParams = 7 THEN
    				WHILE (frameMutex) DO
    					!// If the frame is being used by logger, wait here
    				ENDWHILE
    				frameMutex:= TRUE;
    				currentTool.tframe.trans.x:=params{1};
    				currentTool.tframe.trans.y:=params{2};
    				currentTool.tframe.trans.z:=params{3};
    				currentTool.tframe.rot.q1:=params{4};
    				currentTool.tframe.rot.q2:=params{5};
    				currentTool.tframe.rot.q3:=params{6};
    				currentTool.tframe.rot.q4:=params{7};
    				frameMutex:= FALSE;
    				ok := SERVER_OK;
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
    		CASE 7: !Specify Work Object
    			IF nParams = 7 THEN
    				WHILE (frameMutex) DO
    					!// If the frame is being used by logger, wait here
    				ENDWHILE
    				frameMutex:= TRUE;
    				currentWobj.oframe.trans.x:=params{1};
    				currentWobj.oframe.trans.y:=params{2};
    				currentWobj.oframe.trans.z:=params{3};
    				currentWobj.oframe.rot.q1:=params{4};
    				currentWobj.oframe.rot.q2:=params{5};
    				currentWobj.oframe.rot.q3:=params{6};
    				currentWobj.oframe.rot.q4:=params{7};
    				frameMutex:= FALSE;
    				ok := SERVER_OK;
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
    		CASE 8: !Specify Speed of the Robot
    			IF nParams = 2 THEN
    				currentSpeed.v_tcp:=params{1};
    				currentSpeed.v_ori:=params{2};
    				ok := SERVER_OK;
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
    		CASE 9: !Specify ZoneData
    			IF nParams = 4 THEN
    				IF params{1}=1 THEN
    					currentZone.finep := TRUE;
    					currentZone.pzone_tcp := 0.0;
    					currentZone.pzone_ori := 0.0;
    					currentZone.zone_ori := 0.0;
    				ELSE
    					currentZone.finep := FALSE;
    					currentZone.pzone_tcp := params{2};
    					currentZone.pzone_ori := params{3};
    					currentZone.zone_ori := params{4};
    				ENDIF
    				ok := SERVER_OK;
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
    		CASE 10: !Special Command
    			IF nParams = 6 THEN
    			    specialParams{1} := params{2};
    				specialParams{2} := params{3};
    				specialParams{3} := params{4};
    				specialParams{4} := params{5};
    				specialParams{5} := params{6};
    				TEST params{1}				
    					CASE 1:
    						!Call special command number 1: Bounceless Catch
    						Catching;
    						ok := SERVER_OK;
    					CASE 2:
                            ! Call Kruskal Counter subroutine
                            
                            WHILE (frameMutex) DO
                                !// If the frame is being used by logger, wait here
                            ENDWHILE
                            frameMutex := TRUE;
                            tempTool := currentTool;
                            tempWObj := currentWobj;
                            tempZone := currentZone;
                            tempSpeed := currentSpeed;
                            
                            currentTool := tool0;
                            currentWobj := wobj0;
                            currentZone := z0;
                            frameMutex := FALSE;

                            KruskalCounter;
                            
                            WHILE (frameMutex) DO
                                !// If the frame is being used by logger, wait here
                            ENDWHILE
                            frameMutex := TRUE;
                            currentTool := tempTool;
                            currentWobj := tempWObj;
                            currentZone := tempZone;
                            currentSpeed := tempSpeed;
                            frameMutex := FALSE;
                            
    						ok := SERVER_OK;
    					CASE 3:
    						!Call special command number 3: Enveloping Grasp to Fingertip Grasp
    						Env2Fing;
    						ok := SERVER_OK;
    					CASE 4:
    						!Call special command number 4: Vibrate
    						Vibrate;
    						ok := SERVER_OK;
    					CASE 5:
    						!Call special command number 5: Fingertip to Enveloping with hand facing downward
    						Fing2EnvHD;
    						ok := SERVER_OK;	
    					DEFAULT:
    					    ok:=SERVER_BAD_MSG;
    				ENDTEST				
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
                
            CASE 11: !Toggle vacuum on/off
    			IF nParams = 1 THEN
    				ok := SERVER_OK;
    				IF params{1}=0 THEN
    					SetDO solenoid_passthrough,0;
    				ELSEIF params{1}=1 THEN
    					SetDO solenoid_passthrough,1;
    				ELSE
    					ok:=SERVER_BAD_MSG;
    				ENDIF
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
    			
    		CASE 12: !Inverse Kinematics Solver
    			IF nParams = 7 THEN
                    cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);
                    TPWrite "CurPos = " \Pos:=cartesianPose.trans;
                    TPWrite "CurQuat = " \Orient:=cartesianPose.rot;
                    curConf := cartesianPose.robconf;
                    TPWrite "CurConf = " \Orient:=[curConf.cf1, curConf.cf4, curConf.cf6, curConf.cfx];
                    TPWrite "Pos = " \Pos:= [params{1},params{2},params{3}];
                    TPWrite "Quat = " \Orient:= [params{4},params{5},params{6},params{7}];
    				!// First, let's make sure the quaternion is normalized
    				IF Abs(1.0 - Sqrt(params{4} * params{4} + params{5} * params{5} + params{6} * params{6} + params{7} * params{7})) > 0.001 THEN
    					!// If not, then we cannot find the inverse kinematics for this pose
    					ok := SERVER_BAD_IK;
    				ELSE
                        jointsPose := CJointT();
                        
                        !// Here's our nominal cartesian target from the user
    					cartesianTarget :=[[params{1},params{2},params{3}],
    							[params{4},params{5},params{6},params{7}],
    							[0,0,0,0],
    							[9E9,9E9,9E9,9E9,9E9,9E9]];
                                

                        ik_solution_found := FALSE;
                        minMag := 20000000; !// Larger than 6 * (720 - -720)^2
                        best_ik_result_j := [[0,0,0,0,0,0], [ 0,9E9,9E9,9E9,9E9,9E9]];
                        
                        FOR i FROM 1 TO num_common_cfgs DO
                            ok := SERVER_OK;
                            cartesianTarget.robconf := common_configs{i};
                            !// Now calculate the joint angles, keeping in mind that if we specified an 
        					!// impossible configuration, this will generate an error (See error handler below)
        					ik_result_j := CalcJointT(cartesianTarget, currentTool, \WObj:=currentWObj);
                                      
                            IF ok = SERVER_OK THEN
                                !// If there was no error, check to see if this is the closest joint angle to the current configuration
                                jointMag := (ik_result_j.robax.rax_1 - jointsPose.robax.rax_1) * (ik_result_j.robax.rax_1 - jointsPose.robax.rax_1)
                                + (ik_result_j.robax.rax_2 - jointsPose.robax.rax_2) * (ik_result_j.robax.rax_2 - jointsPose.robax.rax_2)
                                + (ik_result_j.robax.rax_3 - jointsPose.robax.rax_3) * (ik_result_j.robax.rax_3 - jointsPose.robax.rax_3)
                                + (ik_result_j.robax.rax_4 - jointsPose.robax.rax_4) * (ik_result_j.robax.rax_4 - jointsPose.robax.rax_4)
                                + (ik_result_j.robax.rax_5 - jointsPose.robax.rax_5) * (ik_result_j.robax.rax_5 - jointsPose.robax.rax_5)
                                + (ik_result_j.robax.rax_6 - jointsPose.robax.rax_6) * (ik_result_j.robax.rax_6 - jointsPose.robax.rax_6); 
                                
                                !// If it's the closest so far, save it!
                                IF jointMag < minMag THEN
                                    best_ik_result_j := ik_result_j;
                                    minMag := jointMag;
                                    ik_solution_found := TRUE;
                                ENDIF
                            ENDIF
                        ENDFOR
                        
                        !//!// Because the inverse kinematics solver function in RAPID requires us to specify 
                        !//!// a configuration, let's iterate through all possible configurations and 
                        !//!// check if one of them is valid. Every configuration is 90deg
                        !//!// Joint1: -180 to 180, so -2, -1, 0, 1
                        !//!// Joint4: -200 to 200, so -3, -2, -1, 0, 1, 2
                        !//!// Joint6: -400 to 400, so -5, -4, -3, -2, -1, 0, 1, 2, 3, 4
                        !//!// In all we'll check 240 configurations!
                        !//FOR c1 FROM -2 TO 1 STEP 1 DO
                        !//    FOR c4 FROM -3 TO 2 STEP 1 DO
                        !//        FOR c6 FROM -5 TO 4 STEP 1 DO
                        !//            FOR cx FROM 0 TO 7 STEP 1 DO
                        !//                
                        !//                ok := SERVER_OK;
                        !//                cartesianTarget.robconf := [c1,c4,c6,cx];
                        !//                
                        !//                !// Now calculate the joint angles, keeping in mind that if we specified an 
        				!//	            !// impossible configuration, this will generate an error (See error handler below)
        				!//	            ik_result_j := CalcJointT(cartesianTarget, currentTool, \WObj:=currentWObj);
                        !//                
                        !//                IF ok = SERVER_OK THEN
                        !//                    !// If there was no error, check to see if this is the closest joint angle to the current configuration
                        !//                    jointMag := (ik_result_j.robax.rax_1 - jointsPose.robax.rax_1) * (ik_result_j.robax.rax_1 - jointsPose.robax.rax_1)
                        !//                    + (ik_result_j.robax.rax_2 - jointsPose.robax.rax_2) * (ik_result_j.robax.rax_2 - jointsPose.robax.rax_2)
                        !//                    + (ik_result_j.robax.rax_3 - jointsPose.robax.rax_3) * (ik_result_j.robax.rax_3 - jointsPose.robax.rax_3)
                        !//                    + (ik_result_j.robax.rax_4 - jointsPose.robax.rax_4) * (ik_result_j.robax.rax_4 - jointsPose.robax.rax_4)
                        !//                    + (ik_result_j.robax.rax_5 - jointsPose.robax.rax_5) * (ik_result_j.robax.rax_5 - jointsPose.robax.rax_5)
                        !//                    + (ik_result_j.robax.rax_6 - jointsPose.robax.rax_6) * (ik_result_j.robax.rax_6 - jointsPose.robax.rax_6); 
                        !//                    
                        !//                    !// If it's the closest so far, save it!
                        !//                    IF jointMag < minMag THEN
                        !//                        best_ik_result_j := ik_result_j;
                        !//                        minMag := jointMag;
                        !//                        ik_solution_found := TRUE;
                        !//                    ENDIF
                        !//                ENDIF
                        !//            ENDFOR
                        !//        ENDFOR
                        !//    ENDFOR
                        !//ENDFOR
                        
                        !// If we found at least 1 good solution, then return success
    					IF ik_solution_found THEN
                            ok := SERVER_OK;
                        ELSE
                            ok := SERVER_BAD_IK;
                        ENDIF

    					!// Store our result in a string to return to the user
    					addString := NumToStr(best_ik_result_j.robax.rax_1,2) + " ";
    					addString := addString + NumToStr(best_ik_result_j.robax.rax_2,2) + " ";
    					addString := addString + NumToStr(best_ik_result_j.robax.rax_3,2) + " ";
    					addString := addString + NumToStr(best_ik_result_j.robax.rax_4,2) + " ";
    					addString := addString + NumToStr(best_ik_result_j.robax.rax_5,2) + " ";
    					addString := addString + NumToStr(best_ik_result_j.robax.rax_6,2); !End of string
    				
    				ENDIF
    			ELSE
    				ok:=SERVER_BAD_MSG;
    			ENDIF
    			
    		CASE 13: ! Forward Kinematics Solver
    			IF nParams = 6 THEN
    				ok := SERVER_OK;
    				
    				!// First, let's make sure the specified joint angles are within range
    				FOR i FROM 1 TO 6 DO
    					IF params{i} > upper_joint_limits{i} OR params{i} < lower_joint_limits{i} THEN
    						!// If not, then we'll tell the user that their forward kinematics are invalid
    						ok := SERVER_BAD_FK;
    					ENDIF
    				ENDFOR
    					
    				!// If our joints are within limits, then let's carry on
    				IF ok = SERVER_OK THEN
    					!// Create a joint target, and then calculate the corresponding cartesian pose
    					jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
    								[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
    					fk_result_c := CalcRobT(jointsTarget, currentTool, \WObj:=currentWObj);
    					
    					!// Now add this pose to our return string
    					addString := NumToStr(fk_result_c.trans.x,2) + " ";
    					addString := addString + NumToStr(fk_result_c.trans.y,2) + " ";
    					addString := addString + NumToStr(fk_result_c.trans.z,2) + " ";
    					addString := addString + NumToStr(fk_result_c.rot.q1,4) + " ";
    					addString := addString + NumToStr(fk_result_c.rot.q2,4) + " ";
    					addString := addString + NumToStr(fk_result_c.rot.q3,4) + " ";
    					addString := addString + NumToStr(fk_result_c.rot.q4,4); !End of string
    				ENDIF
    			ELSE
    				ok := SERVER_BAD_MSG;
    			ENDIF
    			
                
            CASE 20: !Add Trajectory
                ! Check to make sure we have the right number of parameters
            	IF nParams = 12 THEN
                    ! Next, check that our trajectory buffer is not full
                    IF trajLength < MAX_TRAJ_LENGTH THEN
                        ! If there's a slot in our trajectory, let's add the new position
                        trajLength := trajLength + 1;
                        
                        ! Add the cartesian target (Note that this will use the curTool and curWobj when it is actually executed
                        trajPt{trajLength} := [[params{1},params{2},params{3}],[params{4},params{5},params{6},params{7}],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
                        
                        ! Specify the time it should take to execute in seconds
                        trajTime{trajLength} := params{8};
    				    
                        ! Now, figure out the zone to use
                        IF params{9}=1 THEN
                            ! If a fine zone is requested, do that
    					    trajZone{trajLength}.finep := TRUE;
    					    trajZone{trajLength}.pzone_tcp := 0.0;
    					    trajZone{trajLength}.pzone_ori := 0.0;
    					    trajZone{trajLength}.zone_ori := 0.0;
    				    ELSE
                            ! Otherwise setup the desired zone parameters
    					    trajZone{trajLength}.finep := FALSE;
    					    trajZone{trajLength}.pzone_tcp := params{10};
    					    trajZone{trajLength}.pzone_ori := params{11};
    					    trajZone{trajLength}.zone_ori := params{12};
    				    ENDIF
                        
                        ! We're done, and have successfully added a trajectory point
                        ok := SERVER_OK;
                    ELSE
                        ! If we don't have a slot, return an error
                        ok := SERVER_TRAJ_TOO_LONG;
                    ENDIF
                ELSE
                    ! If the incorrect number of parameters have been sent, return an error
                    ok := SERVER_BAD_MSG;
                ENDIF
                
            CASE 21: !Execute trajectory
                ok := SERVER_OK;
    			moveComplete := FALSE;
                idx := 1;
                trajOk := TRUE;
                SetDO Moving,1;
                WHILE idx <= trajLength AND trajOk DO
                    MoveL trajPt{idx}, currentSpeed \T:=trajTime{idx}, trajZone{idx}, currentTool \WObj:=currentWobj;
                    idx := idx + 1;
                ENDWHILE
                SetDO Moving,0;
                moveComplete := TRUE;
                trajLength := 0;
                trajOk := FALSE;
                
            CASE 22: ! Clear Trajectory
                ok := SERVER_OK;
                trajLength := 0; ! Simply reset the buffer size counter and we're done!
    			
    		CASE 99: !Close Connection
    			IF nParams = 0 THEN
    				TPWrite "SERVER: Client has closed connection.";
    				connected := FALSE;
    				!Closing the server
    				SocketClose clientSocket;
    				SocketClose serverSocket;

    				!Reinitiate the server
    				ServerCreateAndConnect ipController,serverPort;
    				connected := TRUE;
    				reconnected := TRUE;
    				ok := SERVER_OK;
    			ELSE
    				ok := SERVER_BAD_MSG;
    			ENDIF
    		DEFAULT:
    			TPWrite "SERVER: Illegal instruction code";
    			ok := SERVER_BAD_MSG;
    		ENDTEST
    		
    		!Finally we compose the acknowledge string to send back to the client
    		IF connected = TRUE THEN
    			IF reconnected = FALSE THEN
    				sendString := NumToStr(instructionCode,0);
    				sendString := sendString + " " + idCode;
    				sendString := sendString + " " + NumToStr(ok,0);
    				sendString := sendString + " " + NumToStr(ClkRead(timer),2);
    				sendString := sendString + " " + addString + ByteToStr(10\Char);
    				SocketSend clientSocket \Str:=sendString;
    			ENDIF
    		ENDIF
            
            !// Now that we have executed this message, let's reset 
            !// the message state so we can read another
            messageState := MESSAGE_NONE;
            remainingBytes := 80;
            remainingString := "";
            nParams := 0;
            
        !// If our message state is that we received a bad message, 
        !// let's notify the sender and reset
        ELSEIF messageState = MESSAGE_BAD THEN
            IF connected = TRUE THEN
    			IF reconnected = FALSE THEN
    				sendString := NumToStr(instructionCode,0);
    				sendString := sendString + " " + idCode;
    				sendString := sendString + " " + NumToStr(SERVER_BAD_MSG,0);
    				sendString := sendString + " " + NumToStr(ClkRead(timer),2);
    				sendString := sendString + " " + addString + ByteToStr(10\Char);
    				SocketSend clientSocket \Str:=sendString;
    			ENDIF
    		ENDIF
            
            !// Now that we've notified, reset our message state
            messageState := MESSAGE_NONE;
            remainingBytes := 80;
            remainingString := "";
            nParams := 0;
        ENDIF
            
	ENDWHILE
ERROR (LONG_JMP_ALL_ERR)
    TEST ERRNO
		CASE ERR_MOTIONSUP:
            TPWrite "SERVER: ------";
			TPWrite "SERVER: Moton suppervision error.";
			!//Stop the robot motion
			StopMove;

			!//We clear the current path, to remove any residual motions in the path queue.
			ClearPath;
            
            !// If we were executing a sequence of moves, let's stop that
            trajOk := FALSE;
	
			!//Set the target pose of the object to the current location of the robot
			!//When we retry the execution of the program, it will do a MoveL instruction to that target.
			cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
			jointsTarget := CJointT();
	
			!//Enable the motion of the robot 
			StartMove;

			TPWrite "SERVER: Recovered.";
			TPWrite "SERVER: ------";
    		!//Retry execution of the program.
			RETRY;
		CASE ERR_SOCK_CLOSED:
            TPWrite "SERVER: ------";
			TPWrite "SERVER: Client has closed connection.";
			TPWrite "SERVER: Closing socket and restarting.";
			TPWrite "SERVER: ------";
    		connected:=FALSE;
			!//Closing the server
			SocketClose clientSocket;
			SocketClose serverSocket;
			!//Reinitiate the server
			ServerCreateAndConnect ipController,serverPort;
			reconnected:= TRUE;
			connected:= TRUE;
			RETRY; 
		
		CASE ERR_ROBLIMIT:
			!// We get here if we have tried to a cartesian position and realized it is unattainable.
			!// Simply set our 'ok' message to be a bad IK, and set the resulting joint values to 0
			ok := SERVER_BAD_IK;
			ik_result_j := [[0,0,0,0,0,0],[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
			
			!// We'll skip the instruction that caused the error, which was the thing 
			!//trying to calculate the inverse kinematics
			TRYNEXT;
			
		DEFAULT:
            TPWrite "SERVER: ------";
            TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO,0);
			TPWrite "SERVER: Unknown error.";
			TPWrite "SERVER: Closing socket and restarting.";
			TPWrite "SERVER: ------";
    		connected:=FALSE;
			!//Closing the server
			SocketClose clientSocket;
			SocketClose serverSocket;
			!//Reinitiate the server
			ServerCreateAndConnect ipController,serverPort;
			reconnected:= TRUE;
			connected:= TRUE;
			RETRY;
	ENDTEST
ENDPROC

TRAP resetMotion
    !//Routine triggered when the digital output USER_RESET_MOTION is set to 1.
	!//It signals the need to restart the motion of the robot.
	
	!//We set this two digital outputs back to 0, in preparation for the next time there is a collision
	SetDO USER_START_OUTPUT, 0;
    SetDO USER_RESET_MOTION, 0;
	
	!//Note that the motion encoutered a collision 
	ok:= SERVER_COLLISION;
	
	IF moveComplete = TRUE THEN
		!//If the move instruction is complete, there is no need to raise an error. 
		!//Just continue with normal program execution.
		TPWrite "SERVER: ------";
		TPWrite "SERVER: Motion suppervision error after";
		TPWrite "SERVER: move instruction completed.";
	
		!Stop the robot motion
		StopMove;

		!We clear the current path, to remove any residual motions in the path queue.
		ClearPath;
		
		!Restart robot motion execution.
		StartMove;
		TPWrite "SERVER: Recovered.";
		TPWrite "SERVER: ------";
   	ELSE
		!//We signal the restart of the robot motion by raising the ERR_MOTIONSUP error.
		!//It will be handled by the error handler in the main procedure.
		RAISE ERR_MOTIONSUP;
	ENDIF			
ERROR
	RAISE;
ENDTRAP

TRAP colSensorReset
    !//Routine triggered when the collision sensor is set to 0.
	!//It signals the need to restart the motion of the robot.
	
	!//Note that the motion encoutered a collision 
	ok:= SERVER_COLLISION;
	
	IF moveComplete = TRUE THEN
		!//If the move instruction is complete, there is no need to raise an error. 
		!//Just continue with normal program execution.
		TPWrite "SERVER: ------";
		TPWrite "SERVER: Collision sensor error after";
		TPWrite "SERVER: move instruction completed.";
	
		!Stop the robot motion
		StopMove;

		!We clear the current path, to remove any residual motions in the path queue.
		ClearPath;
		
		!Restart robot motion execution.
		StartMove;
		TPWrite "SERVER: Recovered.";
		TPWrite "SERVER: ------";
   	ELSE
		!//We signal the restart of the robot motion by raising the ERR_MOTIONSUP error.
		!//It will be handled by the error handler in the main procedure.
		RAISE ERR_MOTIONSUP;
	ENDIF			
ERROR
	RAISE;
ENDTRAP

ENDMODULE