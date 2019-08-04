MODULE EGM_test_UDP
    
    VAR egmident egmID1;
    VAR egmstate egmSt1;
    
    CONST egm_minmax egm_minmax_lin1:=[-0.1,0.1]; !in mm
    CONST egm_minmax egm_minmax_rot1:=[-0.1,0.1];! in degees

    TASK PERS wobjdata currentWobj:= [ FALSE, TRUE, "", [ [0, 0, 0], [1, 0, 0 ,0] ], [ [0, 0, 0], [1, 0, 0 ,0] ] ];
    
    PERS tooldata Tool_ZZJHand:=[TRUE,[[4,-1.6,160.3],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    PERS tooldata Tool_tc_vacuum:=[TRUE,[[0,0,130],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]]; ! 05/11/2019
    PERS tooldata Tool_Probe:=[TRUE,[[0,0,134.95],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    
    VAR pose posecorEGM:=[[0,0,0],[1,0,0,0]];
    VAR pose posesenEGM:=[[0,0,0],[1,0,0,0]];
    
    ! CONST jointtarget jointsTarget_regrasp:=[[-75, 10, -10, 0, 90, 90],  [ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9]]; ! Quaternion: [0 -1 0 0]
    
    ! cartesian Targets
    VAR robtarget cartesianTarget_ZZJHand:=[[10, 300, 435],
                                  [0,0,1,0],
                                  [0,0,0,0],
                                  [9E9,9E9,9E9,9E9,9E9,9E9]];
    VAR robtarget cartesianTarget_Probe:=[[10, 300, 420],
                                  [0,0.55187,-0.83393,0],
                                  [0,0,0,0],
                                  [9E9,9E9,9E9,9E9,9E9,9E9]];
    VAR robtarget cartesianTarget_Probe_In_Box:=[[30, 407, 378],
                                  [0,0.1951,0.9808,0],
                                  [0,0,0,0],
                                  [9E9,9E9,9E9,9E9,9E9,9E9]];
    VAR robtarget cTarget_ct_vacuum:=[[30, 320, 430],
                                  [0,1,0,0],
                                  [0,0,0,0],
                                  [9E9,9E9,9E9,9E9,9E9,9E9]];
    
    PROC main2()
    ENDPROC
    
    PROC main()
        ConfJ\Off;
        !SingArea\Off;
        !MoveAbsJ jointsTarget_regrasp, v20, fine, currentTool_ZZJHand \Wobj:=currentWobj;
        !ConfL\Off;
        !MoveL cartesianTarget_ZZJHand, v30, fine, Tool_ZZJHand \Wobj:=currentWobj;
        MoveL cTarget_ct_vacuum, v30, fine, Tool_tc_vacuum \Wobj:=currentWobj;
        !ConfL\On;
        
        WaitTime 0.5;
        
        !EXIT;
		EGMReset egmID1; ! to EGM_STATE_DISCONNECTED
        ! -----------------------------------------------------------------------
        !       EGM_STATE_DISCONNECTED
        ! -----------------------------------------------------------------------
		EGMGetId egmID1; 
		egmSt1 := EGMGetState(egmID1);
		TPWrite "EGM state: "\Num := egmSt1;
		
		IF egmSt1 <= EGM_STATE_CONNECTED THEN
			! Set up the EGM data source: UdpUc server using device "EGMsensor:"and configuration "default"
			EGMSetupUC ROB_1, egmID1, "push", "EGMSensor" \pose;  ! to EGM_STATE_CONNECTED
            TPWrite "EGM connected";
		ENDIF
		
        ! -----------------------------------------------------------------------
        !       EGM_STATE_CONNECTED
        ! -----------------------------------------------------------------------
        
        ! EGMActPose defines all frames that are available in EGM		
		EGMActPose egmID1\Tool:=Tool_tc_vacuum \WObj:=currentWobj, posecorEGM,EGM_FRAME_WOBJ, posesenEGM, EGM_FRAME_WOBJ 
		\x:=egm_minmax_lin1 \y:=egm_minmax_lin1 \z:=egm_minmax_lin1
		\rx:=egm_minmax_rot1 \ry:=egm_minmax_rot1 \rz:=egm_minmax_rot1 \LpFilter:=0 \Samplerate:=4 \MaxSpeedDeviation:= 1000;
				
		EGMRunPose egmID1, EGM_STOP_HOLD \x \y \z \Rx \Ry \Rz \CondTime:=1800 \RampInTime:=0.00 \RampOutTime:=0.5;
        ! -----------------------------------------------------------------------
        !       EGM_STATE_RUNNING
        ! -----------------------------------------------------------------------
        ! when EGMRunPose() stops
        ! -----------------------------------------------------------------------
        !       EGM_STATE_CONNECTED
        ! -----------------------------------------------------------------------
        
		egmSt1:=EGMGetState(egmID1); 

		IF egmSt1 = EGM_STATE_CONNECTED THEN
			TPWrite "Reset EGM instance egmID1";
			EGMReset egmID1; 
		ENDIF 
        
		TPWrite "EGM STOPPED";
        WHILE TRUE DO
            WaitTime 0.5;
			TPWrite "Reset EGM instance egmID1";
            
        ENDWHILE
    ENDPROC		
 
ENDMODULE