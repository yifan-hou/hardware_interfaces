MODULE SPECIAL_COMMANDS


    CONST jointtarget Target_00:=[[-5.69,-62.98,45.25,0.07,-72.27,5.46],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget Target_01:=[[-5.86,-35.46,-13.37,0.1,-41.16,5.57],[9E9,9E9,9E9,9E9,9E9,9E9]];

    CONST robtarget Target_10:=[[600,700,1100],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget Target_40:=[[600,700,805],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget Target_50:=[[600,700,800],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];



    !Kruskal Counter vars
    CONST jointtarget homeJ:=[[0.0,-74.42,28.56,0.0,-44.14,-45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget startN:=[[0.0,-74.42,28.56,0.0,-44.14,-135.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget startS:=[[0.0,-74.42,28.56,0.0,-44.14,45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget northJ:=[[0.0,-74.42,28.56,0.0,-10.0,-135.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget southJ:=[[0.0,-74.42,28.56,0.0,-10.0,45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget NE:=[[0.0,-74.42,28.56,0.0,-10.0,-90.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget SW:=[[0.0,-74.42,28.56,0.0,-10.0,90.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget NW:=[[0.0,-74.42,28.56,0.0,-10.0,180.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget SE:=[[0.0,-74.42,28.56,0.0,-10.0,0.0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    VAR num callCount;

    ! SetCartesian (-100, 0, 700), (1, 0, 0, 0)
    CONST jointtarget horz:=[[0.0,-74.42,28.56,0.0,-44.14,-45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotN:=[[0.0,-74.42,28.56,0.0,-44.14,-135.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotS:=[[0.0,-74.42,28.56,0.0,-44.14,45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotW:=[[0.0,-74.42,28.56,0.0,-44.14,135.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotE:=[[0.0,-74.42,28.56,0.0,-44.14,-45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotNE:=[[0.0,-74.42,28.56,0.0,-44.14,-90.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotSW:=[[0.0,-74.42,28.56,0.0,-44.14,90.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotNW:=[[0.0,-74.42,28.56,0.0,-44.14,180.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget rotSE:=[[0.0,-74.42,28.56,0.0,-44.14,0.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltN:=[[0.0,-74.42,28.56,0.0,-10.0,-135.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltS:=[[0.0,-74.42,28.56,0.0,-10.0,45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltW:=[[0.0,-74.42,28.56,0.0,-10.0,135.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltE:=[[0.0,-74.42,28.56,0.0,-10.0,-45.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltNE:=[[0.0,-74.42,28.56,0.0,-10.0,-90.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltSW:=[[0.0,-74.42,28.56,0.0,-10.0,90.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltNW:=[[0.0,-74.42,28.56,0.0,-10.0,180.0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget tiltSE:=[[0.0,-74.42,28.56,0.0,-10.0,0.0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    PROC Catching()
        !TPWrite "Nikhil's Catching code!!";

        VAR robtarget Target_20;
        VAR robtarget Target_30;

        Target_20:=[[600,700,specialParams{1}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        Target_30:=[[600,700,specialParams{2}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

        confL\Off;
        !SingArea \Off;
        MoveAbsJ Target_01,v100,currentZone,currentTool\Wobj:=currentWobj;

        MoveL Target_10,v100,fine,currentTool\WObj:=currentWobj;
        WorldAccLim\On:=specialParams{3};
        MoveL Target_20,vmax,z5,currentTool\WObj:=currentWobj;
        WorldAccLim\On:=specialParams{4};
        MoveL Target_30,vmax,z10,currentTool\WObj:=currentWobj;
        WorldAccLim\On:=specialParams{5};
        MoveL Target_40,vmax,z0,currentTool\WObj:=currentWobj;
        MoveL Target_50,v100,fine,currentTool\WObj:=currentWobj;
        !SingArea \Wrist;

    ENDPROC

    PROC Env2Fing()
        !TPWrite "Nikhil's E2F code!!";
        VAR robtarget Target_60;
        VAR robtarget Target_70;
        VAR robtarget Target_80;

        Target_60:=[[600,700,specialParams{1}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        Target_70:=[[600,700,specialParams{2}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        Target_80:=[[600,700,specialParams{2}-10],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

        confL\Off;
        !SingArea \Off;
        MoveAbsJ Target_00,v100,currentZone,currentTool\Wobj:=currentWobj;

        !MoveL Target_10,v100,fine,currentTool\WObj:=currentWobj;
        MoveL Target_50,v100,fine,currentTool\WObj:=currentWobj;
        WorldAccLim\On:=specialParams{3};
        MoveL Target_60,vmax,z20,currentTool\WObj:=currentWobj;
        WorldAccLim\On:=specialParams{4};
        MoveL Target_70,vmax,z5,currentTool\WObj:=currentWobj;
        MoveL Target_80,v300,fine,currentTool\WObj:=currentWobj;
        !SingArea \Wrist;

    ENDPROC

    PROC Vibrate()


        VAR robtarget actualPos;
        VAR clock Vibtimer;
        TPWrite "Vibration code!!";
        actualPos:=CRobT(\Tool:=tool0\WObj:=wobj0);
        ClkReset Vibtimer;
        ClkStart Vibtimer;
        !SingArea \Off;
        WHILE ClkRead(Vibtimer)<specialParams{1} DO
            MoveL Offs(actualPos,0,-1,0),vmax,z0,tool0\WObj:=wobj0;
            MoveL Offs(actualPos,-0.7071,-1.7071,0),vmax,z0,tool0\WObj:=wobj0;
            MoveL Offs(actualPos,0,-2.4142,0),vmax,z0,tool0\WObj:=wobj0;
            MoveL Offs(actualPos,0,-1.4142,0),vmax,z0,tool0\WObj:=wobj0;
            MoveL Offs(actualPos,0.7071,-0.7071,0),vmax,z0,tool0\WObj:=wobj0;
            MoveL actualPos,vmax,z0,tool0\WObj:=wobj0;

        ENDWHILE
        ClkStop Vibtimer;
        MoveL actualPos,vmax,fine,tool0\WObj:=wobj0;
        !SingArea \Wrist;

    ENDPROC

    PROC Fing2EnvHD()

        VAR robtarget Target_510;
        VAR robtarget Target_520;
        VAR robtarget Target_530;
        VAR robtarget Target_540;
        VAR jointtarget Target_550;

        Target_510:=[[600,200,specialParams{1}],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        Target_520:=[[600,200,specialParams{2}],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        Target_530:=[[600,200,specialParams{3}],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        Target_540:=[[600,200,specialParams{3}-30],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        Target_550:=[[-1.17,48.85,-4.08,0.09,45.23,-1.03],[9E9,9E9,9E9,9E9,9E9,9E9]];

        TPWrite "F2EHD code!!";

        confL\Off;
        !SingArea \Off;
        !MoveAbsJ Target_550, v100, currentZone, currentTool \Wobj:=currentWobj;

        !MoveL Target_10,v100,fine,currentTool\WObj:=currentWobj;
        MoveL Target_510,v100,fine,currentTool\WObj:=currentWobj;
        !WaitTime 1;
        WorldAccLim\On:=specialParams{4};
        MoveL Target_520,vmax,z20,currentTool\WObj:=currentWobj;
        WorldAccLim\On:=specialParams{5};
        MoveL Target_530,vmax,z5,currentTool\WObj:=currentWobj;
        !SingArea \Wrist;
        !WorldAccLim\Off;
        !MoveL Target_540,v300,fine,currentTool\WObj:=currentWobj;

    ENDPROC

    !Get a random number between 0 and 1
    FUNC NUM RANDOM()
        VAR num val:=1;

        var pos pos_current;
        pos_current:=CPos(\Tool:=tool0\WObj:=wobj0);

        !1 seeding, Seed by position
        val:=val*(Abs(ROUND(pos_current.x*100,\Dec:=0))+1);
        !+1 to avoid mul by 0 scenario
        val:=789+val MOD 1000;
        !777 is just a bogus valu, since the mod might give 0

        !Feel free to seed with y,z, more

        !2. seeding, Seed by time
        val:=val*(GetTime(\Sec)+1);
        val:=val*(GetTime(\Min)+1);


        !3 seeding, increment callcount and handle large number
        callCount:=callCount+1;
        IF callCount>1000 THEN
            callCount:=234;
        ENDIF

        !finally a division to get something interesting large float number
        val:=val/callCount;

        !get a value between 0 and 1 using COS ( math)
        RETURN (0.5+COS(val)/2);

    ENDFUNC

    PROC TiltVibrate()
        VAR robtarget actualPos;
        actualPos:=CRobT(\Tool:=tool0\WObj:=wobj0);
        
        MoveL actualPos,vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, -20, 10, 10), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, -30, -30, 0), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, 0, 20, 10), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, 30, 0, 0), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, 10, 0, 10), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, 10, 10, 0), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, 15, -20, 10), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, -20, -20, 0), vmax,z0,tool0\WObj:=wobj0;
        MoveL Offs(actualPos, -30, 10, 10), vmax,z0,tool0\WObj:=wobj0;
        MoveL actualPos,vmax,z0,tool0\WObj:=wobj0;
        
        
    ENDPROC

    PROC KruskalCounter()
        VAR num rand;
        
        IF specialParams{1} = 1 THEN
            Vibrate;
        ENDIF
        
        IF specialParams{5} = 1 THEN
            TiltVibrate;
        ELSEIF specialParams{5} = 2 THEN
            ! home
            MoveAbsJ homeJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            ! south
            MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ southJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            ! southwest
            MoveAbsJ SW,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            ! southeast
            MoveAbsJ SE,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            ! northeast
            MoveAbsJ NE,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            ! southeast
            MoveAbsJ SE,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            MoveAbsJ homeJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
        ELSEIF specialParams{5} = 3 THEN
            ! setup
            MoveAbsJ  homeJ, currentSpeed, currentZone, currentTool\Wobj:=currentWobj;
            WaitTime\InPos, specialParams{1};
            MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ southJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            ! random (SW -> NW -> SW -> SE -> NE)
            MoveAbsJ [[0.0,-74.42,28.56,0.0,-10.0,90.0],[9E9,9E9,9E9,9E9,9E9,9E9]], currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            MoveAbsJ [[0.0,-74.42,28.56,0.0,-10.0,180.0],[9E9,9E9,9E9,9E9,9E9,9E9]], currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            MoveAbsJ [[0.0,-74.42,28.56,0.0,-10.0,90.0],[9E9,9E9,9E9,9E9,9E9,9E9]], currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            MoveAbsJ [[0.0,-74.42,28.56,0.0,-10.0,0.0],[9E9,9E9,9E9,9E9,9E9,9E9]], currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            MoveAbsJ [[0.0,-74.42,28.56,0.0,-10.0,-90.0],[9E9,9E9,9E9,9E9,9E9,9E9]], currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            ! return to home
            MoveAbsJ southJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            MoveAbsJ  homeJ, currentSpeed, currentZone, currentTool\Wobj:=currentWobj;
        ELSEIF specialParams{5} = 4 THEN
            ! setup
            MoveAbsJ  homeJ, currentSpeed, currentZone, currentTool\Wobj:=currentWobj;
            WaitTime\InPos, specialParams{1};
            MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ southJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            ! random (SE -> NE)
           ! MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
           ! MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ SE, currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
           ! MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ NE, currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
           ! MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            ! return to home
           ! MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ southJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
           ! MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ  homeJ, currentSpeed, currentZone, currentTool\Wobj:=currentWobj;
        ELSEIF specialParams{5} = 5 THEN
            ! setup
            MoveAbsJ  homeJ, currentSpeed, currentZone, currentTool\Wobj:=currentWobj;
            WaitTime\InPos, specialParams{1};
            MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ southJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
           ! TiltVibrate;
            ! random (SE -> NE)
            MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ SE, currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
          !  TiltVibrate;
            MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ NE, currentSpeed, currentZone, currentTool\Wobj:=wobj0;
            WaitTime\InPos,specialParams{1};
          !  TiltVibrate;
            MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            ! return to home
            MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ southJ,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
           ! TiltVibrate;
            MoveAbsJ startS,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
            MoveAbsJ  homeJ, currentSpeed, currentZone, currentTool\Wobj:=currentWobj;
        ELSEIF specialParams{5} = 6 THEN
            ! setup
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            ! tilt in order N, S, W, E, NE, SE, SW, NW
            MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            !MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            ! return
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
        ELSEIF specialParams{5} = 7 THEN
             ! setup
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            FOR i FROM 1 TO specialParams{2} DO
                ! 10 randomized movements
                FOR j FROM 1 TO 10 DO
                    rand := RANDOM();
                    IF rand <= 0.125 THEN
                        MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.25 THEN
                        MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.375 THEN
                        MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                      !  MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.5 THEN
                        MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.625 THEN
                        MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.75 THEN
                        MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.875 THEN
                        MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 1 THEN
                        MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ENDIF
                ENDFOR 
            ENDFOR
            ! return
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            ELSEIF specialParams{5} = 8 THEN
            ! setup
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            ! tilt in order N, S, W, E, NE, SE, SW, NW
            MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            !MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            
            MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            !MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
           ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
          !  MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
          !  MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            MoveAbsJ tiltNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            WaitTime\InPos,specialParams{1};
            TiltVibrate;
            WaitTime\InPos,specialParams{1};
            MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
          !  MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
            ! return
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
        ELSEIF specialParams{5} = 9 THEN
             ! setup
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            FOR i FROM 1 TO specialParams{2} DO
                ! 10 randomized movements
                FOR i FROM 1 TO 10 DO
                    rand := RANDOM();
                    IF rand <= 0.125 THEN
                        MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotN, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    !    MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.25 THEN
                        MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotS, currentSpeed, currentZone, currentTool\WObj:=currentWobj;                    
                      !  MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.375 THEN
                        MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.5 THEN
                        MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;                    
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.625 THEN
                        MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotNE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.75 THEN
                        MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotSE, currentSpeed, currentZone, currentTool\WObj:=currentWobj;                    
                      !  MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 0.875 THEN
                        MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotSW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ELSEIF rand <= 1 THEN
                        MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        MoveAbsJ tiltNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                        WaitTime\InPos,specialParams{1};
                        TiltVibrate;
                        WaitTime\InPos,specialParams{1};
                        MoveAbsJ rotNW, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                       ! MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
                    ENDIF
                ENDFOR
            ENDFOR
            ! return
            MoveAbsJ horz, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
            
        ENDIF

    ENDPROC

ENDMODULE