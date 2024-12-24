-- Nicolas Alt, 2014-09-04
-- Cheng Chi, 2023-07-27
-- Yifan Hou, 2024-12-17
--   - Added Velocity resolved impedance control
-- Command-and-measure script
-- Tests showed about 30Hz rate
require "socket"
cmd.register(0xBA); -- Measure only
cmd.register(0xBB); -- Position PD
cmd.register(0xBC); -- Velocity-resolved impedance control

function hasbit(x, p)
  return x % (p + p) >= p       
end

function send_state()
    -- ==== Get measurements ====
    state = gripper.state();
    pos = mc.position();
    speed = mc.speed();
    force = mc.aforce();
    time = socket.gettime();

    if cmd.online() then
        -- Only the lowest byte of state is sent!
        -- printf("send id: %#02X\n", id);
        cmd.send(id, etob(E_SUCCESS), state % 256, ntob(pos), ntob(speed), ntob(force), ntob(time));
    end
end

function process()
    id, payload = cmd.read();
    printf("id: %#02X\n", id);
    -- Position control
    if id == 0xBB then
        print("Running 0xBA")
        -- get args
        cmd_pos = bton({payload[1],payload[2],payload[3],payload[4]});
        --printf( "cmd_pos is %f\n", cmd_pos )
        cmd_kp = bton({payload[5],payload[6],payload[7],payload[8]});
        --printf( "cmd_kp is %f\n", cmd_kp )
        cmd_kd = bton({payload[9],payload[10],payload[11],payload[12]});
        --printf( "cmd_kd is %f\n", cmd_kd )
        cmd_travel_force_limit = bton({payload[13],payload[14],payload[15],payload[16]});
        --printf( "cmd_travel_force_limit is %f\n", cmd_travel_force_limit )
        cmd_blocked_force_limit = bton({payload[17],payload[18],payload[19],payload[20]});
        --printf( "cmd_blocked_force_limit is %f\n", cmd_blocked_force_limit )
       
        -- get state
        pos = mc.position();
        vel = mc.speed();
        
        -- pd controller
        e = cmd_pos - pos;
        de = - vel;
        act_vel = cmd_kp * e + cmd_kd * de;
        --printf( "act_vel is %f\n", act_vel )
        -- command
        mc.speed(act_vel);
        -- force limit
        if mc.blocked() then
            mc.force(cmd_blocked_force_limit);
        else
            mc.force(cmd_travel_force_limit);
        end
    end

    -- Velocity-resolved impedance control
    if id == 0xBC then
        -- get args
        cmd_pos = bton({payload[1],payload[2],payload[3],payload[4]});
        cmd_force = bton({payload[5],payload[6],payload[7],payload[8]});
        cmd_kp = bton({payload[9],payload[10],payload[11],payload[12]});
        cmd_kf = bton({payload[13],payload[14],payload[15],payload[16]});
        
        -- get state
        pos = mc.position();
        vel = mc.speed();
        -- force = mc.aforce();
        
        -- vel-resolved controller
        vel_cmd = cmd_kp * (cmd_pos - pos) - cmd_kf * cmd_force
       
        -- command
        mc.speed(vel_cmd);
        printf("force:%f\n", force);
    end

    
    send_state();
    -- 1000*(socket.gettime() - t_start)
    
end

t_start = socket.gettime();
while true do
    -- printf( "Current Interface is %s\n", cmd.interface() )
    if cmd.online() then
        if not pcall(process) then
            print("Error occured")
            sleep(100)
        end
    else
        sleep(100)
    end
end