%% PLOTS

% time = readtable('/home/fetty/Desktop/control_ws2022/src/control/tailored_mpc/debug/solve_time.txt');
% % exitflags = readtable('/home/fetty/Desktop/control_ws2022/src/control/tailored_mpc/debug/exit_flags.txt');
% 
% meanTime = mean(time.Var3)
% plot(time.Var1.*10,time.Var3)
% title("Solving  time")
% 
% axis equal

% meanExitflag = mean(exitflags.Var3)
% figure()
% plot(exitflags.Var1,exitflags.Var3)
% title("Exit Flags")

vfinal = readtable('/home/fetty/Desktop/velocity_profile.csv');
vaccel = readtable('/home/fetty/Desktop/v_accel.csv');
vdecel = readtable('/home/fetty/Desktop/v_decel.csv');

figure()
plot(vfinal.Var1)

figure()
plot(vaccel.Var1)
hold on
plot(vdecel.Var1)
