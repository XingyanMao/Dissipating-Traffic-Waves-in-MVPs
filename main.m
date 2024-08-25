clc;clear;close all;
Num_veh  = 3;                            % The number of vehicles in a platoon;
Tim_step = 0.1;
Np = 15;
Time_sim = 33;                           % Time length for simulation
Num_step = floor(Time_sim/Tim_step);            % Simulation setps
Tao  =0.71*ones(Num_veh,1);

%读取真实的领航车的行驶数据
True_Leader = readtable('C:\Users\Administrator\Downloads\MixedPlatoon\veh_frontID_NGSIM_Data\us_101\vehID_1106_frontID_1101\frontID_1101.csv');
v0 = True_Leader.v_Vel/3.6;

%读取真实的跟随车的行驶数据
Ture_Follower = readtable('C:\Users\Administrator\Downloads\MixedPlatoon\veh_frontID_NGSIM_Data\us_101\vehID_1106_frontID_1101\1106_609_2_1101_57.4.csv');
v_f = Ture_Follower.v_Vel/3.6;

%读取真实的跟随车的行驶数据
TureFFol = readtable('C:\Users\Administrator\Downloads\MixedPlatoon\veh_frontID_NGSIM_Data\us_101\vehID_1114_frontID_1106\1114_624_2_1106_56.1.csv');
v_ff = TureFFol.v_Vel/3.6;
%% 扩充数据长度
time_original = linspace(0, Time_sim, length(v0)); % 原始时间范围
time_originalff = linspace(0, Time_sim, length(v_ff)); % 原始时间范围
time_extended = linspace(0, Time_sim, Num_step); % 扩充后的时间范围
% 使用插值函数interp1进行插值操作
v0 = interp1(time_original, v0, time_extended, 'linear'); % 线性插值
v_f = interp1(time_original, v_f, time_extended, 'linear'); % 线性插值
v_ff = interp1(time_originalff, v_ff, time_extended, 'linear'); % 线性插值
% 计算加速度
a0= zeros(size(v0));
a0(2:end) = diff(v0) / (Tim_step); 

a_f= zeros(size(v_f));
a_f(2:end) = diff(v_f) ./ (Tim_step);

a_ff= zeros(size(v_ff));
a_ff(2:end) = diff(v_ff) ./ (Tim_step);

%计算两车的xi值
sum_a_squared = 0;
sum_a_TureFollower_squared = 0;
sum_a_TureFFol_squared = 0;
for i = 1:length(a0)
    sum_a_squared = sum_a_squared + a0(i)^2;
    sum_a_TureFollower_squared = sum_a_TureFollower_squared + a_f(i)^2;
    sum_a_TureFFol_squared = sum_a_TureFFol_squared + a_ff(i)^2;
    if sum_a_TureFollower_squared ~= 0
        xi1(i) = sum_a_squared / sum_a_TureFollower_squared;
    else
        xi1(i) = 0;
    end
    if sum_a_TureFFol_squared ~= 0
        xi2(i) = sum_a_TureFollower_squared / sum_a_TureFFol_squared;
    else
        xi2(i) = 0;
    end
end

a0=a0';
v0=v0';
a_f=a_f';
v_f=v_f';
a_ff=a_ff';
v_ff=v_ff';

%% Varable
Postion  = zeros(Num_step,Num_veh);     % postion of each vehicle;
Velocity = zeros(Num_step,Num_veh);     % velocity of each vehicle;
Acceleration = zeros(Num_step,Num_veh);     % Braking or Tracking Acceleration of each vehicle;
U       = zeros(Num_step,Num_veh);     % Desired Braking or Tracking Acceleration of each vehicle;
Cost    = zeros(Num_step,Num_veh);         % 代价函数
Exitflg = zeros(Num_step,Num_veh);         % 推出机制
%% Leading vehicle
d  = 36;                                 % 期望的跟车误差
x0 = zeros(Num_step,1);
x_f = zeros(Num_step,1);
x_ff = zeros(Num_step,1);
% leader 及跟随者状态更新
v0(1)=30;
v_f(1)=30;
v_ff(1)=30;
for i = 2:Num_step
    v0(i) = v0(i-1)+a0(i)*Tim_step;
    v_f(i) = v_f(i-1)+a_f(i)*Tim_step;
    v_ff(i) = v_ff(i-1)+a_ff(i)*Tim_step;
end
for i = 2:Num_step-Np
    x0(i) = x0(i-1)+v0(i)*Tim_step;
    x_f(i) = x_f(i-1)+v_f(i)*Tim_step;
    x_ff(i) = x_ff(i-1)+v_ff(i)*Tim_step;
end

% 初始分布 无误差
for i = 1:Num_veh
    Postion(1,i) = x_f(1)-i*d;
    Velocity(1,i) = v_f(1);
    Acceleration(1,i) = 0;
end
%% 设置跟车模型的参数
Maximum_Acc=4;
Comfort_dec=-4;
% Distributed MPC assumed state

Pa = zeros(Np,Num_veh);       % Assumed postion of each vehicle;
Va = zeros(Np,Num_veh);       % Assumed velocity of each vehicle;
ua = zeros(Np,Num_veh);       % Assumed  Braking or Tracking Acceleration input of each vehicle;
Pa_next = zeros(Np+1,Num_veh);  % 1（0）：为上一时刻的状态Assumed postion of each vehicle at the newt time step;
Va_next = zeros(Np+1,Num_veh);  % Assumed velocity of each vehicle at the newt time step;
ua_next = zeros(Np+1,Num_veh);  % Assumed Braking or Tracking Acceleration of each vehicle at the newt time step;

% Initialzie the assumed state for the first computation: constant speed

for i = 1:Num_veh
    ua(:,i) = Acceleration(1,i);
    Pa(1,i) = Postion(1,i);                % 假设的第一个点  为文章中的 k=0处，为当前车辆的状态；
    Va(1,i) = Velocity(1,i);
    Ta(1,i) = Acceleration(1,i);
    for j = 1:Np
        [Pa(j+1,i),Va(j+1,i),Ta(j+1,i)] = VehicleDynamic(ua(j,i),Tim_step,Pa(j,i),Va(j,i),Ta(j,i),Tao(i));
    end
end

tol_opt = 1e-5;
options = optimset('Display','off','TolFun', tol_opt, 'MaxIter', 2000,...
    'LargeScale', 'off', 'RelLineSrchBnd', [], 'RelLineSrchBndDuration', 1);

Xend = zeros(Num_step,Num_veh); Vend = zeros(Num_step,Num_veh);
%%  循环仿真
for i = 2:Num_step-Np
    tic
    %% OVM
    v_max=max(v_f(:));
    s_go=39;
    s_st=6;
    s(i-1)=x_f(i-1)-Postion(i-1,1);
    if s(i-1)<s_st
        v_des(i-1)=0;
    elseif s(i-1)>s_go
        v_des(i-1)=v_max;
    else
        v_des(i-1)=v_max/2*(1-cos(pi*(s(i-1)-s_st)/(s_go-s_st)));
    end
    alpha=0.6;
    beta=0.9;
    U(i,1) = alpha*(v_des(i-1)-Velocity(i-1,1))+beta*(v_f(i-1)-Velocity(i-1,1));
    [Postion(i,1),Velocity(i,1),Acceleration(i,1)] = VehicleDynamic(U(i,1),Tim_step,Postion(i-1,1),Velocity(i-1,1),Acceleration(i-1,1),Tao(1));

    %% OURS
    rho2=10;rho1=diag([10,1,10]);
    Vehicle_Type = [Tao(2)];                 % the vehicle parameters ：Tao
    X0 = [Postion(i-1,2),Velocity(i-1,2),Acceleration(i-1,2)];
    Xa2(:,2*i-1:2*i) = [Pa(:,2),Va(:,2)];                               % 自己预期的行为，传递给下一辆车
    Xnfa2(:,3*i-2:3*i) = [x_f(i-1:i+Np-1) - 2*d,v_f(i-1:i+Np-1),a_f(i-1:i+Np-1)];                        % 由邻域内其他车辆传过来的行为
    u0 = ua(:,2);   % 起始搜索点
    A = [];b = []; Aeq = []; beq = [];
    lb =[];ub =[]; % 没有线性约束
    %lb = -6*ones(Np,1); ub = 6*ones(Np,1);% 有线性约束

    [u2new, Cost2(i,2), Exitflg(i,2), output] = fmincon(@(u) Costfunction(Np, Tim_step, X0,u, Vehicle_Type,rho1,rho2,Xnfa2(:,3*i-2:3*i)), ...
        u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(min(xi2),xi2(i),Np, Tim_step, X0, u, Vehicle_Type,Xnfa2(:,3*i-2:3*i)),options);
    U(i,2) = u2new(1);
    [Postion(i,2),Velocity(i,2),Acceleration(i,2)] = VehicleDynamic(U(i,2),Tim_step,Postion(i-1,2),Velocity(i-1,2),Acceleration(i-1,2),Tao(2));
    Temp = zeros(Np+1,3);
    Temp(1,:) = [Postion(i,2),Velocity(i,2),Acceleration(i,2)];
    ua(1:Np-1,2) = u2new(2:Np);
    for j = 1:Np-1
        [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,2),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Tao(2));
    end
    ua(Np,2) = 0;
    [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,2),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Tao(2));
    Pa_next(:,2) = Temp(:,1);
    Va_next(:,2) = Temp(:,2);
    Ta_next(:,2) = Temp(:,3);
    toc

    %% IDM
    Desired_Velocity=max(v_f(:))*0.8;
    velocity_diff3=Velocity(i-1,3)-v_f(i-1);
    Desired_distance=3*d+max(0,0+Velocity(i-1,3)*1.2+Velocity(i-1,3)*velocity_diff3/2/sqrt(Maximum_Acc*Comfort_dec));%期望距离=期望最小距离+速度*安全时距+----速度*与前车的速度差/(2*sqrt(最大加速度*舒适减速度))
    Actual_distance3=x_f(i-1)-Postion(i-1,3);
    ratio_Velocity=Velocity(i-1,3)/Desired_Velocity;
    ratio_Distance=Desired_distance/Actual_distance3;
    U(i,3) = Maximum_Acc*(2-ratio_Velocity^15-ratio_Distance^3);
    [Postion(i,3),Velocity(i,3),Acceleration(i,3)] = VehicleDynamic(U(i,3),Tim_step,Postion(i-1,3),Velocity(i-1,3),Acceleration(i-1,3),Tao(1));

    %跟新交换数据矩阵
    Pa = Pa_next;
    Va = Va_next;
    Ta = Ta_next;
end


%% figure plot
t = (1:Num_step)*Tim_step;
color=[
    [99,178,238];
    [118,218,145];
    [248,203,127]]/255;

figure;
long=0.4*2;
width=0.75/2;
set(gcf,'unit','centimeters','position',[5 0 21*0.84/2 29.7*0.87*0.5]);%21*0.84 29.7*0.87 IEEE trans的页边距
set(gcf,'ToolBar','none','ReSize','off');   % 移除工具栏
set(gcf,'color','w'); % 背景设为白色
subplot(2,1,1)
ve(1) = plot(t, Velocity(:,1)-v_f,'linewidth',1.5,'color',color(1,:));hold on;
ve(2) = plot(t, Velocity(:,2)-v_f,'linewidth',1.5,'color',color(2,:));hold on;
ve(3) = plot(t, Velocity(:,3)-v_f,'linewidth',1.5,'color',color(3,:));hold on;
xlim([0,30])
set(gca,'Position',[0.12 1-0.08-width long width]);
g = get(ve(1),'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Times New Roman','FontWeight','bold');
ylabel('a. Velocity error [m/s]','FontSize',10,'FontName','Times New Roman','FontWeight','bold');
h1 = legend('OVM','Proposed model','IDM','Orientation','horizontal','NumColumns',9);
set(h1,'linewidth',1,'FontSize',10,'FontWeight','bold');
set(h1,'position',[0.4,0.91,0.2,0.1]);%legend位置
set(h1,'Box','off');
xlim([0,30])


subplot(2,1,2)
vef(1) = plot(t, Velocity(:,1)-v_ff,'linewidth',1.5,'color',color(1,:));hold on;
vef(2) = plot(t, Velocity(:,2)-v_ff,'linewidth',1.5,'color',color(2,:));hold on;
vef(3) = plot(t, Velocity(:,3)-v_ff,'linewidth',1.5,'color',color(3,:));hold on;
xlim([0,30])
set(gca,'Position',[0.12 1-0.15-2*width long width]);
g = get(vef(1),'Parent');%对应p1所在的坐标轴
set(g,'Linewidth',1.5,'FontSize',10,'FontName','Times New Roman','FontWeight','bold');
ylabel('b. Velocity error [m/s]','FontSize',10,'FontName','Times New Roman','FontWeight','bold');
xlabel('Time [s]','FontSize',10,'FontName','Times New Roman','FontWeight','bold');


rmse_values1 = sqrt((Velocity(1:24/Tim_step,1) - v_ff(1:24/Tim_step)).^2);
mean_rmse1 = mean(rmse_values1);
median_rmse1 = median(rmse_values1);

rmse_values2 = sqrt((Velocity(1:24/Tim_step,2) - v_ff(1:24/Tim_step)).^2);
mean_rmse2 = mean(rmse_values2);
median_rmse2 = median(rmse_values2);

rmse_values3 = sqrt((Velocity(1:24/Tim_step,3) - v_ff(1:24/Tim_step)).^2);
mean_rmse3 = mean(rmse_values3);
median_rmse3 = median(rmse_values3);