function Cost = Costfunction(Np, Tim_step, X0 ,u, Vehicle_Type,rho1,rho2,Xnfa)%自车
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    Pp = zeros(Np,1);     % Predictive Position
    Vp = zeros(Np,1);     % Predictive Velocity
    Tp = zeros(Np,1);     % Predictive Acceleration
    
    Tao = Vehicle_Type(1);
    
    [Pp(1),Vp(1),Tp(1)] = VehicleDynamic(u(1),Tim_step,X0(1),X0(2),X0(3),Tao);
    for i = 1:Np-1
        [Pp(i+1),Vp(i+1),Tp(i+1)] = VehicleDynamic(u(i+1),Tim_step,Pp(i),Vp(i),Tp(i),Tao);
    end
    
    Xp = [Pp,Vp,Tp];      % Predictive State
    

    Cost = 0;                               % 第一步的优化值
    for i = 1:Np-2        %% 注意范数的定义问题， X'Q'QX
        Cost = Cost + u(i+1)*rho2*u(i+1) + (Xp(i,:)-Xnfa(i+1,:))*rho1*(Xp(i,:)-Xnfa(i+1,:))';               
    end
end