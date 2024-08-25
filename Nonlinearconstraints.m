function [C, Ceq] = Nonlinearconstraints(xi_min,xi_max,Np, Tim_step, X0, u, Vehicle_Type,Xnfa)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    Pp = zeros(Np,1);     % Predictive Position
    Vp = zeros(Np,1);     % Predictive Velocity
    Tp = zeros(Np,1);     % Predictive Acceleration
    
    PFa=Xnfa(:,1);
    VFa=Xnfa(:,2);
    TFp=Xnfa(:,3);
	Tao = Vehicle_Type(1);
    
    [Pp(1),Vp(1),Tp(1)] = VehicleDynamic(u(1),Tim_step,X0(1),X0(2),X0(3),Tao);
    for i = 1:Np-1
        [Pp(i+1),Vp(i+1),Tp(i+1)] = VehicleDynamic(u(i+1),Tim_step,Pp(i),Vp(i),Tp(i),Tao);
    end
    
    %% Velocity bounds
    VelMax = 45*ones(Np,1); VelMin = 25*ones(Np,1);
    %% PositionError bounds
    EpMax = 10*ones(Np,1);  EpMin = -10*ones(Np,1);
    C = [Vp-VelMax;VelMin-Vp;...
        Pp-PFa(2:Np+1)-EpMax;-Pp+PFa(2:Np+1)+EpMin;xi_min*norm(TFp,2)-norm(Tp,2);-xi_max*norm(TFp,2)+norm(Tp,2)];%%c(x)\le 0
    Ceq = [Pp(Np)-PFa(Np+1);Vp(Np)-VFa(Np+1);Tp(Np)];%%ceq(x)= 0
end