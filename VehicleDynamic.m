function [PositionN, VelocityN, AccelerationN] = VehicleDynamic(u,Tim_step,Position,Velocity,Acceleration,Tao)
% Vehicle dynamics

    PositionN = Position + Velocity*Tim_step;
    VelocityN = Velocity + Acceleration*Tim_step;
    AccelerationN = Acceleration - 1/Tao*Acceleration*Tim_step + 1/Tao*u*Tim_step;
   
end

