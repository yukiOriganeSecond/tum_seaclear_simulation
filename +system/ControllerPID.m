function [ut_use,et] = ControllerPID(qt, qt_norm, ut_norm, param)
%CONTROLLERPID この関数の概要をここに記述
%   詳細説明をここに記述
    persistent integral_et
    et = qt_norm([1,7,3,5],1) - qt([1,7,3,5],1);  % error at time t
    dt_et = qt_norm([2,8,4,6],1) - qt([2,8,4,6],1);

    if isempty(integral_et) % initial step
        integral_et = 0;
    end
    ut_use = ut_norm + param.kp.*et + param.ki.*integral_et + param.kd.*dt_et;

    integral_et = integral_et + param.dt*et;
end
