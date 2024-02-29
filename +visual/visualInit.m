function visualInit(version)
arguments
    version = "default"
end
%VISUALINIT この関数の概要をここに記述
    if version == "default"
        set(0,"DefaultAxesFontSize",13);    % フォントサイズ13
        set(0,"DefaultLineLineWidth",2);    % 線の太さ2
    else
        set(0,"DefaultAxesFontSize",11);    % フォントサイズ11
        set(0,"DefaultLineLineWidth",1);    % 線の太さ1
    end
    set(0,"DefaultAxesXGrid",'on');     % X軸方向のグリッドON
    set(0,"DefaultAxesYGrid",'on');     % Y軸方向のグリッドON
end

