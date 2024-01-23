function makePathMovie(q,x,param,t_vec,folder_name,speed)
%MAKEPATHMOVIE この関数の概要をここに記述
%   詳細説明をここに記述
    f = figure;
    disp("start animation")
    padding_frame = round(1/param.dt/speed);
    Nk = param.Nt/speed + 2*padding_frame;  % 前後に静止フレームを追加
    t_list = [1*ones(1,padding_frame),1:speed:param.Nt,param.Nt*ones(1,padding_frame)];   % 1とNtでパディング
    F(Nk) = struct('cdata',[],'colormap',[]);
    for k = 1:Nk
        t = t_list(k);
        % plot here
        visual.snapPath(t,q,x,param);
        legend('off')
        % 
        drawnow;
            %F(k) = getframe(f); % gcfをキャプチャ
            F(k) = getframe;    % gcaをキャプチャ
        if strcmp(get(gcf,'currentcharacter'),'q')  % key stop
            break; % 中止したいときはqを押す
        end
    end
    disp("finish animation")
    v = VideoWriter(folder_name+"path_movie",'MPEG-4');
    %v = VideoWriter(filename,'MPEG-4');
    v.FrameRate = round(1/param.dt);
    open(v);
    writeVideo(v,F(1:end));
    close(v);
    disp("Animation : finish saving")
end

