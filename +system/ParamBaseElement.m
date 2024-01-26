classdef ParamBaseElement
    %UNTITLED このクラスの概要をここに記述
    %   詳細説明をここに記述
    
    properties
        type
        average
        error   % error ratio. ex) \pm 0.03 = \pm 30%
    end
    
    methods
        function val = uncertainCalc(obj, force_deterministic)
            arguments
                obj
                force_deterministic = false;
            end
            if obj.type == "Gaussian"&&force_deterministic
                val = obj.error.*obj.average.*randn(size(obj.average))+obj.average;
            elseif obj.type == "White"&&force_deterministic
                val = obj.error.*obj.average.*(2*rand(size(obj.average))-0.5)+obj.average;
            else
                val = obj.average;
            end
        end
    end
end

