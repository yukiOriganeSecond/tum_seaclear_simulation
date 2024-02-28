classdef MethodContainer
    properties (Access = private)
        method_name_list
        base_method_list
        change_params_name
        change_params_val
    end
    
    methods
        function obj = MethodContainer()
            obj = obj.clearMethods();
        end

        function obj = clearMethods(obj)
            obj.method_name_list = [];
            obj.base_method_list = [];
            obj.change_params_name = struct;
            obj.change_params_val = struct;
        end
        
        function obj = addMethod(obj, method_name, base_method, options)
            arguments
                obj
                method_name 
                base_method {mustBeMember(base_method,{'RA-SAA','RA-SAA-PID','PID-CBF','MPPI'})}
                options = []    % changing paramter and (average) value ["alpha",0.2] etc
            end
            obj.method_name_list = [obj.method_name_list, method_name];
            obj.base_method_list = [obj.base_method_list, base_method];
            if length(obj.method_name_list) ~= length(obj.base_method_list)
                disp("(MethodContainer) WARN: MISSING list size")
            end
            if (mod(length(options),2)~=0)
                disp("(MethodContainer) WARN: options size should be odd number")
                options = options(:,end-1);
            end
            for i = 1:length(options)/2
                obj.change_params_name.(method_name) = options(2*i-1);
                obj.change_params_val.(method_name) = options(2*i);
            end
        end
    
        function method_n_ = getNumberOfMethods(obj)
            method_n_ = length(obj.method_name_list);
        end

        function method_list_ = getMethodNameList(obj)
            method_list_ = obj.method_name_list;
        end

        function method_name_ = getMethodName(obj,method_index)
            method_name_ = obj.method_name_list(method_index);
        end

        function base_method_ = getBaseMethod(obj,method_index)
            base_method_ = obj.base_method_list(method_index);
        end

        function param_base = subsMethodParameters(obj,method_index,param_base)
            method_name_ = obj.getMethodName(method_index);
            param_name_list_ = obj.change_params_name.(method_name_);
            param_val_list_ = obj.change_params_val.(method_name_);
            for i = 1:length(param_name_list_)
                if isfield(param_base,param_name_list_(i))
                    prev_val_ = param_base.(param_name_list_(i)).average;
                    if islogical(prev_val_) || isnumeric(prev_val_)
                        param_base.(param_name_list_(i)).average = str2num(param_val_list_(i));
                    else
                        param_base.(param_name_list_(i)).average = param_val_list_(i);
                    end
                else
                    disp("(MethodContainer) WARN: "+param_name_list_(i)+" is not parameter name");
                end
            end
        end
    end
end