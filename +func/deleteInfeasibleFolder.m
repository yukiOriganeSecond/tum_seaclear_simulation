
function list_out_ = deleteInfeasibleFolder(list_in_)
    % delete infeasible folder
    empty_folder_index_list_ = [];
    for i = 1:length(list_in_)   
        if ismember(list_in_(i).name, [".","..","fig"])
            empty_folder_index_list_ = [empty_folder_index_list_, i];
        end
    end
    list_out_ = list_in_;
    list_out_(empty_folder_index_list_) = [];
end