function mat2wnd(source_matrix,destination_file)
%%
% Converts a matrix to wnd file.
%   source_matrix size : N x 2 .
%   first column : Time
%   second column: Wind speed 

    [rows,cols] = size(source_matrix);

    %% writing new ".wnd" destination file 
    fid = fopen(destination_file,'w+');
    for i=1:rows
        time = source_matrix(i,1);
        wind = source_matrix(i,2);
        fprintf(fid,[repmat('%.3f  ',1,8), '\n'],([time wind 0 0 0  0 0 0]'));
    end
    fclose(fid);
end
