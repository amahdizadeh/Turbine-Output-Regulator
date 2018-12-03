function out = SelectRowCol(RowCol, Sys)

Len = length(RowCol);
p = size(Sys.B,2);  % number of inputs

LWT.A = zeros(Len);
LWT.B = zeros(Len,p);
LWT.B_d = zeros(Len,p);


for i=1:Len
    LWT.B(i) = Sys.B(RowCol(i));
    LWT.B_d(i) = Sys.Bd(RowCol(i));
    for j=1:Len
        LWT.A(i,j) = Sys.A(RowCol(i),RowCol(j));        
    end
end

out = LWT;

end