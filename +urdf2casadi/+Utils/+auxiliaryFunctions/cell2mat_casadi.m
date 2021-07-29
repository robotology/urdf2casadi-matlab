function matrix = cell2mat_casadi(casadiCellArray)
%Concatenate cell arrays with casadi matrices along column and row and
%return a matrix. 
numRowCells = size(casadiCellArray,1);
numColumnCells = size(casadiCellArray,2);
matrix = [];
for j = 1:numColumnCells
    matrix = [matrix, vertcat(casadiCellArray{:,j})];
end