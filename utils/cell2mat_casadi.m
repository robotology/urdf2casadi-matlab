function matrix = cell2mat_casadi(casadiCellArray)
numRowsElementArray = size(casadiCellArray{1,1},1);
numColumnsElementArray = size(casadiCellArray{1,1},2);
numCells = size(casadiCellArray,2);
matrix = casadi.SX(numRowsElementArray,numCells*numColumnsElementArray);
for i = 1 : numCells
    linkOffSet = numColumnsElementArray*(i-1);
    matrix(:,linkOffSet+1:linkOffSet+numColumnsElementArray) = casadiCellArray{1,i};
end