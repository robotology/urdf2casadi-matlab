function matrix = cell2mat_casadi(casadiCellArray,concatenationDirection)
%Concatenate cell arrays with casadi matrices along column or row and
%return a matrix. Each array element needs to have the same dimension. 
numRowElementsPerArray = size(casadiCellArray{1,1},1);
numColumnsElementsPerArray = size(casadiCellArray{1,1},2);
numCells = size(casadiCellArray,2);
if strcmp(concatenationDirection,'concatenateAlongColumn')
   matrix = casadi.SX(numRowElementsPerArray*numCells,numColumnsElementsPerArray);
   for i = 1 : numCells
        elementIndex = numRowElementsPerArray*(i-1);
        matrix(elementIndex+1:elementIndex+numRowElementsPerArray,:) = casadiCellArray{1,i};
   end
else
   matrix = casadi.SX(numRowElementsPerArray,numCells*numColumnsElementsPerArray);
   for i = 1 : numCells
        elementIndex = numColumnsElementsPerArray*(i-1);
        matrix(:,elementIndex+1:elementIndex+numColumnsElementsPerArray) = casadiCellArray{1,i};
   end
end
