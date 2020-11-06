function [H,HDot,C] =computeSymbolicCoriolismatrix(q,qd,smds)
%Compute mass matrix, the derivative of the mass matrix and the Coriolis
% matrix using Algorithm 1 from 
% "Numerical Methods to Compute the Coriolis Matrix and Christoffel Symbols
% for Rigid-Body System" by Sebastian Echeandia and Patrick M. Wensing
for i=1:smds.NB
    v{i}=Xup{i}*v{smds.parent(i)} + S{i}*qd(i);
    SDot= crm(v{i})*S{i}+ringS{i};
    IC{i} = smds.I{i};
    BC{i} = 0,5*[crf(v{i})*smds.I{i} + crf(smds.I{i}*v{i}) - smds.I{i}*crm(v{i})];
end
for j = smds.NB:-1:1
    F1 = IC{j}*SDot{j} + BC{j}*S{j};
    F2 = IC{j}*S{j};
    F3 = (BC{j}.')*S{j};
    C(j,j) = S{j}.'*F1;
    i = j;
    while i>0
        F1 = Xup{i}.'*F1;
        F2 = Xup{i}.'*F2;
        F3 = Xup{i}.'*F3;
        i = smds.parent(i);
        C(i,j) = S{i}.'*F1;
        C(j,i) = (SDot{i}.'*F2 + S{i}.'*F3).';
        H(i,j) = S{i}.'*F2;
        H(j,i) = (S{i}.'*F2).';
        HDot(i,j) = SDot{i}.'*F2 + S{i}.'*(F1+F3);
    end
    IC{smds.parent(j)} = IC{smds.parent(j)} + Xup{j}.'*IC{j}*Xup{j};
    BC{smds.parent{j}} = BC{smds.parent{j}} + Xup{j}.'*BC{j}*Xup{j};
end
end