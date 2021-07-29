function  [Xj,S] = jcalc(jointAxis, jtyp, q )

% jcalc  joint transform and motion subspace matrices.
% [Xj,S]=jcalc(type,q)  returns the joint transform and motion subspace
% matrices for a joint of the given type.  jtyp can be either a string or a
% structure containing a string-valued field called 'code'.  Either way,
% the string contains the joint type code.  For joints that take
% parameters (e.g. helical), jtyp must be a structure, and it must contain
% a field called 'pars', which in turn is a structure containing one or
% more parameters.  (For a helical joint, pars contains a parameter called
% 'pitch'.)  q is the joint's position variable.

% Import auxiliary functions 
import urdf2casadi.Utils.modelExtractionFunctions.fromRotationAxisToRotationMatrix
import urdf2casadi.Utils.Spatial.xlt
import urdf2casadi.Utils.Spatial.plux
import urdf2casadi.Utils.Spatial.rotz

if ischar( jtyp )
  code = jtyp;
else
  code = jtyp.code;
end

switch code
    
  case 'fixed'				% actually for fixed joint S is 0 x 6 vector(see Featherstone)
    Xj = eye(6);
    S = [0;0;0;0;0;0];
    
  case {'R','Rx','Ry','Rz'}			% revolute X axis
    % According to the URDF convention the axis A is returned wrt the child
    % link. This means that the parent link of the joint rotates of q wrt the child link frame around the axis A. but
    % we need the inverse rotation: of the child wrt the parent around axis
    % A of angle q. This can be done by considering the negative axis.
    Rot = fromRotationAxisToRotationMatrix(-jointAxis,q);
    Xj = plux( Rot, zeros(3,1) );
    S = [jointAxis(1);jointAxis(2);jointAxis(3);0;0;0];

  case 'Px'				% prismatic X axis
    Xj = xlt([q 0 0]);
    S = [0;0;0;jointAxis(1);jointAxis(2);jointAxis(3)];
    
  case 'Py'				% prismatic Y axis
    Xj = xlt([0 q 0]);
    S = [0;0;0;jointAxis(1);jointAxis(2);jointAxis(3)];
    
  case {'P','Pz'}			% prismatic Z axis
    Xj = xlt([0 0 q]);
    S = [0;0;0;jointAxis(1);jointAxis(2);jointAxis(3)];
    
  case 'H'				% helical (Z axis)
    Xj = rotz(q) * xlt([0 0 q*jtyp.pars.pitch]);
    S = [0;0;1;0;0;jtyp.pars.pitch];
    
  case 'r'				% planar revolute
    Xj = plnr( q, [0 0] );
    S = [1;0;0];
    
  case 'px'				% planar prismatic X axis
    Xj = plnr( 0, [q 0] );
    S = [0;1;0];
    
  case 'py'				% planar prismatic Y axis
    Xj = plnr( 0, [0 q] );
    S = [0;0;1];
    
  otherwise
    error( 'unrecognised joint code ''%s''', code );
end
