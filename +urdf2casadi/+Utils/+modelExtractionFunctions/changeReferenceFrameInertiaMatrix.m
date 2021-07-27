function output = changeReferenceFrameInertiaMatrix(I_gG, t_og,rot_GL,mass, rotation_convention)
%Function to compute inertia transfom from a frame G (centered at the center of mass g) to a frame L (centered in the body lical frame origin
% point o) given a translation from g to o of t_og and a rotation from L to G of rot_LG  
% Import skew function
import urdf2casadi.Utils.Spatial.skew

if (size(I_gG,2) == 6)
    if strcmp(rotation_convention,'rpy')
        % See http://planning.cs.uiuc.edu/node102.html
        % or http://www.ladispe.polito.it/corsi/meccatronica/01pcyqw/2012-13/Slides/Reference%20frames%20and%20rotations.pdf
        % Roll(rotation about x)
        R_r =[ 1,            0,           0;
               0,  cos(rot_GL(1)), sin(rot_GL(1));
               0, -sin(rot_GL(1)), cos(rot_GL(1))];
       % Pitch(rotation about y)
        R_p =[ cos(rot_GL(2)), 0, -sin(rot_GL(2));
               0,              1,           0;
               sin(rot_GL(2)), 0,  cos(rot_GL(2))];
        % Yaw(rotation about z)
        R_y =[ cos(rot_GL(3)), sin(rot_GL(3)), 0;
              -sin(rot_GL(3)), cos(rot_GL(3)), 0;
               0,           0,                 1];

        G_R_L = R_y*R_p*R_r;
        L_R_G = G_R_L';
    end
    % Inertia matrix wrt orientation of frame A centered in point g
    I_gA = L_R_G*I_gG*L_R_G.';
    % Inertia matrix wrt orientation of frame A and cetered in o
    output = I_gA - mass*skew(t_og)*skew(t_og);
end
if (size(I_gG,2) == 3)
    if strcmp(rotation_convention,'rpy')
        % See http://planning.cs.uiuc.edu/node102.html
        % or http://www.ladispe.polito.it/corsi/meccatronica/01pcyqw/2012-13/Slides/Reference%20frames%20and%20rotations.pdf
        % Roll(rotation about x)
        R_r =[ 1,            0,           0;
               0,  cos(rot_GL(1)), sin(rot_GL(1));
               0, -sin(rot_GL(1)), cos(rot_GL(1))];
        % Pitch(rotation about y)
        R_p =[ cos(rot_GL(2)), 0, -sin(rot_GL(2));
               0,              1,           0;
               sin(rot_GL(2)), 0,  cos(rot_GL(2))];
        % Yaw(rotation about z)
        R_y =[ cos(rot_GL(3)), sin(rot_GL(3)), 0;
              -sin(rot_GL(3)), cos(rot_GL(3)), 0;
               0,           0,                 1];
        % Rotation matrix from local body frame (L) orientation to the frame centered 
        % in the center of mass (G) and wrt the Inertia Matrix is computed and
        % repoted in the udf
        G_R_L = R_y*R_p*R_r;
        L_R_G = G_R_L';
    end
    % Inertia matrix wrt orientation of frame A centered in point g
    output = L_R_G*I_gG*L_R_G.';
end
end