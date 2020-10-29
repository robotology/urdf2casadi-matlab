function output = changeReferenceFrameInertiaMatrix(I_gB, t_og,rot_AB,mass, rotation_convention)
%Function to compute inertia transfom from a frame B (centered at the center of mass g) to a frame A (centered in a generel point o) given a translation
%from B to A of t_AB and a rotation from B to A of rot_AB  

if (size(I_gB,2) == 6)
    if strcmp(rotation_convention,'rpy')
        % See http://planning.cs.uiuc.edu/node102.html
        % or http://www.ladispe.polito.it/corsi/meccatronica/01pcyqw/2012-13/Slides/Reference%20frames%20and%20rotations.pdf
        % Roll(rotation about x)
        R_r =[ 1,            0,           0;
               0,  cos(rot_AB(1)), sin(rot_AB(1));
               0, -sin(rot_AB(1)), cos(rot_AB(1))];
       % Pitch(rotation about y)
        R_p =[ cos(rot_AB(2)), 0, -sin(rot_AB(2));
               0,              0,           0;
               sin(rot_AB(2)), 0,  cos(rot_AB(2))];
        % Yaw(rotation about z)
        R_y =[ cos(rot_AB(3)), sin(rot_AB(3)), 0;
              -sin(rot_AB(3)), cos(rot_AB(3)), 0;
               0,           0,                 0];

        A_R_B = R_y*R_p*R_r;
    %     A_R_B = R_r*R_p*R_y;
    end
    % Inertia matrix wrt orientation of frame A centered in point g
    I_gA = A_R_B*I_gB*A_R_B.';
    % Inertia matrix wrt orientation of frame A and cetered in o
    output = I_gA - mass*skew(t_og)*skew(t_og);
end
if (size(I_gB,2) == 3)
    if strcmp(rotation_convention,'rpy')
        % See http://planning.cs.uiuc.edu/node102.html
        % or http://www.ladispe.polito.it/corsi/meccatronica/01pcyqw/2012-13/Slides/Reference%20frames%20and%20rotations.pdf
        % Roll(rotation about x)
        R_r =[ 1,            0,           0;
               0,  cos(rot_AB(1)), sin(rot_AB(1));
               0, -sin(rot_AB(1)), cos(rot_AB(1))];
        % Pitch(rotation about y)
        R_p =[ cos(rot_AB(2)), 0, -sin(rot_AB(2));
               0,              0,           0;
               sin(rot_AB(2)), 0,  cos(rot_AB(2))];
        % Yaw(rotation about z)
        R_y =[ cos(rot_AB(3)), sin(rot_AB(3)), 0;
              -sin(rot_AB(3)), cos(rot_AB(3)), 0;
               0,           0,                 0];

        A_R_B = R_y*R_p*R_r;
        %     A_R_B = R_r*R_p*R_y;
    end
    % Inertia matrix wrt orientation of frame A centered in point g
    output = A_R_B*I_gB*A_R_B.';
end
end