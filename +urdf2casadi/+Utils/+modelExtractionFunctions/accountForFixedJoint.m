function smds = accountForFixedJoint(smds)
%Accorporate bodies connected by `fixed`  joints 

% Find indeces of fixed joints
fixedJoints = find(ismember(smds.jtype,'fixed'));

%% Save fixed joint and corresponding link name
smds.fixed.jointName = smds.jointName(fixedJoints);   
smds.fixed.linkName = smds.linkName(fixedJoints);
smds.fixed.linkInertia = smds.I(fixedJoints);
smds.fixed.XTree = smds.Xtree(fixedJoints);

if ~isempty(fixedJoints)
    % For each joint j accorporate the link j in link j-1 
    for j = smds.NB:-1:1

        if strcmp(smds.jtype{j}, 'fixed')
            % Update spatial inertia matrix of bodies connected by fixed joints
            % Sum the spatial inertia of the body connected by the fixed joint by
            % first expressing the inestia in the same reference frame
            % Exclude the first joint from the count:
            % If the joint connecting the first link with the base link is fixed
            % its inertia is added to the base link, which for now is condered
            % fixed.
            if j>1
            smds.I{1,j-1} = smds.I{1,j-1} + (smds.Xtree{j}.')*smds.I{1,j}*smds.Xtree{j};
            end
            
            if j<smds.NB
                % Update the transform considering that the joint is fixed 
                smds.Xtree{j+1} = smds.Xtree{j+1}*smds.Xtree{j};
                
                % Update parents list. Since we are removing the fixed joint
                % link we decrease by one the parents.
                smds.parent(j+1:end) = smds.parent(j+1:end)-1;
            end

        end
    end
    %% Remove link corresponding to fixed joints
    % Update the final number of considered joints
    smds.NB = smds.NB - max(size(fixedJoints));
    
    % Delete from the list of parents the entry of the body with index
    % fixedJoint(j), even if it connects with the base link
    smds.parent(fixedJoints) = [];
    
    % Delete body at fixedJoints(j) index from body list
    smds.I(fixedJoints) = [];
    
    % The masses should have already been added in the spatial inertia
    smds.mass(fixedJoints) = [];
    smds.com(fixedJoints) = [];
    smds.rpy(fixedJoints) = [];
    
    % Delete fixed joint from joint list
    smds.Xtree(fixedJoints) = [];
    smds.linkName(fixedJoints) = [];
    smds.jointName(fixedJoints) = [];
    
    % Remove fixed joints from list of joint types and axis
    smds.jtype(fixedJoints) = [];
    smds.jaxis(fixedJoints) = [];
end
end