mod = kinDynComp.model();
for i = 1:6
    from_frame = kinDynComp.getFrameName(i-1);
    to_frame = kinDynComp.getFrameName(i);
    x = kinDynComp.getRelativeTransform(to_frame,from_frame);
    px = x.getPosition();
    rx = x.getRotation();
    posIDyn{i} = px.toMatlab;
    rotIDyn{i} = rx.toMatlab;
    
    x = x.asAdjointTransform();
    x = x.toMatlab();
    xx(4:6,1:3) = x(1:3,4:6);
    xx(1:3,4:6) = x(4:6,1:3);
    xx(1:3,1:3) = x(4:6,4:6);
    xx(4:6,4:6) = x(1:3,1:3);
    XupIDyn{i} = xx;

    xWorld = kinDynComp.getWorldTransform(i); 
    xWorld = xWorld.asAdjointTransform();
    xWorld = xWorld.toMatlab();
    xxW(4:6,1:3) = xWorld(1:3,4:6);
    xxW(1:3,4:6) = xWorld(4:6,1:3);
    xxW(1:3,1:3) = xWorld(4:6,4:6);
    xxW(4:6,4:6) = xWorld(1:3,1:3);
    XWorldIDyn{i} = xxW;
end
%%
for j =1:7
    l = mod.getLink(j-1);
    I = l.getInertia();
    I = I.asMatrix;
    I = I.toMatlab;
    ii(4:6,1:3) = I(1:3,4:6);
    ii(1:3,4:6) = I(4:6,1:3);
    ii(1:3,1:3) = I(4:6,4:6);
    ii(4:6,4:6) = I(1:3,1:3);
    IiDyn{j} = ii;
end