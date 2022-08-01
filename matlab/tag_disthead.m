function [tagID,dist,head] = tag_disthead(ID,pose,xform)
% tag_disthead receives:
%   - a vector of tag IDs
%   - a vector of X,Y,Z postions in the CAMERA REFERENCE FRAME
%   - tf transform from CAMERA FRAME to BODY FRAME
% tag_dist_head returns:
%   - vector of tag IDs
%   - vector of distances from BODY ORIGIN to tags 
%   - vector of heading angles (degrees)
    


origin = [xform.Transform.Translation.X, xform.Transform.Translation.Y, ...
    xform.Transform.Translation.Z]';
quat = [xform.Transform.Rotation.W,...
        xform.Transform.Rotation.X,...
        xform.Transform.Rotation.Y,...
        xform.Transform.Rotation.Z];
rotm = quat2rotm(quat);


tagID = ID;
for ii=1:numel(tagID)
    pos(:,ii) = origin + rotm*(pose(:,ii));
    dist(ii) = norm(pos(:,ii));
    head(ii) = atan2d(pos(2,ii),pos(1,ii));

end