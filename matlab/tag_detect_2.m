function [tagID,dist,head] = tag_detect_2(tags,xform)
% tag_detect receives:
%   - an AprilTagDetectionArray
%   - tf transform from CAMERA FRAME to BODY FRAME
% tag_detect returns:
%   - vector of tag IDs
%   - vector of distances from BODY ORIGIN to tags 
%   - vector of heading angles (degrees) CCW positive, 0* dead ahead

numtags = numel(tags.Detections);
if numtags == 0
    tagID = [];
    dist = [];
    head = [];
else
    for i = 1:numtags
        tagID(i) = tags.Detections(i).Id;
        pose(1,i) = tags.Detections(i).Pose.Pose.Pose.Position.X;
        pose(2,i) = tags.Detections(i).Pose.Pose.Pose.Position.Y;
        pose(3,i) = tags.Detections(i).Pose.Pose.Pose.Position.Z;
    end
    origin = [xform.Transform.Translation.X, xform.Transform.Translation.Y, ...
    xform.Transform.Translation.Z]';
    quat = [xform.Transform.Rotation.W,...
            xform.Transform.Rotation.X,...
            xform.Transform.Rotation.Y,...
         xform.Transform.Rotation.Z];
    rotm = quat2rotm(quat);

    for ii=1:numel(tagID)
        pos(:,ii) = origin + rotm*(pose(:,ii));
        dist(ii) = norm(pos(:,ii));
        head(ii) = atan2d(pos(2,ii),pos(1,ii));
    
    end
end








