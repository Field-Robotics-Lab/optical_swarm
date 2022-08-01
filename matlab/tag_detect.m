function [ID,pose] = tag_detect(tags)
% tag_detect receives an AprilTagDetectionArray and returns an n-column
% vector of tag IDs, where n is the number of tags; and a three-row
% vector with the X,Y,Z positions of each tag IN THE CAMERA REFERENCE FRAME

numtags = numel(tags.Detections);

for i = 1:numtags
    ID(i) = tags.Detections(i).Id;
    pose(1,i) = tags.Detections(i).Pose.Pose.Pose.Position.X;
    pose(2,i) = tags.Detections(i).Pose.Pose.Pose.Position.Y;
    pose(3,i) = tags.Detections(i).Pose.Pose.Pose.Position.Z;
end



end