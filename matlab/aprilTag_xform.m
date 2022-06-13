function [tagID,dist,head] = aprilTag_xform(tag_tf)
% aprilTag_xform receives tf transforms from detected AprilTags and returns
% a vector of distances and a vector of heading angles for the detected
% tags

for ii=1:numel(tag_tf)


tf = tag_tf(ii);
X = tf.Transform.Translation.X;
Y = tf.Transform.Translation.Y;
tagID{ii} = tf.ChildFrameId;
dist(ii) = norm([X Y]);
head(ii) = atan2d(Y,X);

end