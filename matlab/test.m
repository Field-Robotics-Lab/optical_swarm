close all
clear

try
    rosinit
catch
    rosshutdown
    rosinit
end


% ROS Calculation Rate
desiredRate = 10; %10 hertz 
rate = rateControl(desiredRate);




% Setup Publisher

% left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
% right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
% left_msg = rosmessage(left_pub);
% right_msg = rosmessage(right_pub);

%%

while true
tftree = rostf;
frames = tftree.AvailableFrames;
tag_index = find(not(cellfun('isempty',strfind(frames,'april_tag'))));
if isempty(tag_index) == 1
    tags = {};
else
   for i=1:numel(tag_index)
tags{i} = frames{tag_index(i)};
   end
for j=1:numel(tags)
    tag_tf(j) = getTransform(tftree,'sandwich_0/base_link',tags{j},'Timeout',inf);
end
[tagID,dist,head] = aprilTag_xform(tag_tf)

end
% tf1 = frames(index(1))
% tf1 = frames{index(1)}
% 
% tf_1 = getTransform(tftree,'sandwich_0/base_link',tf1,'Timeout',inf)
% tf_1.Transform.Translation
% 
% tf4 = getTransform(tftree,'sandwich_0/base_link','tag_4','Timeout',inf)
% tf1 = getTransform(tftree,'sandwich_0/base_link','tag_1','Timeout',inf)
% trans = tf1.Transform.Translation;
% rot = tf1.Transform.Rotation;


waitfor(rate);
end