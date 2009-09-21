%% I = sensor_msgs_processImage(image_msg)
%%
%% returns an image matrix given a sensor_msgs/Image
function I = sensor_msgs_processImage(image_msg)
channels = 1;
if( strfind(image_msg.encoding,'rgb') || strfind(image_msg.encoding,'bgr') || strfind(image_msg.encoding,'C3') )
    channels = 3;
end
I = reshape(image_msg.data,[image_msg.step image_msg.height]);
I = permute(reshape(I(1:channels*image_msg.width,:),[channels image_msg.width image_msg.height]),[3 2 1]);
