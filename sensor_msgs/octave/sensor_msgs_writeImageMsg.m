%% image_msg = sensor_msgs_processImage(I)
%%
%% returns an image matrix given a sensor_msgs/Image
%% If I is in integer format, assumes range for each channel is [0,255]
%% otherwise range for each channel is [0,1]
function image_msg = sensor_msgs_writeImageMsg(I)

if( ~isinteger(I) )
    I = I*255;
end

s = size(I);

image_msg = sensor_msgs_Image();
image_msg.width = s(2);
image_msg.height = s(1);
if( length(s) >= 3 && s(3) == 3 )
    image_msg.encoding = 'rgb8';
else
    image_msg.encoding = 'mono8';
end

image_msg.data = reshape(permute(uint8(I),length(s):-1:1),[prod(s) 1]); %% reverse the dimension order
