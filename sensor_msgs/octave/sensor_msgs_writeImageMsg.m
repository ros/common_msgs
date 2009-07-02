%% image_msg = image_msgs_processImage(I)
%%
%% returns an image matrix given a image_msgs/Image
%% If I is in integer format, assumes range for each channel is [0,255]
%% otherwise range for each channel is [0,1]
function image_msg = image_msgs_writeImageMsg(I)

image_msg = image_msgs_Image();
image_msg.label = 'OctaveImage';
image_msg.depth = 'uint8';

if( size(I,3) == 1 )
    image_msg.encoding = 'mono';
elseif ( size(I,3) == 3 )
    image_msg.encoding = 'rgb';
else
    error('unrecognized image format');
end

if( ~isinteger(I) )
    I = I*255;
end

s = size(I);
dim = cell(3,1);
dim{1} = std_msgs_MultiArrayDimension();
dim{1}.label = 'height';
dim{1}.size = s(1);
dim{1}.stride = s(1)*s(2)*s(3);
dim{2} = std_msgs_MultiArrayDimension();
dim{2}.label = 'width';
dim{2}.size = s(2);
dim{2}.stride = s(2)*s(3);
dim{3} = std_msgs_MultiArrayDimension();
dim{3}.label = 'channel';
dim{3}.size = s(3);
dim{3}.stride = s(3);
image_msg.uint8_data.layout.dim = dim;
image_msg.uint8_data.data = reshape(permute(uint8(I),length(s):-1:1),[prod(s) 1]); %% reverse the dimension order
