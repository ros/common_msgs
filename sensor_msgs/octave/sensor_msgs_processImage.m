%% I = sensor_msgs_processImage(image_msg)
%%
%% returns an image matrix given a sensor_msgs/Image
function I = sensor_msgs_processImage(image_msg)
%res.camimage_msg.layout
%I = zeros([data.height data.width 3]);
I = [];
if( isempty(image_msg.depth) )
    return;
end
layout = eval(sprintf('image_msg.%s_data.layout',image_msg.depth));
dimsizes = [];
for i = 1:length(layout.dim)
    dimsizes(i) = layout.dim{i}.size;
end
if( ~isempty(dimsizes) )
    I = reshape(eval(sprintf('image_msg.%s_data.data',image_msg.depth)),dimsizes(end:-1:1));
    %% reverse the dimension order
    I = permute(I,length(dimsizes):-1:1);
end
