%% I = image_msgs_processImage(image_msg)
%%
%% returns an image matrix given a image_msgs/Image
function I = image_msgs_processImage(image_msg)
%res.camimage_msg.layout
%I = zeros([data.height data.width 3]);
layout = eval(sprintf('image_msg.%s_data.layout',image_msg.depth));
dimsizes = [];
for i = 1:length(layout.dim)
    dimsizes(i) = layout.dim{i}.size;
end
I = reshape(eval(sprintf('image_msg.%s_data.data',image_msg.depth)),dimsizes(end:-1:1));
%% reverse the dimension order
I = permute(I,length(dimsizes):-1:1);
