%% I = sensor_msgs_processImage(image_msg)
%%
%% returns an image matrix given a sensor_msgs/Image
function I = sensor_msgs_processImage(image_msg)
channels = 1;
if( strfind(image_msg.encoding,'32FC') )
    %% have to convert the raw data to floats
    fid = tmpfile();
    fwrite(fid,image_msg.data,'uint8');
    fseek(fid,0,SEEK_SET);
    image_msg.data = fread(fid,length(image_msg.data)/4,'single=>double');
    image_msg.step = image_msg.step/4;
    channels = str2num(image_msg.encoding(end));
elseif( strfind(image_msg.encoding,'64FC') )
    %% have to convert the raw data to floats
    fid = tmpfile();
    fwrite(fid,image_msg.data,'uint8');
    fseek(fid,0,SEEK_SET);
    image_msg.data = fread(fid,length(image_msg.data)/8,'double=>double');
    image_msg.step = image_msg.step/8;
    channels = str2num(image_msg.encoding(end));
elseif( strfind(image_msg.encoding,'8SC') )
    fid = tmpfile();
    fwrite(fid,image_msg.data,'uint8');
    fseek(fid,0,SEEK_SET);
    image_msg.data = fread(fid,length(image_msg.data),'int8=>double');
    channels = str2num(image_msg.encoding(end));
elseif( strfind(image_msg.encoding,'8UC') )
    channels = str2num(image_msg.encoding(end));
elseif( strfind(image_msg.encoding,'16SC') )
    fid = tmpfile();
    fwrite(fid,image_msg.data,'uint8');
    fseek(fid,0,SEEK_SET);
    image_msg.data = fread(fid,length(image_msg.data)/2,'int16=>double');
    image_msg.step = image_msg.step/2;
    channels = str2num(image_msg.encoding(end));
elseif( strfind(image_msg.encoding,'16UC') )
    fid = tmpfile();
    fwrite(fid,image_msg.data,'uint8');
    fseek(fid,0,SEEK_SET);
    image_msg.data = fread(fid,length(image_msg.data)/2,'uint16=>double');
    image_msg.step = image_msg.step/2;
    channels = str2num(image_msg.encoding(end));
elseif( strfind(image_msg.encoding,'32SC') )
    fid = tmpfile();
    fwrite(fid,image_msg.data,'uint8');
    fseek(fid,0,SEEK_SET);
    image_msg.data = fread(fid,length(image_msg.data)/4,'int32=>double');
    image_msg.step = image_msg.step/4;
    channels = str2num(image_msg.encoding(end));
elseif( strfind(image_msg.encoding,'32UC') )
    fid = tmpfile();
    fwrite(fid,image_msg.data,'uint8');
    fseek(fid,0,SEEK_SET);
    image_msg.data = fread(fid,length(image_msg.data)/4,'uint32=>double');
    image_msg.step = image_msg.step/4;
    channels = str2num(image_msg.encoding(end));
else
    if( strfind(image_msg.encoding,'rgb') || strfind(image_msg.encoding,'bgr') )
        channels = 3;
        image_msg.data = image_msg.data*(1/255.0);
    end
end

I = reshape(image_msg.data,[image_msg.step image_msg.height]);
I = permute(reshape(I(1:channels*image_msg.width,:),[channels image_msg.width image_msg.height]),[3 2 1]);
