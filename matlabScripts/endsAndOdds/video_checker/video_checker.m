%  this programme is used to read video and select appropriate frame to
%  save and as a illustration for a docx
% 
% Author Keith W.
% Date 07.18.2022
% Ver. 1.0
name = 'G:\pic\交互力\guanyuan\交互力3视频\IMG_0235'; %VID_20220714_150546

obj = VideoReader([name,'.MOV']);
num_frame = obj.NumFrames;
for i = 34500 :4: num_frame
    frame = read(obj,i);
    imshow(frame);
    frame_name = [name,'_',num2str(i),'.jpg'];
end
