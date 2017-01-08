function frame_idx = bootstrapFrames(dataset, frame_number)
% Returns the frame index of the image used for the initialization given
% frame_number being first or second image of the bootstrapping pair.
% 
% Input:
%  - dataset(1x1) : dataset selector 0: KITTI, 1: Malaga, 2: parking
%  - frame_number(string) : pair item specifier
%
% Output:
%  - frame_idx(1x1) : image to choose for bootstrap image

if strcmp(frame_number,'first')
    frame_idx = 1;
elseif strcmp(frame_number,'second')
    if dataset == 0 % KITTI
        frame_idx = 3;
    elseif dataset == 1 % MALAGA
        frame_idx = 6;
    elseif dataset == 2 % PARKING
        frame_idx = 5;
    elseif dataset == 3 % POLY-UP
        frame_idx = 5;
    elseif dataset == 4 % POLY-DOWN
        frame_idx = 3;
    else
        error('invalid dataset!');  
    end
else
    error('invalid frame_number!');
end

end
