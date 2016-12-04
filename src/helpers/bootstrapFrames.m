function frame_idx = bootstrapFrames(dataset, frame_number)
% Sets the frames used for the bootstraping
% 
% Input:
%  - dataset(int)       : which dataset is used 0: KITTI, 1: Malaga, 2: parking
%  - frame_number(int)  : 1 or 2 
%
% Output:
%  - frame_idx(int)     : Frame_id used for initialization

    if frame_number == 1
        frame_idx = 1;
    elseif frame_number == 2
        if dataset == 0 % KITTI
            frame_idx = 3;
        elseif dataset == 1 % MALAGA
            frame_idx = 2;
        elseif dataset == 2 % PARKING
            frame_idx = 2;
        else
            error('invalid dataset!');  
        end
    else
        error('invalid frame_number!');
    end
end