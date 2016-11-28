function frame_idx = bootstrap_frames(dataset, frame_number)
%%% TODO STUB

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