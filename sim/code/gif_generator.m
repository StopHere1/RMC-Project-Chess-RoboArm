function gif_generator(fig, filename, index, DelayTime)
    drawnow
    frame = getframe(fig);
    [imind, map] = rgb2ind(frame2im(frame), 256);
    % Write to the GIF File
    if index == 1
        imwrite(imind, map, filename, 'gif', 'Loopcount', inf, 'DelayTime', DelayTime);
    else
        imwrite(imind, map, filename, 'gif', 'DelayTime', DelayTime, 'WriteMode', 'append');
    end
