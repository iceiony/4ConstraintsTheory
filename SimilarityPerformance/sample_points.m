function [sample_out] = sample_points(img,sample_size)
    sample_out = [];
    step = size(img) ./ sample_size;

    for i=1:sample_size(1)
        for j=1:sample_size(2)
            x = round(i * step(1));
            y = round(j * step(2));           
            if img(x,y)>0
                sample_out(end+1,:) = [i,j];
            end
        end
    end
    
end