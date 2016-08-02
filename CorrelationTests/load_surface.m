function [X,Y,Z] = load_surface(fileName)
    points = load(fileName);
    VIEW_DIM = sqrt(length(points));

    %adjust coordinates to matlab's rendering 
    Y = reshape(points(:,1),VIEW_DIM,VIEW_DIM);
    Z = reshape(points(:,2),VIEW_DIM,VIEW_DIM);
    X = reshape(points(:,3),VIEW_DIM,VIEW_DIM);
    
    %replace values for points that corespond to the floor
    floor = Z(:) < 1;
    Z(floor) = min( Z( ~floor ) );
    
%     Z(Z(:)<1) = nan;
end