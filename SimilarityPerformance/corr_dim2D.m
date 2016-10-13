function [full_corr,angleX,a_rotated,b_rotated] = corr_dim2D(a,b)
%Calculates correlations between two multi-dimensional random variables.
%The correlation matrix returned is the value of Pearson correlation on 
%each combination of columns of X and Y.

%align pca components
[a,ax] = align_pca(a);
[b,bx] = align_pca(b);

angleX = abs(ax-bx);


minX = min([a(:,1);b(:,1)]);
maxX = max([a(:,1);b(:,1)]);
lnX = ceil(maxX - minX);

minY = min([a(:,2);b(:,2)]);
maxY = max([a(:,2);b(:,2)]);
lnY = ceil(maxY - minY);

a_rotated = create_img(a,minX,minY,lnX,lnY);
b_rotated = create_img(b,minX,minY,lnX,lnY);

a_rotated = imgaussfilt(a_rotated,1.5);
b_rotated = imgaussfilt(b_rotated,1.5);

full_corr = corr(a_rotated(:),b_rotated(:));

b_mirror1 = b_rotated(:,end:-1:1);
b_mirror2 = b_rotated(end:-1:1,:);
b_mirror3 = b_rotated(end:-1:1,end:-1:1);
corr_mirror1 = corr(a_rotated(:),b_mirror1(:));
corr_mirror2 = corr(a_rotated(:),b_mirror2(:));
corr_mirror3 = corr(a_rotated(:),b_mirror3(:));

if full_corr < corr_mirror1
    full_corr = corr_mirror1;
    b_rotated = b_mirror1;
end

if full_corr < corr_mirror2
    full_corr = corr_mirror2;
    b_rotated = b_mirror2;
end

if full_corr < corr_mirror3
    full_corr = corr_mirror3;
    b_rotated = b_mirror3;
end

    function [out] = create_img(in,minX,minY,lnX,lnY)
        out = zeros(lnY+1,lnX+1);
        x = (in(:,1) - minX)+1;
        y = (in(:,2) - minY)+1;
        
        x = floor(x);
        y = floor(y);
        for idx = 1:length(in)
            j = x(idx);
            i = y(idx);    
            out(i,j) = 1;
        end
    end

    function [out,angleX] = align_pca(in)   
        in = in - repmat(mean(in),length(in),1);        
        [~,~,v] = svd(in);     

        %align 1st component to x-axis        
        angleX = mod(atan2(v(2),v(1)),2*pi);        
        v = rot(-angleX) * v;
        
        if v(2,2) < 0 
            angleX = mod(angleX + pi,2*pi);
        end
        
        out = (rot(-angleX) * in')';
    
    end

    function [angle] = remove_error(angle)
        if  abs(angle) < 0.01 || abs(2*pi-angle) < 0.01 
            angle = 0;
        end
    end

end