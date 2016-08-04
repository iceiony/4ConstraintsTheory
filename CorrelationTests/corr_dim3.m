function [full_corr,angleX,angleY,angleZ] = corr_dim3(a,b)
%Calculates correlations between two multi-dimensional random variables.
%The correlation matrix returned is the value of Pearson correlation on 
%each combination of columns of X and Y.

%align pca components
oa = a - repmat(mean(a),length(a),1);
ob = b - repmat(mean(b),length(b),1);
[a,ax,ay,az] = align_pca(a);
[b,bx,by,bz] = align_pca(b);


%correlate on main components
x_corr = corr(a(:,1),b(:,1));
y_corr = corr(a(:,2),b(:,2));
z_corr = corr(a(:,3),b(:,3));

% full_corr = ( abs(x_corr) + abs(y_corr) + abs(z_corr) ) / 3;
full_corr = x_corr * y_corr * z_corr;

sgn = sign([x_corr,y_corr,z_corr]);
if sign(full_corr) > 0 && any(sgn<0)
    if sgn(1) > 0
        bx = mod(bx + pi,2*pi);
    end
    if sgn(2) > 0
        by = mod(by + pi,2*pi);
    end
    if sgn(3) > 0
        bz = mod(bz + pi,2*pi);
    end
end

%determine rotations for the second surface to match the first
r = rotz(az) * roty(ay) * rotx(ax) * rotx(-bx) * roty(-by) * rotz(-bz);

%test lines for estimated rotation matrix
%the difference between points should be 0
round(sum(oa - (r * ob')'),4)

angleZ = atan(r(2)/r(1));
angleX = atan(r(6)/r(9));
angleY = atan(-r(3)/sqrt(r(6)^2 + r(9)^2)); 

% test lines for angles calculations, the result should be 0
% r2 = rotz(angleZ) * roty(angleY) * rotx(angleX); 
% round(r - r2,4)

    function [out,angleX,angleY,angleZ] = align_pca(in)
        in = in - repmat(mean(in),length(in),1);
        [~,~,v] = svd(in);     
   
        %align 1st component to x-axis
        vx = v(:,1);
        angleZ  = mod(atan2(vx(2),vx(1)),2*pi);
        angleY  = mod(-asin(vx(3)),2*pi);
        
        angleZ = remove_error(angleZ);
        angleY = remove_error(angleY);
        
        v =  roty(-angleY) * rotz(-angleZ) * v;

        %align 2nd component to y-axis
        vy = v(:,2);
        angleX = mod(atan2(vy(3),vy(2)),2*pi);
        
        angleX = remove_error(angleX);
        v = rotx(-angleX) * v;

        %rotate points from pca angles
        out = rotx(-angleX) * roty(-angleY) * rotz(-angleZ) * in';
        out = out';
    end

    function [angle] = remove_error(angle)
        if  abs(angle) < 0.0001 || abs(2*pi-angle) < 0.0001 
            angle = 0;
        end
    end

end