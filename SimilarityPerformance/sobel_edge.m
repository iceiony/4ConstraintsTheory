function[out] = sobel_edge(in)
    in = double(in);

    gx = [-1 -2 -1;
           0  0  0;
           1  2  1];
       
    gy = [-1 0 1;
          -2 0 2;
          -1 0 1];

    X = conv2(gx,in);
    Y = conv2(gy,in);
    
    out = sqrt( X.^2 + Y.^2 );
end