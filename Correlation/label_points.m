function label_points(a,b)
    for i=1:max(length(a),length(b))
        if ( i <= length(a))
            text(a(i,1),a(i,2),num2str(i));
        end
        
        if ( i <= length(b))
            text(b(i,1),b(i,2),num2str(i));
        end
end