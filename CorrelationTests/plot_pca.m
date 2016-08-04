function plot_pca(D)
    meanD = repmat(mean(D),length(D),1);
    D = D - meanD;
    [U,S,V]=svd(D);
    
    for i=1:size(D,2)
        aux=V(:,i)*V(:,i)'*D';
        aux=aux' + meanD;

        if(size(D,2) == 2)
            plot(aux(:,1),aux(:,2),'g');  
        else
            plot3(aux(:,1),aux(:,2),aux(:,3),'g')
        end
        
    end
end