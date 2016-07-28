%2 shapes of different sizes : 
% .---          ----.
% |                 |
% .---              |
%               ----.
% 
% 
close all; 

%same orientation
a = [ 0 1 ;
      1 1 ;
      1 0 ;
      0 0 ];
  
b = [ 3 5 ;
      5 5 ;
      5 3 ;
      3 3 ]; 
  
visualise_corr(a,b);

%same orientation reverse points
a = [ 0 1 ;
      1 1 ;
      1 0 ;
      0 0 ];
  
b = [ 3 3 ;
      5 3 ;
      5 5 ;
      3 5 ]; 
  
visualise_corr(a,b);

%arbitrary uncorrelated shape
a = [ 0 1 ;
      1 1 ;
      1 0 ;
      0 0 ];
  
b = [ 3 5 ;
      3 3 ;
      5 5 ;
      5 3 ];
  
visualise_corr(a,b);

%reverse orientation same points
a = [ 0 1 ;
      1 1 ;
      1 0 ;
      0 0 ];
  
b = [ 5 5 ;
      3 5 ;
      3 3 ;
      5 3 ]; 
  
visualise_corr(a,b);

%square and form
a = [ 0 1 ;
      1 1 ;
      1 0 ;
      0 0 ;
      0 1 ];
  
b = [ 3 6 ;
      3 5 ;
      5 5 ;
      5 3 ;
      3 3 ]; 
  
visualise_corr(a,b);

%90 degrees rotated
a = [ 0 1 ;
      1 1 ;
      1 0 ;
      0 0 ];
  
b = [ 5 5 ;
      5 3 ;
      3 3 ;
      3 5 ]; 

visualise_corr(a,b);

%90 degrees the other way
a = [ 0 1 ;
      1 1 ;
      1 0 ;
      0 0 ];
  
b = [ 3 3 ;
      3 5 ;
      5 5 ;
      5 3 ];

visualise_corr(a,b);
