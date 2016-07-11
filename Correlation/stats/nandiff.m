
function [ y ] = nandiff( x )
%like nansum but for difference
x(isnan(x)) = 0;
y = diff(x);
end

