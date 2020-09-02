%%Create transformation matrices from list of D-H parameters
%%with the D_H parameters for the nth link being of the form params(n,:) = [a_i-1, alpha_i-1, di, theta_i]
function T = dhTransform(params)
    if size(params,2) ~= 4
        error('Error: must pass 4xn matrix of D-H parameters(dhTransform())');
    end
    n = size(params,1);
    T = zeros(4,4,n);
    for i = 1:n
        T(:,:,i) = transform(params(i,1), params(i,2), params(i,3), params(i,4));
    end
end
         