function[jacob] = jacobNR(links, th)
    jacob = zeros(2, length(th));
    
    % Fill in the first column of the jacobian matrix
    [row2, row1] = frdNR(links, th);
    jacob(:, 1) = [-row1; row2];
    
    % Fill in the columns of jacob from left
    n = length(th);
    for i = 2:n
        [row2, row1] = frdNR(links(1:i-1), th(1:i-1));
         jacob(:, i) = jacob(:, 1) + [row1; -row2];
    end
end
