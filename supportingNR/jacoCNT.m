function [jcon] = jacoCNT(links, th, index, contact_link_length)
    n = length(links);
    th = th(1:index);
    links = links(1:index);
    links(index) = contact_link_length;
    
    jcon = jacobNR(links, th);
    if index < n
        jcon = [jcon, zeros(2, n-index)];
    end
end