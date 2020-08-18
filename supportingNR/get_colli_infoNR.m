function[contact_nrmls, dista, contact_link_lengths, link_c_cp_array] = get_colli_infoNR(links, th, obs_centers, obs_radii)
    BIG = 1e6;

    % Define contact wrench vector
    contact_nrmls = zeros(2,length(links));
    dista = zeros(1, length(links));
    contact_link_lengths = zeros(1, length(links));
    link_c_cp_array = zeros(2, 2, length(links));
    
    % Get the end-points of each link
    segs = zeros(2, length(links)+1);
    for i = 1:length(links)
        th_temp = 0;
        for j = 1:i
            th_temp = th_temp + th(j);
        end
        segs(:, i+1) = segs(:, i) + links(i)*[cos(th_temp); sin(th_temp)];
    end
    
    % Compute link and closest circle contact points
    for i = 1:length(links)
        min_dist = BIG; % initialize with some big number
        seg = segs(:, i:i+1);
        for j = 1:length(obs_radii)
            [link_c_cp, current_dist]=circle_line_distance(obs_centers(:, j), obs_radii(j), seg);
            if current_dist < min_dist
                min_dist = current_dist;
                dista(i) = min_dist;
                contact_nrmls(:, i) = (link_c_cp(:, 2) - link_c_cp(:, 1))/norm(link_c_cp(:, 2) - link_c_cp(:, 1));
                link_c_cp_array(:, : , i) = link_c_cp;
            end
        end
    end
    
    % Compute link lengths upto the contact points
    for i = 1:length(links)
        contact_link_lengths(i) = norm(link_c_cp_array(:, 2, i) - segs(:, i));
    end
end