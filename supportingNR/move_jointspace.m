function [th_current, current_dist] = move_jointspace(th_to, th_from, beta, goal_st, links, obs_centers, obs_radii, boundary)
    
    move_dir = (th_to - th_from)/norm(th_to - th_from);
    nn = floor(norm(th_to - th_from)/beta);
    th_prev = th_from;
    for i = 1:nn
        th_current = th_prev + beta*move_dir;
        if ~check_colli_RRT(links, th_current, obs_centers, obs_radii, boundary)
            th_current = th_prev;
            break;
        end
        % Limit th_prev within -pi and pi
        for i = 1:length(th_current)
            if th_current(i) < -pi
                th_current(i) = th_current(i) + 2*pi;
            end
            if th_current(i) > pi
                th_current(i) = th_current(i) - 2*pi;
            end
        end
        th_prev = th_current;
    end
    current_dist = norm((th_current - goal_st), Inf);
end