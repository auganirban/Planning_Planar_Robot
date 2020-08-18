function [] = move_jointspace_visualize(th_to, th_from, beta, goal_st, links, obs_centers, obs_radii, boundary)
% % %     global obs_centers; global obs_radii;
    
    move_dir = (th_to - th_from)/norm(th_to - th_from);
    nn = floor(norm(th_to - th_from)/beta);
    th_prev = th_from;
    
    if nn == 0
        return;
    end
    for i = 1:nn
        th_current = th_prev + beta*move_dir;
        if ~check_colli_RRT(links, th_current, obs_centers, obs_radii, boundary)
            th_current = th_prev;
            break;
        end
        if mod(i, 10) == 0
            visualizeNR_RRT(links, th_current);
        end
        th_prev = th_current;
        i
    end
    nn
    current_dist = norm((th_current - goal_st), Inf);
end
