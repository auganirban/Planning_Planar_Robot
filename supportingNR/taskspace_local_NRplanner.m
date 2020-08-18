function[q_new] = taskspace_local_NRplanner(qe, vel, dof)
   global v;           global dista;         global contact_link_lengths;
   global q_o;         global contact_nrmls; global links;
   global obs_centers; global obs_radii;     global q_array; global vc_array;

    % My Local Planner function
    v = vel'; 
    q_o = qe'; 
    
    % vector to hold solution of unknowns
    z = zeros(2*dof, 1);
    
    % Fix the lower and upper bound of unknowns
    for j = 1:2*dof
        l(j) = -Inf;
        u(j) = Inf;
    end
    
    % Modify lower bound of complementarity variables
    for j = dof+1:2*dof
        l(j) = 0;
    end
    
    % Get collission info and call pathsolver
    q_new = qe';
    
    % Get collision information
    [contact_nrmls, dista, contact_link_lengths, ~] = get_colli_infoNR(links, q_new, obs_centers, obs_radii);
    [z, ~, ~] = pathmcp(z,l,u,'taskspaceNR_mcpfuncjacEval');
    q_o = z(1:dof);
    q_new = z(1:dof)';
    q_array = [q_array, q_o];
    vc_array = [vc_array, z(dof+1:end)];
end