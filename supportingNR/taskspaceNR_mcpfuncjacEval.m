function [f, J, domerr]= taskspaceNR_mcpfuncjacEval(z, jacflag)
    global num_dof; global links; global h; global safe_dist;
    global contact_nrmls; global dista; global contact_link_lengths;
    global q_o; global v;

    f = []; J = []; domerr = 0;

    % Define the unknowns
    q_n = z(1:num_dof);
    v_n = z(num_dof+1:end);
    
    % Compute the contact-Jacobian(s) and their pseudo-inverses
    jcon_array = zeros(2, num_dof, num_dof);
    inv_jcon_array = zeros(num_dof, 2, num_dof);
    
    for i = 1:num_dof
        jcon_array(:, :, i) = jacoCNT(links, q_o, i, contact_link_lengths(i));
        inv_jcon_array(:, :, i) = pinv(jcon_array(:, :, i));
    end
    
    additional_js_vec = zeros(num_dof, 1);
    for i = 1:num_dof
        additional_js_vec = additional_js_vec + inv_jcon_array(:, :, i)*contact_nrmls(:, i)*v_n(i);
    end
    eq_of_motion = (q_o-q_n) + (v) + h*additional_js_vec;

    for i = 1:num_dof
        f(i) = eq_of_motion(i,1);
        f(num_dof+i) = contact_nrmls(:, i)'*jcon_array(:, :, i)*(q_n - q_o) + (dista(i) - safe_dist);
    end
    
    if (jacflag)
        J = zeros(16, 16);
        
        for i = 1:num_dof
            J(i, i) = -1;
            J(1:num_dof, num_dof+i) = h*inv_jcon_array(:, :, i)*contact_nrmls(:, i);
            J(num_dof+i, 1:num_dof) = contact_nrmls(:, i)'*jcon_array(:, :, i);
        end
        
        J = sparse(J);
    end

end
