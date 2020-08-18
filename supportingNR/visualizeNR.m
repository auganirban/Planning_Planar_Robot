function[xe, ye] = visualizeNR(links, th)    
    xe = 0; xb = 0;
    ye = 0; yb = 0;
    k = 1;
    
    m(k) = plot(xe, ye, "ro", 'markerfacecolor', [1, 0, 0]);
    
    for i = 1:length(links)
        th_current = 0;
        for j = 1:i
            th_current = th_current + th(j);
        end
        xe = xe + links(i)*cos(th_current);
        ye = ye + links(i)*sin(th_current);
        m(k+1) = plot([xb, xe], [yb, ye], "b", "LineWidth", 2);
        m(k+2) = plot(xe, ye, "ro", 'markerfacecolor', [1, 0, 0]);
        xb = xe; yb = ye';
        k = k+2;
    end
    
    drawnow;
    pause(0.1);
    
    for i = 1:length(m)
        delete(m(i));
    end
end
