function[xe, ye] = frdNR(links, th)
    xe = 0; xb = 0;
    ye = 0; yb = 0;
    k = 1;

    for i = 1:length(links)
        th_current = 0;
        for j = 1:i
            th_current = th_current + th(j);
        end
        xe = xe + links(i)*cos(th_current);
        ye = ye + links(i)*sin(th_current);
        xb = xe; yb = ye';
        k = k+2;
    end
end
