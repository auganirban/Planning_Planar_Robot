function h = filledCircle(center,r,n,color)
    theta = linspace(0,2*pi,n);
    rho = ones(1,n)*r;
    [x,y] = pol2cart(theta,rho);
    x = x + center(1);
    y = y + center(2);
    h=fill(x,y,color);
    axis square;
end
