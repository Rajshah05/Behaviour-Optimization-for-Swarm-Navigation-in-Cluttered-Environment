function h = circle(x,y,r,num)
    if nargin == 3
        num = 50;
    end
    th = linspace(0,2*pi,num);
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    c = [.5 .5 .5];
    h = fill(xunit, yunit, c);
end