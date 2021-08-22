%% Bilinear interpolation alternative to buitin one (faster code)
function z = bilinear(xs, ys, zs, x, y)
    idx = 1; idy = 1;
    for i=2:length(xs)
        if x <= xs(i)
            idx = i-1;
            break;
        end
    end
    
    for i=2:length(ys)
        if y <= ys(i)
            idy = i-1;
            break;
        end
    end
    
    a = x - xs(idx);
    b = xs(idx+1) - x;
    c = y - ys(idy);
    d = ys(idy+1) - y;
    z = (d*(b*zs(idx,idy) + a*zs(idx+1,idy)) + c*(b*zs(idx,idy+1) + a*zs(idx+1,idy+1))) / ((b+a)*(d+c));
end