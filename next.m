function y = next(xs, ys, x)
    for i=2:length(xs)
        if x <= xs(i)
            y = ys(i);
            break;
        end
    end
end