function visualFrame(ax, T, options)
%VISUALCONVHULL 
    arguments
        ax 
        T(4,4) double 
        options.scale(1,1) double {mustBePositive} = 0.2
    end

    ax.NextPlot = "add";
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);
    vecX = T(1:3,1) * options.scale;
    vecY = T(1:3,2) * options.scale;
    vecZ = T(1:3,3) * options.scale;

    for i = 1:3
        h = quiver3(x, y, z, vecX(1), vecX(2), vecX(3), 'Color', 'r', 'LineWidth', 2);
        set(h,'AutoScale','on', 'AutoScaleFactor',1)

        h2 = quiver3(x, y, z, vecY(1), vecY(2), vecY(3), 'Color', 'g', 'LineWidth', 2);
        set(h2,'AutoScale','on', 'AutoScaleFactor',1)

        h3 = quiver3(x, y, z, vecZ(1), vecZ(2), vecZ(3), 'Color', 'b', 'LineWidth', 2);
        set(h3,'AutoScale','on', 'AutoScaleFactor',1)
    end
end

