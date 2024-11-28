function visualAbstractArm(ax, TShoulder, TShoulder_sym, TCenter, TElbow, TWrist, TEE, options)
    arguments
        ax 
        TShoulder      % the right shoulder 
        TShoulder_sym  % the left symmetric shoulder
        TCenter       % the center front on chest
        TElbow 
        TWrist 
        TEE 
        options.radius(1,1) double {mustBePositive} = 0.01
        options.color = [1.0000 0.8000 0.6000]
    end
    
    ax.NextPlot = 'add';

    plot3([TShoulder(1,4),TShoulder_sym(1,4)],[TShoulder(2,4),TShoulder_sym(2,4)],[TShoulder(3,4),TShoulder_sym(3,4)], 'Color',options.color, 'LineWidth', 5);
    plot3([TShoulder(1,4),TCenter(1,4)],[TShoulder(2,4),TCenter(2,4)],[TShoulder(3,4),TCenter(3,4)], 'Color',options.color, 'LineWidth', 5);
    plot3([TShoulder_sym(1,4),TCenter(1,4)],[TShoulder_sym(2,4),TCenter(2,4)],[TShoulder_sym(3,4),TCenter(3,4)], 'Color',options.color, 'LineWidth', 5);


    [X_Shoulder,Y_Shoulder,Z_Shoulder] = generateSphere(options.radius*2, TShoulder(1:3, 4));
    surf(ax, X_Shoulder, Y_Shoulder, Z_Shoulder,'FaceColor',options.color ,'FaceAlpha', 0.8, 'EdgeAlpha', 0.3)
    plot3([TShoulder(1,4),TElbow(1,4)],[TShoulder(2,4),TElbow(2,4)],[TShoulder(3,4),TElbow(3,4)], 'Color',options.color, 'LineWidth', 5);

    [X_Elbow, Y_Elbow, Z_Elbow] = generateCylinder(options.radius*2, TElbow(1:3, 4), TElbow(1:3, 1:3));
    surf(ax, X_Elbow, Y_Elbow, Z_Elbow,'FaceColor',options.color ,'FaceAlpha',0.8, 'EdgeAlpha', 0.3)
    plot3([TElbow(1,4),TWrist(1,4)],[TElbow(2,4),TWrist(2,4)],[TElbow(3,4),TWrist(3,4)], 'Color',options.color, 'LineWidth', 5);

    [X_Wrist,Y_Wrist,Z_Wrist] = generateSphere(options.radius*2, TWrist(1:3, 4));
    surf(ax, X_Wrist, Y_Wrist, Z_Wrist,'FaceColor',options.color ,'FaceAlpha', 0.8, 'EdgeAlpha', 0.3)
    plot3([TWrist(1,4),TEE(1,4)],[TWrist(2,4),TEE(2,4)],[TWrist(3,4),TEE(3,4)], 'Color',options.color, 'LineWidth', 5);

    [X_EE,Y_EE, Z_EE] = generateSphere(options.radius, TEE(1:3, 4));
    surf(ax, X_EE, Y_EE, Z_EE,'FaceColor',options.color ,'FaceAlpha',0.8, 'EdgeColor','none')
end

function [X,Y,Z] = generateSphere(radius, center)
    [X,Y,Z] = sphere(10);
    X = X * radius + center(1);
    Y = Y * radius + center(2);
    Z = Z * radius + center(3);
end

function [X,Y,Z] = generateCylinder(radius, center, orientation)
    [X,Y,Z] = cylinder(radius);
    Z = Z * radius;
    
    for i = 1:numel(X)
        point = [X(i);Y(i);Z(i)];
        point = orientation * point;
        X(i) = point(1);
        Y(i) = point(2);
        Z(i) = point(3);
    end

    X = X + center(1);
    Y = Y + center(2);
    Z = Z + center(3);
end