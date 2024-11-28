function isInHull = inConvexhull(point, hullCenter, hullNormal)
%INCONVEXHULL check if a value is in the convex hull
%        point: points to be indentified
%        hullCenter: points of triangularzation center
%        hullNormal: points of triangularzation normal
% see: https://math.stackexchange.com/a/870810
    diffVec = hullCenter - point;
    isInHull = all(dot(diffVec, hullNormal, 2) >= 0);
end

