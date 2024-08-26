function difference = circularDiff(value1, value2)
%CIRCULARDIFF Takes the circular difference between the value1 and value2, 
    diff = value1 - value2;
    diff_pos = diff + 2 * pi;
    diff_neg = diff - 2 * pi;
    
    difference = diff;
    abs_mask = abs(diff_pos) < abs(diff);
    difference(abs_mask) = diff_pos(abs_mask);
    abs_mask2 = abs(diff_neg) < abs(diff);
    difference(abs_mask2) = diff_neg(abs_mask2);
end

