function H = calcHWithRANSAC(p1, p2)
% Returns the homography that maps p2 to p1 under RANSAC.
% Pre-conditions:
%     Both p1 and p2 are nx2 matrices where each row is a feature point.
%     p1(i, :) corresponds to p2(i, :) for i = 1, 2, ..., n
%     n >= 4
% Post-conditions:
%     Returns H, a 3 x 3 homography matrix

    assert(all(size(p1) == size(p2)));  % input matrices are of equal size
    assert(size(p1, 2) == 2);  % input matrices each have two columns
    assert(size(p1, 1) >= 4);  % input matrices each have at least 4 rows

    % The following code computes a homography matrix using all feature points
    % of p1 and p2. Modify it to compute a homography matrix using the inliers
    % of p1 and p2 as determined by RANSAC.
    %
    % Your implementation should use the helper function calcH in two
    % places - 1) finding the homography between four point-pairs within
    % the RANSAC loop, and 2) finding the homography between the inliers
    % after the RANSAC loop.
    
    numItr = 100;
    maxDist = 3;
    maxInliers = 0;
    bestH = zeros(3,3);
    for i = 1:numItr
        randomIndices = randperm(size(p1,1),4);
        H = calcH(p1(randomIndices,:),p2(randomIndices,:));
        inlierCount = 0;
        for j = 1 : size(p1,1)
            if sum(sum(([p1(j,:),1]' - H*[p2(j,:),1]').^2)) < maxDist
               inlierCount = inlierCount + 1;
            end
        end
        if (inlierCount > maxInliers)
           bestH = H;
           maxInliers = inlierCount;
        end        
    end
    inliers = zeros(maxInliers,1);
    j = 1;
    for i = 1 : size(p1,1)
       if sum(sum(([p1(i,:),1]' - bestH*[p2(i,:),1]').^2)) < maxDist
           inliers(j) = i;
           j = j + 1;
       end
    end

    H = calcH(p1(inliers,:), p2(inliers,:));

end

% Returns the homography that maps p2 to p1 in the least squares sense
% Pre-conditions:
%     Both p1 and p2 are nx2 matrices where each row is a feature point.
%     p1(i, :) corresponds to p2(i, :) for i = 1, 2, ..., n
%     n >= 4
% Post-conditions:
%     Returns H, a 3 x 3 homography matrix
    
    assert(all(size(p1) == size(p2)));
    assert(size(p1, 2) == 2);
    
    n = size(p1, 1);
    if n < 4
        error('Not enough points');
    end
    H = zeros(3, 3);  % Homography matrix to be returned

    A = zeros(n*3,9);
    b = zeros(n*3,1);
    for i=1:n
        A(3*(i-1)+1,1:3) = [p2(i,:),1];
        A(3*(i-1)+2,4:6) = [p2(i,:),1];
        A(3*(i-1)+3,7:9) = [p2(i,:),1];
        b(3*(i-1)+1:3*(i-1)+3) = [p1(i,:),1];
    end
    x = (A\b)';
    H = [x(1:3); x(4:6); x(7:9)];

end
