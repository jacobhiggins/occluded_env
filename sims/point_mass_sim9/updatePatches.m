function patches = updatePatches(patches,p,xintersect,maxRange)
    num = patches.num;
    empty_odds = 0.5; % Odds of measuring empty space, i.e. ration of prob of false negative to true negative
    [x,y] = p.getPos();
    for i = 1:num
        
       x1 = patches.x(i,1); % left most x position of patch
       x2 = patches.x(i,2); % right most x position of patch
       y1 = patches.y(i,1); % lower most y position of patch
       y2 = patches.y(i,3); % upper most y position of patch
       xavg = (x1+x2)/2;
       yavg = (y1+y2)/2;
       d = norm([x-xavg,y-yavg],2); % Dist between robot and patch
       if xavg < xintersect && d < maxRange
          prob = patches.probs(i);
          patches.probs(i) = 1/(1 + (1-prob)/(empty_odds*prob));
          patches.color(i,:) = ones(3,1) - patches.probs(i)*ones(3,1);
       end
    end
end