% Given series of x-y positions
% perform simple path integral to find distance
function d = int_dist(poss)
    poss1 = [poss; poss(end,:)];
    poss2 = [poss(1,:);poss];
    diffs = poss2 - poss1;
    d = sqrt( sum(diffs.^2,2) );
    d = sum(d,1);
end