function p = get_patch_prob(xp,patches)
    xl = patches.x(1,1);
    delx = patches.delx;
    num = patches.num;
    i = floor((xp - xl)/delx);
    if i < 0 || i > num-1
       p = 0; 
    else
       p = patches.probs(i+1); 
    end
end