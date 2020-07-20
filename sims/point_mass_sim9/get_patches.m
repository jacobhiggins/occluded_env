function patches = get_patches(hls,hws)
%     probs = 0;
%     probs = 0:0.1:1;
    probs = 0.5*ones(100,1);
%     probs = [0;0;0;0;0;0;1;1;1;1;1];
%     probs = 1:-0.1:0;
%     probs = zeros(11,1);
    num = length(probs);
    patches.probs = probs;
    patches.num = num;
    patches.x = [];
    patches.y = [];
    patches.color = [];
    x = hws(1);
    delx = (hls(2)-hws(1)-hws(3))/num;
    ylower = hls(1)-hws(2);
    yupper = hls(1);
    for i = 1:num
        x1 = x + (i-1)*delx;
        x2 = x + i*delx;
        patches.x = [patches.x;x1,x2,x2,x1];
        patches.y = [patches.y;ylower,ylower,yupper,yupper];
        patches.color = [patches.color;1-probs(i),1-probs(i),1-probs(i)];
    end
    patches.delx = delx;
end