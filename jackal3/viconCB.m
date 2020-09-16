function viconCB(msg,~)
    global r;
    r.x = msg.Transform.Translation.X;
    r.y = msg.Transform.Translation.Y;
end