function attMat = constantPoint(style,t,deg)
switch style
    case '+x'
        q0 = ones(length(t),1);
        q1 = zeros(length(t),1);
        q2 = zeros(length(t),1);
        q3 = zeros(length(t),1);
    case '+y'
        
    case '+z'
        
    case 'ang' %currently assuming pitch angle
        ang = deg*pi/180;
        q0 = cos(ang/2)*ones(length(t),1);
        q2 = zeros(length(t),1); %x-roll
        q1 = sin(ang/2)*ones(length(t),1); %y-pitch
        q3 = zeros(length(t),1); %z-yaw
        
        
end

wdes = zeros(length(t),3);

attMat = [q0 q1 q2 q3 wdes];

end
        