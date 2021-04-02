function ang = VecAngle(v1,v2,degs)
    % Calculate the angle between two vectors

if nargin == 2
    degs = false;
end

if degs
    ang = acosd(dot(v1,v2)/(norm(v1)*norm(v2)));
else
    ang = acos(dot(v1,v2)/(norm(v1)*norm(v2)));
end