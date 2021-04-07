function [com,coord,area,norms] = spacecraftGeom(f,v)
% Calculate the barycenter coordinates, surface area, and surface normals
% f: faces
% v: vertices

N = size(f,1);

com = mean(v);

coord = zeros(N,3);
area = zeros(N,1);
norms = zeros(N,3);
for i = 1:N
    verts = v(f(i,:)',:);
    coord(i,:) = mean(verts,1);
    area(i) = getArea(verts);
    norms(i,:) = unitVec(coord(i,:)-com);
end

end

function A = getArea(verts)
    d1 = norm(verts(1,:)-verts(2,:));
    d2 = norm(verts(1,:)-verts(4,:));
    d3 = norm(verts(1,:)-verts(4,:));
    if d1 == d2
        A = d1 * d3;
    else
        A = d1 * d2;
    end
end