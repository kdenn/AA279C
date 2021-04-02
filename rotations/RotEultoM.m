function R = RotEultoM(eul)
% Transform euler angles into a rotation matrix

	R = R3(eul(1))*R1(eul(2))*R3(eul(3));
end