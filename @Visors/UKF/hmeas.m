function z = hmeas(xACAF,rSat,R_ACAF2cam,cam)
% Convert 3D ACAF coordinate to 2D im
% xACI = RotVert(xACAF',ast.alp,ast.del,ast.W_0,ast.W_d,JD,false);
% z = (transPt32(xACI,rSat,R,cam))';
% xACI = ast.RotACAF2ACI(JD) * xACAF;
t = R_ACAF2cam * -rSat;
z = cam.ProjectiveTransform(xACAF,R_ACAF2cam,t);
end