function active_faces = get_active_faces(faces, ref_vec)
% Returns a struct only containing the 'active' or 'wet' faces. I.e. the
% faces that are lit (for SRP torques) or faces that can impact with
% atmosphere particles (for drag torques)

% Number of faces
N = faces.N; 

% Nx3 matrix where each row is ref_vec
ref_vec_arr = repmat(ref_vec', N, 1);

% Nx1 vector of dot products between ref vector and unit normal of face
dot_prods = dot(ref_vec_arr, faces.norm, 2); 

% Indices of 'active' faces
ind_active = dot_prods > 0;

% Return struct
active_faces = struct(); 
active_faces.bary = faces.bary(ind_active,:);
active_faces.area = faces.area(ind_active);
active_faces.norm = faces.norm(ind_active,:);
active_faces.N = length(active_faces.area);
end