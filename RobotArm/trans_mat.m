function A = trans_mat(rotZ1, rotX, rotZ2)
    
    Az1 = [cos(rotZ1) -sin(rotZ1) 0; sin(rotZ1) cos(rotZ1) 0; 0 0 1];
    Ax = [1 0 0; 0 cos(rotX) -sin(rotX); 0 sin(rotX) cos(rotX)];
    Az2 = [cos(rotZ2) -sin(rotZ2) 0; sin(rotZ2) cos(rotZ2) 0; 0 0 1];
    
    A = Az1*Ax*Az2;
    
end