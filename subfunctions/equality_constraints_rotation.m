function [Aeq,beq] = equality_constraints_rotation(RT,Rc,k,J)

R = RT*Rc;

RPY = rotm2eul(R,'XYZ');
thetax = RPY(1); thetay = RPY(2); thetaz = RPY(3);

thetadot = thetadot_zrot(thetax,thetay,thetaz,k);


wc = [[1;0;0],rotx(thetax)*[0;1;0], ...
    rotx(thetax)*roty(thetay)*[0;0;1]]*thetadot;


% for rotation about z axis:
beq = [wc(1);wc(2)];
Aeq = [J(1:2,:),zeros(2,2)];

end