function Mc = actuator_control(q,q_des,w,w_des,linear)

eul = dcm2eulerangles(quat2dcm(q));
eul_des = dcm2eulerangles(quat2dcm(q_des));

alpha = eul_des - eul;
alpha_d = w_des - w;

visorsInertia


