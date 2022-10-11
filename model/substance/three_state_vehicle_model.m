function dx = three_state_vehicle_model(in1,in2,P)
%THREE_STATE_VEHICLE_MODEL
%    DX = THREE_STATE_VEHICLE_MODEL(IN1,IN2,P)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    11-Jul-2022 18:29:38

q3 = in1(6,:);
u1 = in2(1,:);
u2 = in2(2,:);
u6 = in2(6,:);
t2 = cos(q3);
t3 = sin(q3);
t4 = u1.^2;
t5 = u2.^2;
t6 = t2.*u1;
t7 = t3.*u2;
t8 = t4+t5;
t9 = t6+t7;
t11 = sqrt(t8);
t10 = sign(t9);
dx = [t2.*t10.*t11;t3.*t10.*t11;0.0;0.0;0.0;u6];