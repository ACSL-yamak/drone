function out1 = Jacobian_quat(in1,in2)
%Jacobian_quat
%    OUT1 = Jacobian_quat(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    23-May-2022 13:35:02

p6 = in2(:,6);
p7 = in2(:,7);
p8 = in2(:,8);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
x11 = in1(11,:);
x12 = in1(12,:);
x13 = in1(13,:);
t2 = p6.*x11;
t3 = p7.*x12;
t4 = p8.*x13;
t5 = 1.0./p6;
t6 = 1.0./p7;
t7 = 1.0./p8;
t9 = x1./2.0;
t10 = x2./2.0;
t11 = x3./2.0;
t12 = x4./2.0;
t13 = x11./2.0;
t14 = x12./2.0;
t15 = x13./2.0;
t8 = -t4;
t16 = -t10;
t17 = -t11;
t18 = -t12;
t19 = -t13;
t20 = -t14;
t21 = -t15;
mt1 = [0.0,t13,t14,t15,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t19,0.0,t21,t14,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t20,t15,0.0,t19,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t21,t20,t13,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t16,t9,t12,t17,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6.*(t4-p6.*x13),-t7.*(t3-p6.*x12),t17,t18,t9,t10,0.0,0.0,0.0,0.0,0.0,0.0,-t5.*(t4-p7.*x13),0.0,t7.*(t2-p7.*x11),t18,t11,t16,t9,0.0,0.0,0.0,0.0];
mt2 = [0.0,0.0,t5.*(t3-p8.*x12),-t6.*(t2-p8.*x11),0.0];
out1 = reshape([mt1,mt2],13,13);