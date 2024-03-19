function V1 = Vf_SuspendedLoad(obj,in1,in2,in3,in4)
%VF_SUSPENDEDLOAD
%    V1 = VF_SUSPENDEDLOAD(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    19-Aug-2020 17:36:36

Xd3 = in2(:,3);
dXd3 = in2(:,7);
dpl3 = in1(13,:);
f11 = in4(:,1);
f12 = in4(:,2);
pl3 = in1(10,:);
t2 = f11.*f12;
t3 = f12.^2;
t4 = -dpl3;
t5 = -pl3;
t6 = -t3;
t7 = dXd3+t4;
t8 = Xd3+t5;
t9 = f11+t6;
t10 = f11.*t9;
t11 = f12.*t9;
t12 = -t10;
t13 = t2+t11;
t14 = f12.*t13;
t15 = t12+t14;
V1 = [f11.*t8+f12.*t7,-t2.*t8+t7.*t9,-t7.*t13+t8.*t12,-t7.*(t10-t14)+f11.*t8.*t13,t7.*(f11.*t13+f12.*(t10-t14))+f11.*t8.*(t10-t14)];