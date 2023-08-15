function out1 = CSLC_6_R0TFdMd(in1,in2,in3,in4,in5,in6)
%CSLC_6_R0TFdMd
%    OUT1 = CSLC_6_R0TFdMd(IN1,IN2,IN3,IN4,IN5,IN6)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/08/15 20:21:02

%[R0'*Fd;Md] for (26)
R01_1 = in3(1);
R01_2 = in3(4);
R01_3 = in3(7);
R02_1 = in3(2);
R02_2 = in3(5);
R02_3 = in3(8);
R03_1 = in3(3);
R03_2 = in3(6);
R03_3 = in3(9);
R0d1_1 = in4(1);
R0d1_2 = in4(4);
R0d1_3 = in4(7);
R0d2_1 = in4(2);
R0d2_2 = in4(5);
R0d2_3 = in4(8);
R0d3_1 = in4(3);
R0d3_2 = in4(6);
R0d3_3 = in4(9);
ddx0d1 = in2(7,:);
ddx0d2 = in2(8,:);
ddx0d3 = in2(9,:);
do0d1 = in2(16,:);
do0d2 = in2(17,:);
do0d3 = in2(18,:);
dx01 = in1(8,:);
dx02 = in1(9,:);
dx03 = in1(10,:);
dx0d1 = in2(4,:);
dx0d2 = in2(5,:);
dx0d3 = in2(6,:);
g = in5(:,1);
j01 = in5(:,3);
j02 = in5(:,4);
j03 = in5(:,5);
kdx01 = in6(:,5);
kdx02 = in6(:,6);
kdx03 = in6(:,7);
ko0 = in6(:,8);
kr0 = in6(:,4);
kx01 = in6(:,1);
kx02 = in6(:,2);
kx03 = in6(:,3);
m0 = in5(:,2);
o01 = in1(11,:);
o02 = in1(12,:);
o03 = in1(13,:);
o0d1 = in2(13,:);
o0d2 = in2(14,:);
o0d3 = in2(15,:);
x01 = in1(1,:);
x02 = in1(2,:);
x03 = in1(3,:);
x0d1 = in2(1,:);
x0d2 = in2(2,:);
x0d3 = in2(3,:);
t2 = R01_1.*R0d1_1;
t3 = R01_1.*R0d1_2;
t4 = R01_2.*R0d1_1;
t5 = R01_1.*R0d1_3;
t6 = R01_2.*R0d1_2;
t7 = R01_3.*R0d1_1;
t8 = R01_2.*R0d1_3;
t9 = R01_3.*R0d1_2;
t10 = R01_3.*R0d1_3;
t11 = R02_1.*R0d2_1;
t12 = R02_1.*R0d2_2;
t13 = R02_2.*R0d2_1;
t14 = R02_1.*R0d2_3;
t15 = R02_2.*R0d2_2;
t16 = R02_3.*R0d2_1;
t17 = R02_2.*R0d2_3;
t18 = R02_3.*R0d2_2;
t19 = R02_3.*R0d2_3;
t20 = R03_1.*R0d3_1;
t21 = R03_1.*R0d3_2;
t22 = R03_2.*R0d3_1;
t23 = R03_1.*R0d3_3;
t24 = R03_2.*R0d3_2;
t25 = R03_3.*R0d3_1;
t26 = R03_2.*R0d3_3;
t27 = R03_3.*R0d3_2;
t28 = R03_3.*R0d3_3;
t29 = -ddx0d1;
t30 = -ddx0d2;
t31 = -dx0d1;
t32 = -dx0d2;
t33 = -dx0d3;
t34 = -x0d1;
t35 = -x0d2;
t36 = -x0d3;
t37 = dx01+t31;
t38 = dx02+t32;
t39 = dx03+t33;
t40 = t34+x01;
t41 = t35+x02;
t42 = t36+x03;
t50 = t2+t11+t20;
t51 = t3+t12+t21;
t52 = t4+t13+t22;
t53 = t5+t14+t23;
t54 = t6+t15+t24;
t55 = t7+t16+t25;
t56 = t8+t17+t26;
t57 = t9+t18+t27;
t58 = t10+t19+t28;
t43 = kdx01.*t37;
t44 = kdx02.*t38;
t45 = kdx03.*t39;
t46 = kx01.*t40;
t47 = kx02.*t41;
t48 = kx03.*t42;
t60 = o0d1.*t50;
t61 = o0d1.*t52;
t62 = o0d2.*t51;
t63 = o0d1.*t55;
t64 = o0d2.*t54;
t65 = o0d3.*t53;
t66 = o0d2.*t57;
t67 = o0d3.*t56;
t68 = o0d3.*t58;
t49 = -t45;
t59 = -t48;
t69 = t29+t43+t46;
t70 = t30+t44+t47;
t72 = t60+t62+t65;
t73 = t61+t64+t67;
t74 = t63+t66+t68;
t71 = ddx0d3+g+t49+t59;
t75 = R01_2.*j02.*t72;
t76 = R01_3.*j03.*t72;
t77 = R01_1.*j01.*t73;
t78 = R02_2.*j02.*t72;
t79 = R01_3.*j03.*t73;
t80 = R02_3.*j03.*t72;
t81 = R01_1.*j01.*t74;
t82 = R02_1.*j01.*t73;
t83 = R01_2.*j02.*t74;
t84 = R03_2.*j02.*t72;
t85 = R02_3.*j03.*t73;
t86 = R03_3.*j03.*t72;
t87 = R02_1.*j01.*t74;
t88 = R03_1.*j01.*t73;
t89 = R02_2.*j02.*t74;
t90 = R03_3.*j03.*t73;
t91 = R03_1.*j01.*t74;
t92 = R03_2.*j02.*t74;
t93 = -t77;
t94 = -t81;
t95 = -t82;
t96 = -t83;
t97 = -t87;
t98 = -t88;
t99 = -t89;
t100 = -t91;
t101 = -t92;
t102 = t75+t93;
t103 = t76+t94;
t104 = t78+t95;
t105 = t79+t96;
t106 = t80+t97;
t107 = t84+t98;
t108 = t85+t99;
t109 = t86+t100;
t110 = t90+t101;
mt1 = [-R01_1.*m0.*t69-R02_1.*m0.*t70+R03_1.*m0.*t71;-R01_2.*m0.*t69-R02_2.*m0.*t70+R03_2.*m0.*t71;-R01_3.*m0.*t69-R02_3.*m0.*t70+R03_3.*m0.*t71;-kr0.*(t8./2.0-t9./2.0+t17./2.0-t18./2.0+t26./2.0-t27./2.0)-ko0.*(o01-t72)+o0d1.*(R0d1_1.*t105+R0d2_1.*t108+R0d3_1.*t110)+o0d2.*(R0d1_2.*t105+R0d2_2.*t108+R0d3_2.*t110)+o0d3.*(R0d1_3.*t105+R0d2_3.*t108+R0d3_3.*t110)+do0d1.*(j01.*t2+j01.*t11+j01.*t20)+do0d2.*(j01.*t3+j01.*t12+j01.*t21)+do0d3.*(j01.*t5+j01.*t14+j01.*t23)];
mt2 = [kr0.*(t5./2.0-t7./2.0+t14./2.0-t16./2.0+t23./2.0-t25./2.0)-ko0.*(o02-t73)-o0d1.*(R0d1_1.*t103+R0d2_1.*t106+R0d3_1.*t109)-o0d2.*(R0d1_2.*t103+R0d2_2.*t106+R0d3_2.*t109)-o0d3.*(R0d1_3.*t103+R0d2_3.*t106+R0d3_3.*t109)+do0d1.*(j02.*t4+j02.*t13+j02.*t22)+do0d2.*(j02.*t6+j02.*t15+j02.*t24)+do0d3.*(j02.*t8+j02.*t17+j02.*t26)];
mt3 = [-kr0.*(t3./2.0-t4./2.0+t12./2.0-t13./2.0+t21./2.0-t22./2.0)-ko0.*(o03-t74)+o0d1.*(R0d1_1.*t102+R0d2_1.*t104+R0d3_1.*t107)+o0d2.*(R0d1_2.*t102+R0d2_2.*t104+R0d3_2.*t107)+o0d3.*(R0d1_3.*t102+R0d2_3.*t104+R0d3_3.*t107)+do0d1.*(j03.*t7+j03.*t16+j03.*t25)+do0d2.*(j03.*t9+j03.*t18+j03.*t27)+do0d3.*(j03.*t10+j03.*t19+j03.*t28)];
out1 = [mt1;mt2;mt3];
end
