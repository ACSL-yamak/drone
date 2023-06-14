function Uep = Uep(in1,in2,in3,in4)
%Uep
%    Uep = Uep(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/06/14 20:09:16

T1 = in1(14,:);
V1 = in3(1,:);
V2 = in3(2,:);
V3 = in3(3,:);
V4 = in3(4,:);
dT1 = in1(15,:);
ddXd4 = in2(:,12);
ddddXd1 = in2(:,17);
ddddXd2 = in2(:,18);
ddddXd3 = in2(:,19);
jx = in4(:,6);
jy = in4(:,7);
jz = in4(:,8);
m = in4(:,1);
o1 = in1(11,:);
o2 = in1(12,:);
o3 = in1(13,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
q3 = in1(4,:);
t2 = q0.*q1;
t3 = q0.*q2;
t4 = q1.*q3;
t5 = q2.*q3;
t6 = q0.^2;
t7 = q1.^2;
t9 = q2.^2;
t11 = q3.^2;
t14 = q0.*q3.*2.0;
t15 = q1.*q2.*2.0;
t16 = jx.*o1.*o3;
t17 = jy.*o2.*o3;
t18 = jz.*o1.*o3;
t19 = jz.*o2.*o3;
t20 = -V3;
t21 = -ddddXd2;
t22 = 1.0./jx;
t23 = 1.0./jy;
t24 = 1.0./m;
t32 = (o1.*q0)./2.0;
t33 = (o1.*q1)./2.0;
t34 = (o2.*q0)./2.0;
t35 = (o1.*q2)./2.0;
t36 = (o2.*q1)./2.0;
t37 = (o3.*q0)./2.0;
t38 = (o1.*q3)./2.0;
t39 = (o2.*q2)./2.0;
t40 = (o3.*q1)./2.0;
t41 = (o2.*q3)./2.0;
t42 = (o3.*q2)./2.0;
t43 = (o3.*q3)./2.0;
t8 = t6.^2;
t10 = t7.^2;
t12 = t9.^2;
t13 = t11.^2;
t29 = -t7;
t30 = -t9;
t31 = -t11;
t44 = -t18;
t45 = -t19;
t46 = -t35;
t47 = -t40;
t48 = -t41;
t49 = t2+t5;
t50 = t2.^2.*2.0;
t51 = t3.^2.*2.0;
t52 = t6.*t11.*2.0;
t53 = t7.*t9.*2.0;
t54 = t4.^2.*2.0;
t55 = t5.^2.*2.0;
t56 = dT1.*q0.*t24.*2.0;
t57 = dT1.*q1.*t24.*2.0;
t58 = dT1.*q2.*t24.*2.0;
t59 = dT1.*q3.*t24.*2.0;
t60 = T1.*o1.*q0.*t24;
t61 = T1.*o1.*q1.*t24;
t62 = T1.*o2.*q0.*t24;
t63 = T1.*o1.*q2.*t24;
t64 = T1.*o2.*q1.*t24;
t65 = T1.*o3.*q0.*t24;
t66 = T1.*o1.*q3.*t24;
t67 = T1.*o2.*q2.*t24;
t68 = T1.*o3.*q1.*t24;
t69 = T1.*o2.*q3.*t24;
t70 = T1.*o3.*q2.*t24;
t71 = T1.*o3.*q3.*t24;
t73 = T1.*t6.*t24;
t74 = T1.*t7.*t24;
t75 = T1.*t9.*t24;
t83 = T1.*t2.*t24.*2.0;
t84 = T1.*t3.*t24.*2.0;
t85 = T1.*t14.*t24;
t86 = T1.*t15.*t24;
t87 = T1.*t4.*t24.*2.0;
t88 = T1.*t5.*t24.*2.0;
t89 = t14+t15;
t95 = T1.*q1.*q2.*t24.*-2.0;
t109 = t33+t39+t43;
t25 = T1.*t8;
t26 = T1.*t10;
t27 = T1.*t12;
t28 = T1.*t13;
t72 = -t56;
t77 = T1.*t50;
t78 = T1.*t51;
t79 = T1.*t52;
t80 = T1.*t53;
t81 = T1.*t54;
t82 = T1.*t55;
t90 = -t63;
t91 = -t65;
t92 = -t69;
t93 = -t70;
t94 = -t71;
t96 = -t87;
t97 = t89.^2;
t98 = T1.*t24.*t29;
t99 = T1.*t24.*t30;
t100 = T1.*t24.*t31;
t101 = t16+t44;
t102 = t17+t45;
t103 = q0.*t89.*4.0;
t104 = q1.*t89.*4.0;
t105 = q2.*t89.*4.0;
t106 = q3.*t89.*4.0;
t110 = t83+t88;
t111 = t85+t86;
t112 = t6+t11+t29+t30;
t113 = t6+t7+t30+t31;
t114 = t36+t37+t46;
t115 = t34+t38+t47;
t116 = t32+t42+t48;
t118 = t85+t95;
t137 = T1.*t24.*t109.*2.0;
t138 = q0.*t24.*t109.*2.0;
t139 = q1.*t24.*t109.*2.0;
t140 = q2.*t24.*t109.*2.0;
t205 = t8+t10+t12+t13+t50+t51+t52+t53+t54+t55;
t117 = t84+t96;
t119 = t113.^2;
t120 = q0.*t113.*4.0;
t121 = q1.*t113.*4.0;
t122 = q2.*t113.*4.0;
t123 = q3.*t113.*4.0;
t124 = 1.0./t112;
t125 = 1.0./t113;
t144 = T1.*t24.*t114.*2.0;
t145 = T1.*t24.*t115.*2.0;
t146 = T1.*t24.*t116.*2.0;
t147 = q0.*t24.*t115.*2.0;
t148 = q1.*t24.*t114.*2.0;
t149 = q0.*t24.*t116.*2.0;
t150 = q2.*t24.*t114.*2.0;
t151 = q1.*t24.*t116.*2.0;
t152 = q2.*t24.*t115.*2.0;
t153 = q3.*t24.*t114.*2.0;
t154 = q3.*t24.*t115.*2.0;
t155 = -t140;
t156 = q3.*t24.*t116.*2.0;
t175 = t23.*t101.*t111;
t176 = t22.*t102.*t110;
t188 = t22.*t102.*t118;
t195 = t73+t74+t99+t100;
t196 = t73+t75+t98+t100;
t206 = t61+t67+t72+t94+t137;
t207 = 1.0./t205;
t210 = t25+t26+t27+t28+t77+t78+t79+t80+t81+t82;
t126 = 1.0./t119;
t127 = t125.^3;
t128 = t125.*2.0;
t161 = -t149;
t162 = -t153;
t166 = t97+t119;
t171 = t106+t120;
t172 = t105+t121;
t187 = t23.*t101.*t117;
t192 = -t175;
t194 = -t188;
t197 = t23.*t101.*t195;
t198 = t22.*t102.*t196;
t200 = t58+t62+t66+t68+t145;
t208 = t59+t64+t90+t91+t144;
t209 = t57+t60+t92+t93+t146;
t211 = 1.0./t210;
t215 = t109.*t206;
t216 = t115.*t206;
t217 = t116.*t206;
t232 = t147+t148+t155+t156;
t129 = q0.*t128;
t130 = q1.*t128;
t131 = q2.*t128;
t132 = q3.*t128;
t133 = t2.*t126.*4.0;
t134 = t3.*t126.*4.0;
t135 = t4.*t126.*4.0;
t136 = t5.*t126.*4.0;
t142 = q0.*q3.*t126.*8.0;
t143 = q1.*q2.*t126.*8.0;
t157 = t6.*t126.*4.0;
t158 = t7.*t126.*4.0;
t159 = t9.*t126.*4.0;
t160 = t11.*t126.*4.0;
t165 = t89.*t126.*2.0;
t173 = q0.*t89.*t126.*-2.0;
t174 = q1.*t89.*t126.*-2.0;
t179 = 1.0./t166;
t181 = t2.*t89.*t127.*8.0;
t182 = t3.*t89.*t127.*8.0;
t183 = q0.*q3.*t89.*t127.*8.0;
t184 = q1.*q2.*t89.*t127.*8.0;
t185 = t4.*t89.*t127.*8.0;
t186 = t5.*t89.*t127.*8.0;
t193 = -t187;
t199 = -t198;
t212 = t109.*t200;
t213 = t114.*t200;
t214 = t115.*t200;
t218 = t109.*t209;
t219 = -t215;
t220 = t114.*t208;
t221 = t115.*t208;
t222 = t114.*t209;
t223 = t116.*t208;
t224 = t116.*t209;
t233 = t139+t150+t154+t161;
t234 = t138+t151+t152+t162;
t235 = dT1.*t232;
t141 = -t136;
t163 = -t157;
t164 = -t158;
t169 = q2.*t165;
t170 = q3.*t165;
t180 = t179.^2;
t189 = -t181;
t190 = -t183;
t191 = -t184;
t203 = t131+t174;
t204 = t132+t173;
t225 = -t220;
t226 = -t222;
t227 = -t223;
t228 = t134+t135+t186;
t236 = dT1.*t233;
t237 = dT1.*t234;
t238 = -t235;
t201 = t129+t170;
t202 = t130+t169;
t229 = t134+t135+t189;
t230 = t133+t141+t182;
t231 = t133+t141+t185;
t239 = t128+t160+t163+t190;
t240 = t128+t159+t164+t191;
t241 = V1+ddddXd3+t176+t193+t214+t219+t224+t225+t237;
t242 = V2+ddddXd1+t194+t197+t212+t216+t226+t227+t238;
t243 = t20+t21+t192+t199+t213+t217+t218+t221+t236;
et1 = V4+ddXd4+t115.*((o1.*t119.*t179.*t201)./2.0+(o2.*t119.*t179.*t204)./2.0-(o3.*t119.*t179.*t203)./2.0+t114.*t122.*t179.*t201+t115.*t122.*t179.*t202+t116.*t122.*t179.*t203-t109.*t119.*t179.*t230-t114.*t119.*t179.*t228-t116.*t119.*t179.*t240-t115.*t119.*t179.*(t143+t165+t9.*t89.*t127.*8.0)-q2.*t109.*t113.*t179.*t204.*4.0-t109.*t119.*t180.*t204.*(t104-t122)+t114.*t119.*t180.*t201.*(t104-t122)+t115.*t119.*t180.*t202.*(t104-t122)+t116.*t119.*t180.*t203.*(t104-t122));
et2 = t114.*(o1.*t119.*t179.*t202.*(-1.0./2.0)+(o2.*t119.*t179.*t203)./2.0+(o3.*t119.*t179.*t204)./2.0+t114.*t123.*t179.*t201+t115.*t123.*t179.*t202+t116.*t123.*t179.*t203-t115.*t119.*t179.*t228+t116.*t119.*t179.*t231+t109.*t119.*t179.*t239-t114.*t119.*t179.*(t142+t165+t11.*t89.*t127.*8.0)-q3.*t109.*t113.*t179.*t204.*4.0-t109.*t119.*t180.*t204.*(t103-t123)+t114.*t119.*t180.*t201.*(t103-t123)+t115.*t119.*t180.*t202.*(t103-t123)+t116.*t119.*t180.*t203.*(t103-t123));
et3 = t116.*(o2.*t119.*t179.*t201.*(-1.0./2.0)+(o1.*t119.*t179.*t204)./2.0+(o3.*t119.*t179.*t202)./2.0+t109.*t121.*t179.*t204-t109.*t119.*t179.*t229+t114.*t119.*t179.*t231-t115.*t119.*t179.*t240+t116.*t119.*t179.*(t143+t165-t7.*t89.*t127.*8.0)-q1.*t113.*t114.*t179.*t201.*4.0-q1.*t113.*t115.*t179.*t202.*4.0-q1.*t113.*t116.*t179.*t203.*4.0-t109.*t119.*t172.*t180.*t204+t114.*t119.*t172.*t180.*t201+t115.*t119.*t172.*t180.*t202+t116.*t119.*t172.*t180.*t203);
et4 = t109.*((o1.*t119.*t179.*t203)./2.0+(o2.*t119.*t179.*t202)./2.0+(o3.*t119.*t179.*t201)./2.0+t114.*t120.*t179.*t201+t115.*t120.*t179.*t202+t116.*t120.*t179.*t203-t115.*t119.*t179.*t230-t116.*t119.*t179.*t229+t114.*t119.*t179.*t239+t109.*t119.*t179.*(t142+t165-t6.*t89.*t127.*8.0)-q0.*t109.*t113.*t179.*t204.*4.0+t109.*t119.*t171.*t180.*t204-t114.*t119.*t171.*t180.*t201-t115.*t119.*t171.*t180.*t202-t116.*t119.*t171.*t180.*t203)-t22.*t102.*((q0.*t119.*t179.*t203)./2.0-(q2.*t119.*t179.*t201)./2.0-(q1.*t119.*t179.*t204)./2.0+(q3.*t119.*t179.*t202)./2.0)+t23.*t101.*((q0.*t119.*t179.*t202)./2.0+(q1.*t119.*t179.*t201)./2.0-(q2.*t119.*t179.*t204)./2.0-(q3.*t119.*t179.*t203)./2.0);
et5 = -((jx.*o1.*o2-jy.*o1.*o2).*((q0.*t119.*t179.*t201)./2.0-(q1.*t119.*t179.*t202)./2.0+(q2.*t119.*t179.*t203)./2.0-(q3.*t119.*t179.*t204)./2.0))./jz;
Uep = [m.*t112.*t207.*t241+m.*t207.*t243.*(t2-t5).*2.0+m.*t207.*t242.*(t3+t4).*2.0;m.*t211.*t243.*(jx.*t6+jx.*t9+jx.*t29+jx.*t31)+m.*t211.*t242.*(jx.*q0.*q3-jx.*q1.*q2).*2.0-m.*t211.*t241.*(jx.*t2+jx.*t5).*2.0;m.*t211.*t242.*(jy.*t6+jy.*t7+jy.*t30+jy.*t31)-m.*t211.*t243.*(jy.*q0.*q3+jy.*q1.*q2).*2.0-m.*t211.*t241.*(jy.*t3-jy.*t4).*2.0;(jz.*t124.*(et1+et2+et3+et4+et5).*(t8+t10+t12+t13+t50-t51+t52+t53-t54+t55+t2.*t5.*8.0))./(t6+t7+t9+t11)-jz.*m.*t49.*t113.*t124.*t211.*t242.*2.0+jz.*m.*t49.*t124.*t211.*t241.*(t3-t4).*4.0+jz.*m.*t49.*t124.*t211.*t243.*(q0.*q3+q1.*q2).*4.0];
end