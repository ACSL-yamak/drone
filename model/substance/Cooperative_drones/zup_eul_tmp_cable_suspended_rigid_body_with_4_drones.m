function dX = zup_eul_tmp_cable_suspended_rigid_body_with_4_drones(in1,in2,in3,in4)
%ZUP_EUL_TMP_CABLE_SUSPENDED_RIGID_BODY_WITH_4_DRONES
%    dX = ZUP_EUL_TMP_CABLE_SUSPENDED_RIGID_BODY_WITH_4_DRONES(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/06/26 22:49:16

Mi1_1 = in2(2,:);
Mi1_2 = in2(3,:);
Mi1_3 = in2(4,:);
Mi2_1 = in2(6,:);
Mi2_2 = in2(7,:);
Mi2_3 = in2(8,:);
Mi3_1 = in2(10,:);
Mi3_2 = in2(11,:);
Mi3_3 = in2(12,:);
Mi4_1 = in2(14,:);
Mi4_2 = in2(15,:);
Mi4_3 = in2(16,:);
X4 = in1(4,:);
X5 = in1(5,:);
X6 = in1(6,:);
X7 = in1(7,:);
X8 = in1(8,:);
X9 = in1(9,:);
X10 = in1(10,:);
X11 = in1(11,:);
X12 = in1(12,:);
X13 = in1(13,:);
X14 = in1(14,:);
X15 = in1(15,:);
X16 = in1(16,:);
X17 = in1(17,:);
X18 = in1(18,:);
X19 = in1(19,:);
X20 = in1(20,:);
X21 = in1(21,:);
X22 = in1(22,:);
X23 = in1(23,:);
X24 = in1(24,:);
X25 = in1(25,:);
X26 = in1(26,:);
X27 = in1(27,:);
X28 = in1(28,:);
X29 = in1(29,:);
X30 = in1(30,:);
X31 = in1(31,:);
X32 = in1(32,:);
X33 = in1(33,:);
X34 = in1(34,:);
X35 = in1(35,:);
X36 = in1(36,:);
X37 = in1(37,:);
X38 = in1(38,:);
X39 = in1(39,:);
X40 = in1(40,:);
X41 = in1(41,:);
X42 = in1(42,:);
X43 = in1(43,:);
X44 = in1(44,:);
X45 = in1(45,:);
X46 = in1(46,:);
X47 = in1(47,:);
X48 = in1(48,:);
X49 = in1(49,:);
X50 = in1(50,:);
X51 = in1(51,:);
X52 = in1(52,:);
X53 = in1(53,:);
X54 = in1(54,:);
X55 = in1(55,:);
X56 = in1(56,:);
X57 = in1(57,:);
X58 = in1(58,:);
X59 = in1(59,:);
X60 = in1(60,:);
ddX1 = in4(1,:);
ddX2 = in4(2,:);
ddX3 = in4(3,:);
ddX4 = in4(4,:);
ddX5 = in4(5,:);
ddX6 = in4(6,:);
fi1 = in2(1,:);
fi2 = in2(5,:);
fi3 = in2(9,:);
fi4 = in2(13,:);
g = in3(:,1);
ji1_1 = in3(:,26);
ji1_2 = in3(:,27);
ji1_3 = in3(:,28);
ji2_1 = in3(:,29);
ji2_2 = in3(:,30);
ji2_3 = in3(:,31);
ji3_1 = in3(:,32);
ji3_2 = in3(:,33);
ji3_3 = in3(:,34);
ji4_1 = in3(:,35);
ji4_2 = in3(:,36);
ji4_3 = in3(:,37);
li1 = in3(:,18);
li2 = in3(:,19);
li3 = in3(:,20);
li4 = in3(:,21);
mi1 = in3(:,22);
mi2 = in3(:,23);
mi3 = in3(:,24);
mi4 = in3(:,25);
rho1_1 = in3(:,6);
rho1_2 = in3(:,7);
rho1_3 = in3(:,8);
rho2_1 = in3(:,9);
rho2_2 = in3(:,10);
rho2_3 = in3(:,11);
rho3_1 = in3(:,12);
rho3_2 = in3(:,13);
rho3_3 = in3(:,14);
rho4_1 = in3(:,15);
rho4_2 = in3(:,16);
rho4_3 = in3(:,17);
t2 = cos(X4);
t3 = cos(X5);
t4 = cos(X37);
t5 = cos(X38);
t6 = cos(X40);
t7 = cos(X41);
t8 = cos(X43);
t9 = cos(X44);
t10 = cos(X46);
t11 = cos(X47);
t12 = sin(X4);
t13 = sin(X5);
t14 = sin(X37);
t15 = sin(X38);
t16 = sin(X40);
t17 = sin(X41);
t18 = sin(X43);
t19 = sin(X44);
t20 = sin(X46);
t21 = sin(X47);
t22 = X10.^2;
t23 = X11.^2;
t24 = X12.^2;
t25 = X13.^2;
t26 = X14.^2;
t27 = X15.^2;
t28 = X16.^2;
t29 = X17.^2;
t30 = X18.^2;
t31 = X19.^2;
t32 = X20.^2;
t33 = X21.^2;
t34 = X22.^2;
t35 = X23.^2;
t36 = X24.^2;
t37 = -ddX1;
t38 = -ddX3;
t39 = 1.0./li1;
t40 = 1.0./li2;
t41 = 1.0./li3;
t42 = 1.0./li4;
t43 = 1.0./mi1;
t44 = 1.0./mi2;
t45 = 1.0./mi3;
t46 = 1.0./mi4;
t52 = X4./2.0;
t53 = X5./2.0;
t54 = X6./2.0;
t55 = X37./2.0;
t56 = X38./2.0;
t57 = X39./2.0;
t58 = X40./2.0;
t59 = X41./2.0;
t60 = X42./2.0;
t61 = X43./2.0;
t62 = X44./2.0;
t63 = X45./2.0;
t64 = X46./2.0;
t65 = X47./2.0;
t66 = X48./2.0;
t47 = 1.0./t3;
t48 = 1.0./t5;
t49 = 1.0./t7;
t50 = 1.0./t9;
t51 = 1.0./t11;
t67 = t25-1.0;
t68 = t26-1.0;
t69 = t27-1.0;
t70 = t28-1.0;
t71 = t29-1.0;
t72 = t30-1.0;
t73 = t31-1.0;
t74 = t32-1.0;
t75 = t33-1.0;
t76 = t34-1.0;
t77 = t35-1.0;
t78 = t36-1.0;
t79 = cos(t52);
t80 = cos(t53);
t81 = cos(t54);
t82 = cos(t55);
t83 = cos(t56);
t84 = cos(t57);
t85 = cos(t58);
t86 = cos(t59);
t87 = cos(t60);
t88 = cos(t61);
t89 = cos(t62);
t90 = cos(t63);
t91 = cos(t64);
t92 = cos(t65);
t93 = cos(t66);
t94 = sin(t52);
t95 = sin(t53);
t96 = sin(t54);
t97 = sin(t55);
t98 = sin(t56);
t99 = sin(t57);
t100 = sin(t58);
t101 = sin(t59);
t102 = sin(t60);
t103 = sin(t61);
t104 = sin(t62);
t105 = sin(t63);
t106 = sin(t64);
t107 = sin(t65);
t108 = sin(t66);
t109 = t22+t23;
t110 = t22+t24;
t111 = t23+t24;
t112 = t79.*t80.*t81;
t113 = t82.*t83.*t84;
t114 = t85.*t86.*t87;
t115 = t88.*t89.*t90;
t116 = t91.*t92.*t93;
t117 = t79.*t80.*t96;
t118 = t79.*t81.*t95;
t119 = t80.*t81.*t94;
t120 = t82.*t83.*t99;
t121 = t82.*t84.*t98;
t122 = t83.*t84.*t97;
t123 = t85.*t86.*t102;
t124 = t85.*t87.*t101;
t125 = t86.*t87.*t100;
t126 = t88.*t89.*t105;
t127 = t88.*t90.*t104;
t128 = t89.*t90.*t103;
t129 = t91.*t92.*t108;
t130 = t91.*t93.*t107;
t131 = t92.*t93.*t106;
t132 = t79.*t95.*t96;
t133 = t80.*t94.*t96;
t134 = t81.*t94.*t95;
t135 = t82.*t98.*t99;
t136 = t83.*t97.*t99;
t137 = t84.*t97.*t98;
t138 = t85.*t101.*t102;
t139 = t86.*t100.*t102;
t140 = t87.*t100.*t101;
t141 = t88.*t104.*t105;
t142 = t89.*t103.*t105;
t143 = t90.*t103.*t104;
t144 = t91.*t107.*t108;
t145 = t92.*t106.*t108;
t146 = t93.*t106.*t107;
t147 = t94.*t95.*t96;
t148 = t97.*t98.*t99;
t149 = t100.*t101.*t102;
t150 = t103.*t104.*t105;
t151 = t106.*t107.*t108;
t152 = -t132;
t153 = -t134;
t154 = -t135;
t155 = -t137;
t156 = -t138;
t157 = -t140;
t158 = -t141;
t159 = -t143;
t160 = -t144;
t161 = -t146;
t162 = t112+t147;
t163 = t118+t133;
t164 = t113+t148;
t165 = t121+t136;
t166 = t114+t149;
t167 = t124+t139;
t168 = t115+t150;
t169 = t127+t142;
t170 = t116+t151;
t171 = t130+t145;
t172 = t162.^2;
t173 = t163.^2;
t174 = t164.^2;
t175 = t165.^2;
t176 = t166.^2;
t177 = t167.^2;
t178 = t168.^2;
t179 = t169.^2;
t180 = t170.^2;
t181 = t171.^2;
t182 = t117+t153;
t183 = t119+t152;
t184 = t120+t155;
t185 = t122+t154;
t186 = t123+t157;
t187 = t125+t156;
t188 = t126+t159;
t189 = t128+t158;
t190 = t129+t161;
t191 = t131+t160;
t213 = t162.*t163.*2.0;
t214 = t164.*t165.*2.0;
t215 = t166.*t167.*2.0;
t216 = t168.*t169.*2.0;
t217 = t170.*t171.*2.0;
t192 = t182.^2;
t193 = t183.^2;
t194 = t184.^2;
t195 = t185.^2;
t196 = t186.^2;
t197 = t187.^2;
t198 = t188.^2;
t199 = t189.^2;
t200 = t190.^2;
t201 = t191.^2;
t202 = -t173;
t203 = -t175;
t204 = -t177;
t205 = -t179;
t206 = -t181;
t218 = t162.*t182.*2.0;
t219 = t162.*t183.*2.0;
t220 = t163.*t182.*2.0;
t221 = t163.*t183.*2.0;
t222 = t164.*t185.*2.0;
t223 = t165.*t184.*2.0;
t224 = t166.*t187.*2.0;
t225 = t167.*t186.*2.0;
t226 = t168.*t189.*2.0;
t227 = t169.*t188.*2.0;
t228 = t170.*t191.*2.0;
t229 = t171.*t190.*2.0;
t236 = t182.*t183.*2.0;
t237 = t184.*t185.*2.0;
t238 = t186.*t187.*2.0;
t239 = t188.*t189.*2.0;
t240 = t190.*t191.*2.0;
t207 = -t192;
t208 = -t193;
t209 = -t195;
t210 = -t197;
t211 = -t199;
t212 = -t201;
t230 = -t220;
t231 = -t221;
t232 = -t223;
t233 = -t225;
t234 = -t227;
t235 = -t229;
t241 = -t236;
t242 = t213+t236;
t243 = t218+t221;
t244 = t219+t220;
t245 = t214+t237;
t246 = t215+t238;
t247 = t216+t239;
t248 = t217+t240;
t249 = t213+t241;
t250 = t218+t231;
t251 = t219+t230;
t252 = t222+t232;
t253 = t224+t233;
t254 = t226+t234;
t255 = t228+t235;
t256 = rho1_1.*t242;
t257 = rho1_1.*t244;
t258 = rho1_2.*t242;
t259 = rho1_2.*t243;
t260 = rho1_3.*t243;
t261 = rho1_3.*t244;
t262 = rho2_1.*t242;
t263 = rho2_1.*t244;
t264 = rho2_2.*t242;
t265 = rho2_2.*t243;
t266 = rho2_3.*t243;
t267 = rho2_3.*t244;
t268 = rho3_1.*t242;
t269 = rho3_1.*t244;
t270 = rho3_2.*t242;
t271 = rho3_2.*t243;
t272 = rho3_3.*t243;
t273 = rho3_3.*t244;
t274 = rho4_1.*t242;
t275 = rho4_1.*t244;
t276 = rho4_2.*t242;
t277 = rho4_2.*t243;
t278 = rho4_3.*t243;
t279 = rho4_3.*t244;
t304 = X10.*X11.*t243;
t305 = X10.*X11.*t244;
t306 = X10.*X12.*t242;
t307 = X10.*X12.*t243;
t308 = X11.*X12.*t242;
t309 = X11.*X12.*t244;
t310 = X13.*X14.*fi1.*t245;
t311 = X13.*X15.*fi1.*t245;
t312 = X16.*X17.*fi2.*t246;
t313 = X16.*X18.*fi2.*t246;
t314 = X19.*X20.*fi3.*t247;
t315 = X19.*X21.*fi3.*t247;
t316 = X22.*X23.*fi4.*t248;
t317 = X22.*X24.*fi4.*t248;
t363 = t109.*t242;
t364 = t110.*t244;
t365 = t111.*t243;
t366 = fi1.*t67.*t245;
t367 = fi2.*t70.*t246;
t368 = fi3.*t73.*t247;
t369 = fi4.*t76.*t248;
t377 = t172+t173+t207+t208;
t378 = t172+t192+t202+t208;
t379 = t172+t193+t202+t207;
t380 = t174+t194+t203+t209;
t381 = t176+t196+t204+t210;
t382 = t178+t198+t205+t211;
t383 = t180+t200+t206+t212;
t280 = rho1_1.*t250;
t281 = rho1_1.*t251;
t282 = rho1_2.*t249;
t283 = rho1_2.*t251;
t284 = rho1_3.*t249;
t285 = rho1_3.*t250;
t286 = rho2_1.*t250;
t287 = rho2_1.*t251;
t288 = rho2_2.*t249;
t289 = rho2_2.*t251;
t290 = rho2_3.*t249;
t291 = rho2_3.*t250;
t292 = rho3_1.*t250;
t293 = rho3_1.*t251;
t294 = rho3_2.*t249;
t295 = rho3_2.*t251;
t296 = rho3_3.*t249;
t297 = rho3_3.*t250;
t298 = rho4_1.*t250;
t299 = rho4_1.*t251;
t300 = rho4_2.*t249;
t301 = rho4_2.*t251;
t302 = rho4_3.*t249;
t303 = rho4_3.*t250;
t326 = X10.*X11.*t249;
t327 = X10.*X11.*t250;
t328 = X10.*X12.*t249;
t329 = X10.*X12.*t251;
t330 = X11.*X12.*t250;
t331 = X11.*X12.*t251;
t332 = X13.*X14.*fi1.*t252;
t333 = X14.*X15.*fi1.*t252;
t334 = X16.*X17.*fi2.*t253;
t335 = X17.*X18.*fi2.*t253;
t336 = X19.*X20.*fi3.*t254;
t337 = X20.*X21.*fi3.*t254;
t338 = X22.*X23.*fi4.*t255;
t339 = X23.*X24.*fi4.*t255;
t352 = -t304;
t353 = -t306;
t354 = -t309;
t370 = t109.*t251;
t371 = t110.*t250;
t372 = t111.*t249;
t373 = fi1.*t68.*t252;
t374 = fi2.*t71.*t253;
t375 = fi3.*t74.*t254;
t376 = fi4.*t77.*t255;
t388 = rho1_1.*t377;
t389 = rho1_1.*t378;
t390 = rho1_2.*t378;
t391 = rho1_2.*t379;
t392 = rho1_3.*t377;
t393 = rho1_3.*t379;
t394 = rho2_1.*t377;
t395 = rho2_1.*t378;
t396 = rho2_2.*t378;
t397 = rho2_2.*t379;
t398 = rho2_3.*t377;
t399 = rho2_3.*t379;
t400 = rho3_1.*t377;
t401 = rho3_1.*t378;
t402 = rho3_2.*t378;
t403 = rho3_2.*t379;
t404 = rho3_3.*t377;
t405 = rho3_3.*t379;
t406 = rho4_1.*t377;
t407 = rho4_1.*t378;
t408 = rho4_2.*t378;
t409 = rho4_2.*t379;
t410 = rho4_3.*t377;
t411 = rho4_3.*t379;
t412 = X10.*X11.*t377;
t413 = X10.*X11.*t379;
t414 = X10.*X12.*t378;
t415 = X10.*X12.*t379;
t416 = X11.*X12.*t377;
t417 = X11.*X12.*t378;
t418 = X13.*X15.*fi1.*t380;
t419 = X14.*X15.*fi1.*t380;
t420 = X16.*X18.*fi2.*t381;
t421 = X17.*X18.*fi2.*t381;
dX = ft_1({Mi1_1,Mi1_2,Mi1_3,Mi2_1,Mi2_2,Mi2_3,Mi3_1,Mi3_2,Mi3_3,Mi4_1,Mi4_2,Mi4_3,X10,X11,X12,X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,X26,X27,X28,X29,X30,X31,X32,X33,X34,X35,X36,X49,X50,X51,X52,X53,X54,X55,X56,X57,X58,X59,X60,X7,X8,X9,ddX1,ddX2,ddX4,ddX5,ddX6,fi1,fi2,fi3,fi4,g,ji1_1,ji1_2,ji1_3,ji2_1,ji2_2,ji2_3,ji3_1,ji3_2,ji3_3,ji4_1,ji4_2,ji4_3,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t10,t109,t11,t110,t111,t12,t13,t14,t15,t16,t17,t18,t19,t2,t20,t21,t256,t257,t258,t259,t260,t261,t262,t263,t264,t265,t266,t267,t268,t269,t270,t271,t272,t273,t274,t275,t276,t277,t278,t279,t280,t281,t282,t283,t284,t285,t286,t287,t288,t289,t290,t291,t292,t293,t294,t295,t296,t297,t298,t299,t3,t300,t301,t302,t303,t305,t307,t308,t310,t311,t312,t313,t314,t315,t316,t317,t326,t327,t328,t329,t330,t331,t332,t333,t334,t335,t336,t337,t338,t339,t352,t353,t354,t363,t364,t365,t366,t367,t368,t369,t37,t370,t371,t372,t373,t374,t375,t376,t377,t378,t379,t38,t380,t381,t382,t383,t388,t389,t39,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t4,t40,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t41,t410,t411,t412,t413,t414,t415,t416,t417,t418,t419,t42,t420,t421,t43,t44,t45,t46,t47,t48,t49,t5,t50,t51,t6,t69,t7,t72,t75,t78,t8,t9});
end
function dX = ft_1(ct)
[Mi1_1,Mi1_2,Mi1_3,Mi2_1,Mi2_2,Mi2_3,Mi3_1,Mi3_2,Mi3_3,Mi4_1,Mi4_2,Mi4_3,X10,X11,X12,X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,X26,X27,X28,X29,X30,X31,X32,X33,X34,X35,X36,X49,X50,X51,X52,X53,X54,X55,X56,X57,X58,X59,X60,X7,X8,X9,ddX1,ddX2,ddX4,ddX5,ddX6,fi1,fi2,fi3,fi4,g,ji1_1,ji1_2,ji1_3,ji2_1,ji2_2,ji2_3,ji3_1,ji3_2,ji3_3,ji4_1,ji4_2,ji4_3,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t10,t109,t11,t110,t111,t12,t13,t14,t15,t16,t17,t18,t19,t2,t20,t21,t256,t257,t258,t259,t260,t261,t262,t263,t264,t265,t266,t267,t268,t269,t270,t271,t272,t273,t274,t275,t276,t277,t278,t279,t280,t281,t282,t283,t284,t285,t286,t287,t288,t289,t290,t291,t292,t293,t294,t295,t296,t297,t298,t299,t3,t300,t301,t302,t303,t305,t307,t308,t310,t311,t312,t313,t314,t315,t316,t317,t326,t327,t328,t329,t330,t331,t332,t333,t334,t335,t336,t337,t338,t339,t352,t353,t354,t363,t364,t365,t366,t367,t368,t369,t37,t370,t371,t372,t373,t374,t375,t376,t377,t378,t379,t38,t380,t381,t382,t383,t388,t389,t39,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t4,t40,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t41,t410,t411,t412,t413,t414,t415,t416,t417,t418,t419,t42,t420,t421,t43,t44,t45,t46,t47,t48,t49,t5,t50,t51,t6,t69,t7,t72,t75,t78,t8,t9] = ct{:};
t422 = X19.*X21.*fi3.*t382;
t423 = X20.*X21.*fi3.*t382;
t424 = X22.*X24.*fi4.*t383;
t425 = X23.*X24.*fi4.*t383;
t429 = fi1.*t69.*t380;
t430 = fi2.*t72.*t381;
t431 = fi3.*t75.*t382;
t432 = fi4.*t78.*t383;
t433 = t109.*t378;
t434 = t110.*t377;
t435 = t111.*t379;
t341 = -t282;
t344 = -t288;
t347 = -t294;
t350 = -t300;
t355 = -t332;
t356 = -t333;
t357 = -t334;
t358 = -t335;
t359 = -t336;
t360 = -t337;
t361 = -t338;
t362 = -t339;
t384 = -t373;
t385 = -t374;
t386 = -t375;
t387 = -t376;
t426 = -t412;
t427 = -t415;
t428 = -t417;
t436 = t258+t285;
t437 = t264+t291;
t438 = t270+t297;
t439 = t276+t303;
t452 = -ddX5.*(t260-t281);
t454 = -ddX5.*(t266-t287);
t456 = -ddX5.*(t272-t293);
t458 = -ddX5.*(t278-t299);
t464 = t259+t388;
t465 = t256+t393;
t466 = t265+t394;
t467 = t262+t399;
t468 = t271+t400;
t469 = t268+t405;
t470 = t277+t406;
t471 = t274+t411;
t472 = t283+t392;
t473 = t289+t398;
t474 = t295+t404;
t475 = t301+t410;
t508 = -ddX5.*(t284-t389);
t509 = -ddX6.*(t280-t391);
t511 = -ddX5.*(t290-t395);
t512 = -ddX6.*(t286-t397);
t514 = -ddX5.*(t296-t401);
t515 = -ddX6.*(t292-t403);
t517 = -ddX5.*(t302-t407);
t518 = -ddX6.*(t298-t409);
t520 = ddX4.*(t261-t390);
t521 = ddX4.*(t267-t396);
t522 = ddX4.*(t273-t402);
t523 = ddX4.*(t279-t408);
t524 = t307+t370+t416;
t525 = t308+t371+t413;
t526 = t305+t372+t414;
t542 = t328+t354+t433;
t543 = t331+t352+t434;
t544 = t327+t353+t435;
t440 = ddX4.*t436;
t441 = ddX4.*t437;
t442 = ddX4.*t438;
t443 = ddX4.*t439;
t444 = t257+t341;
t446 = t263+t344;
t448 = t269+t347;
t450 = t275+t350;
t480 = ddX5.*t465;
t481 = ddX6.*t464;
t482 = ddX5.*t467;
t483 = ddX6.*t466;
t484 = ddX5.*t469;
t485 = ddX6.*t468;
t486 = ddX5.*t471;
t487 = ddX6.*t470;
t496 = ddX4.*t472;
t497 = ddX4.*t473;
t498 = ddX4.*t474;
t499 = ddX4.*t475;
t527 = rho1_1.*t526;
t528 = rho1_2.*t525;
t529 = rho1_3.*t524;
t530 = rho2_1.*t526;
t531 = rho2_2.*t525;
t532 = rho2_3.*t524;
t533 = rho3_1.*t526;
t534 = rho3_2.*t525;
t535 = rho3_3.*t524;
t536 = rho4_1.*t526;
t537 = rho4_2.*t525;
t538 = rho4_3.*t524;
t539 = t330+t363+t427;
t540 = t326+t364+t428;
t541 = t329+t365+t426;
t545 = t355+t366+t418;
t546 = t310+t384+t419;
t547 = t311+t356+t429;
t548 = t357+t367+t420;
t549 = t312+t385+t421;
t550 = t313+t358+t430;
t551 = t359+t368+t422;
t552 = t314+t386+t423;
t553 = t315+t360+t431;
t554 = t361+t369+t424;
t555 = t316+t387+t425;
t556 = t317+t362+t432;
t558 = rho1_1.*t544;
t560 = rho1_2.*t543;
t562 = rho1_3.*t542;
t564 = rho2_1.*t544;
t566 = rho2_2.*t543;
t568 = rho2_3.*t542;
t570 = rho3_1.*t544;
t572 = rho3_2.*t543;
t574 = rho3_3.*t542;
t576 = rho4_1.*t544;
t578 = rho4_2.*t543;
t580 = rho4_3.*t542;
t453 = ddX6.*t444;
t455 = ddX6.*t446;
t457 = ddX6.*t448;
t459 = ddX6.*t450;
t501 = -t480;
t503 = -t482;
t505 = -t484;
t507 = -t486;
t510 = -t496;
t513 = -t497;
t516 = -t498;
t519 = -t499;
t557 = rho1_1.*t541;
t559 = rho1_2.*t540;
t561 = rho1_3.*t539;
t563 = rho2_1.*t541;
t565 = rho2_2.*t540;
t567 = rho2_3.*t539;
t569 = rho3_1.*t541;
t571 = rho3_2.*t540;
t573 = rho3_3.*t539;
t575 = rho4_1.*t541;
t577 = rho4_2.*t540;
t579 = rho4_3.*t539;
t581 = -t560;
t583 = -t566;
t585 = -t572;
t587 = -t578;
t460 = -t453;
t461 = -t455;
t462 = -t457;
t463 = -t459;
t582 = -t561;
t584 = -t567;
t586 = -t573;
t588 = -t579;
t589 = ddX2+t452+t481+t510+t529+t557+t581;
t590 = ddX2+t454+t483+t513+t532+t563+t583;
t591 = ddX2+t456+t485+t516+t535+t569+t585;
t592 = ddX2+t458+t487+t519+t538+t575+t587;
t593 = t37+t440+t501+t509+t528+t558+t582;
t594 = t37+t441+t503+t512+t531+t564+t584;
t595 = t37+t442+t505+t515+t534+t570+t586;
t596 = t37+t443+t507+t518+t537+t576+t588;
t597 = g+t38+t460+t508+t520+t527+t559+t562;
t598 = g+t38+t461+t511+t521+t530+t565+t568;
t599 = g+t38+t462+t514+t522+t533+t571+t574;
t600 = g+t38+t463+t517+t523+t536+t577+t580;
mt1 = [X7;X8;X9;t47.*(X10.*t3+X12.*t2.*t13+X11.*t12.*t13);X11.*t2-X12.*t12;t47.*(X12.*t2+X11.*t12);ddX1;-ddX2;t38;ddX4;-ddX5;-ddX6;-X14.*X27+X15.*X26;X13.*X27-X15.*X25;-X13.*X26+X14.*X25;-X17.*X30+X18.*X29;X16.*X30-X18.*X28;-X16.*X29+X17.*X28;-X20.*X33+X21.*X32;X19.*X33-X21.*X31;-X19.*X32+X20.*X31;-X23.*X36+X24.*X35;X22.*X36-X24.*X34;-X22.*X35+X23.*X34;X15.*t39.*t589+X14.*t39.*t597+X14.*t39.*t43.*t547-X15.*t39.*t43.*t546];
mt2 = [-X15.*t39.*t593-X13.*t39.*t597-X13.*t39.*t43.*t547+X15.*t39.*t43.*t545;-X13.*t39.*t589+X14.*t39.*t593+X13.*t39.*t43.*t546-X14.*t39.*t43.*t545;X18.*t40.*t590+X17.*t40.*t598+X17.*t40.*t44.*t550-X18.*t40.*t44.*t549;-X18.*t40.*t594-X16.*t40.*t598-X16.*t40.*t44.*t550+X18.*t40.*t44.*t548;-X16.*t40.*t590+X17.*t40.*t594+X16.*t40.*t44.*t549-X17.*t40.*t44.*t548;X21.*t41.*t591+X20.*t41.*t599+X20.*t41.*t45.*t553-X21.*t41.*t45.*t552;-X21.*t41.*t595-X19.*t41.*t599-X19.*t41.*t45.*t553+X21.*t41.*t45.*t551;-X19.*t41.*t591+X20.*t41.*t595+X19.*t41.*t45.*t552-X20.*t41.*t45.*t551];
mt3 = [X24.*t42.*t592+X23.*t42.*t600+X23.*t42.*t46.*t556-X24.*t42.*t46.*t555;-X24.*t42.*t596-X22.*t42.*t600-X22.*t42.*t46.*t556+X24.*t42.*t46.*t554;-X22.*t42.*t592+X23.*t42.*t596+X22.*t42.*t46.*t555-X23.*t42.*t46.*t554;t48.*(X49.*t5+X51.*t4.*t15+X50.*t14.*t15);X50.*t4-X51.*t14;t48.*(X51.*t4+X50.*t14);t49.*(X52.*t7+X54.*t6.*t17+X53.*t16.*t17);X53.*t6-X54.*t16;t49.*(X54.*t6+X53.*t16);t50.*(X55.*t9+X57.*t8.*t19+X56.*t18.*t19);X56.*t8-X57.*t18;t50.*(X57.*t8+X56.*t18);t51.*(X58.*t11+X60.*t10.*t21+X59.*t20.*t21);X59.*t10-X60.*t20;t51.*(X60.*t10+X59.*t20)];
mt4 = [(Mi1_1+X50.*X51.*ji1_2-X50.*X51.*ji1_3)./ji1_1;-(Mi1_2+X49.*X51.*ji1_1-X49.*X51.*ji1_3)./ji1_2;-(Mi1_3-X49.*X50.*ji1_1+X49.*X50.*ji1_2)./ji1_3;(Mi2_1+X53.*X54.*ji2_2-X53.*X54.*ji2_3)./ji2_1;-(Mi2_2+X52.*X54.*ji2_1-X52.*X54.*ji2_3)./ji2_2;-(Mi2_3-X52.*X53.*ji2_1+X52.*X53.*ji2_2)./ji2_3;(Mi3_1+X56.*X57.*ji3_2-X56.*X57.*ji3_3)./ji3_1;-(Mi3_2+X55.*X57.*ji3_1-X55.*X57.*ji3_3)./ji3_2;-(Mi3_3-X55.*X56.*ji3_1+X55.*X56.*ji3_2)./ji3_3;(Mi4_1+X59.*X60.*ji4_2-X59.*X60.*ji4_3)./ji4_1];
mt5 = [-(Mi4_2+X58.*X60.*ji4_1-X58.*X60.*ji4_3)./ji4_2;-(Mi4_3-X58.*X59.*ji4_1+X58.*X59.*ji4_2)./ji4_3];
dX = [mt1;mt2;mt3;mt4;mt5];
end
