function dX = zup_tmp_cable_suspended_rigid_body_with_4_drones(in1,in2,in3,in4)
%ZUP_TMP_CABLE_SUSPENDED_RIGID_BODY_WITH_4_DRONES
%    dX = ZUP_TMP_CABLE_SUSPENDED_RIGID_BODY_WITH_4_DRONES(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/06/24 12:16:45

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
X61 = in1(61,:);
X62 = in1(62,:);
X63 = in1(63,:);
X64 = in1(64,:);
X65 = in1(65,:);
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
t2 = X4.^2;
t3 = X5.^2;
t4 = X6.^2;
t5 = X7.^2;
t6 = X11.^2;
t7 = X12.^2;
t8 = X13.^2;
t9 = X14.^2;
t10 = X15.^2;
t11 = X16.^2;
t12 = X17.^2;
t13 = X18.^2;
t14 = X19.^2;
t15 = X20.^2;
t16 = X21.^2;
t17 = X22.^2;
t18 = X23.^2;
t19 = X24.^2;
t20 = X25.^2;
t21 = X38.^2;
t22 = X39.^2;
t23 = X40.^2;
t24 = X41.^2;
t25 = X42.^2;
t26 = X43.^2;
t27 = X44.^2;
t28 = X45.^2;
t29 = X46.^2;
t30 = X47.^2;
t31 = X48.^2;
t32 = X49.^2;
t33 = X50.^2;
t34 = X51.^2;
t35 = X52.^2;
t36 = X53.^2;
t37 = X4.*X5.*2.0;
t38 = X4.*X6.*2.0;
t39 = X4.*X7.*2.0;
t40 = X5.*X6.*2.0;
t41 = X5.*X7.*2.0;
t42 = X6.*X7.*2.0;
t43 = X38.*X39.*2.0;
t44 = X38.*X40.*2.0;
t45 = X39.*X41.*2.0;
t46 = X40.*X41.*2.0;
t47 = X42.*X43.*2.0;
t48 = X42.*X44.*2.0;
t49 = X43.*X45.*2.0;
t50 = X44.*X45.*2.0;
t51 = X46.*X47.*2.0;
t52 = X46.*X48.*2.0;
t53 = X47.*X49.*2.0;
t54 = X48.*X49.*2.0;
t55 = X50.*X51.*2.0;
t56 = X50.*X52.*2.0;
t57 = X51.*X53.*2.0;
t58 = X52.*X53.*2.0;
t59 = -ddX3;
t60 = 1.0./li1;
t61 = 1.0./li2;
t62 = 1.0./li3;
t63 = 1.0./li4;
t64 = 1.0./mi1;
t65 = 1.0./mi2;
t66 = 1.0./mi3;
t67 = 1.0./mi4;
t68 = -t40;
t69 = -t41;
t70 = -t42;
t71 = -t46;
t72 = -t50;
t73 = -t54;
t74 = -t58;
t75 = -t3;
t76 = -t4;
t77 = -t5;
t78 = -t22;
t79 = -t23;
t80 = -t26;
t81 = -t27;
t82 = -t30;
t83 = -t31;
t84 = -t34;
t85 = -t35;
t86 = t9-1.0;
t87 = t10-1.0;
t88 = t11-1.0;
t89 = t12-1.0;
t90 = t13-1.0;
t91 = t14-1.0;
t92 = t15-1.0;
t93 = t16-1.0;
t94 = t17-1.0;
t95 = t18-1.0;
t96 = t19-1.0;
t97 = t20-1.0;
t98 = t6+t7;
t99 = t6+t8;
t100 = t7+t8;
t101 = t37+t42;
t102 = t38+t41;
t103 = t39+t40;
t104 = t44+t45;
t105 = t48+t49;
t106 = t52+t53;
t107 = t56+t57;
t108 = t37+t70;
t109 = t38+t69;
t110 = t39+t68;
t111 = t43+t71;
t112 = t47+t72;
t113 = t51+t73;
t114 = t55+t74;
t115 = rho1_1.*t101;
t116 = rho1_1.*t102;
t117 = rho1_2.*t102;
t118 = rho1_2.*t103;
t119 = rho1_3.*t101;
t120 = rho1_3.*t103;
t121 = rho2_1.*t101;
t122 = rho2_1.*t102;
t123 = rho2_2.*t102;
t124 = rho2_2.*t103;
t125 = rho2_3.*t101;
t126 = rho2_3.*t103;
t127 = rho3_1.*t101;
t128 = rho3_1.*t102;
t129 = rho3_2.*t102;
t130 = rho3_2.*t103;
t131 = rho3_3.*t101;
t132 = rho3_3.*t103;
t133 = rho4_1.*t101;
t134 = rho4_1.*t102;
t135 = rho4_2.*t102;
t136 = rho4_2.*t103;
t137 = rho4_3.*t101;
t138 = rho4_3.*t103;
t163 = X11.*X12.*t101;
t164 = X11.*X12.*t103;
t165 = X11.*X13.*t102;
t166 = X11.*X13.*t103;
t167 = X12.*X13.*t101;
t168 = X12.*X13.*t102;
t179 = X14.*X15.*fi1.*t104;
t180 = X14.*X16.*fi1.*t104;
t181 = X17.*X18.*fi2.*t105;
t182 = X17.*X19.*fi2.*t105;
t183 = X20.*X21.*fi3.*t106;
t184 = X20.*X22.*fi3.*t106;
t185 = X23.*X24.*fi4.*t107;
t186 = X23.*X25.*fi4.*t107;
t210 = fi1.*t86.*t104;
t211 = fi2.*t89.*t105;
t212 = fi3.*t92.*t106;
t213 = fi4.*t95.*t107;
t214 = t98.*t102;
t215 = t99.*t101;
t216 = t100.*t103;
t224 = t2+t5+t75+t76;
t225 = t2+t4+t75+t77;
t226 = t2+t3+t76+t77;
t227 = t21+t24+t78+t79;
t228 = t25+t28+t80+t81;
t229 = t29+t32+t82+t83;
t230 = t33+t36+t84+t85;
t139 = rho1_1.*t108;
t140 = rho1_1.*t110;
t141 = rho1_2.*t108;
t142 = rho1_2.*t109;
t143 = rho1_3.*t109;
t144 = rho1_3.*t110;
t145 = rho2_1.*t108;
t146 = rho2_1.*t110;
t147 = rho2_2.*t108;
t148 = rho2_2.*t109;
t149 = rho2_3.*t109;
t150 = rho2_3.*t110;
t151 = rho3_1.*t108;
t152 = rho3_1.*t110;
t153 = rho3_2.*t108;
t154 = rho3_2.*t109;
t155 = rho3_3.*t109;
t156 = rho3_3.*t110;
t157 = rho4_1.*t108;
t158 = rho4_1.*t110;
t159 = rho4_2.*t108;
t160 = rho4_2.*t109;
t161 = rho4_3.*t109;
t162 = rho4_3.*t110;
t173 = X11.*X12.*t109;
t174 = X11.*X12.*t110;
t175 = X11.*X13.*t108;
t176 = X11.*X13.*t109;
t177 = X12.*X13.*t108;
t178 = X12.*X13.*t110;
t191 = -t164;
t192 = -t165;
t193 = -t167;
t194 = X14.*X15.*fi1.*t111;
t195 = X15.*X16.*fi1.*t111;
t196 = X17.*X18.*fi2.*t112;
t197 = X18.*X19.*fi2.*t112;
t198 = X20.*X21.*fi3.*t113;
t199 = X21.*X22.*fi3.*t113;
t200 = X23.*X24.*fi4.*t114;
t201 = X24.*X25.*fi4.*t114;
t217 = fi1.*t87.*t111;
t218 = fi2.*t90.*t112;
t219 = fi3.*t93.*t113;
t220 = fi4.*t96.*t114;
t221 = t98.*t108;
t222 = t99.*t110;
t223 = t100.*t109;
t235 = rho1_1.*t224;
t236 = rho1_1.*t225;
t237 = rho1_2.*t224;
t238 = rho1_2.*t226;
t239 = rho1_3.*t225;
t240 = rho1_3.*t226;
t241 = rho2_1.*t224;
t242 = rho2_1.*t225;
t243 = rho2_2.*t224;
t244 = rho2_2.*t226;
t245 = rho2_3.*t225;
t246 = rho2_3.*t226;
t247 = rho3_1.*t224;
t248 = rho3_1.*t225;
t249 = rho3_2.*t224;
t250 = rho3_2.*t226;
t251 = rho3_3.*t225;
t252 = rho3_3.*t226;
t253 = rho4_1.*t224;
t254 = rho4_1.*t225;
t255 = rho4_2.*t224;
t256 = rho4_2.*t226;
t257 = rho4_3.*t225;
t258 = rho4_3.*t226;
t259 = X11.*X12.*t225;
t260 = X11.*X12.*t226;
t261 = X11.*X13.*t224;
t262 = X11.*X13.*t226;
t263 = X12.*X13.*t224;
t264 = X12.*X13.*t225;
t265 = X14.*X16.*fi1.*t227;
t266 = X15.*X16.*fi1.*t227;
t267 = X17.*X19.*fi2.*t228;
t268 = X18.*X19.*fi2.*t228;
t269 = X20.*X22.*fi3.*t229;
t270 = X21.*X22.*fi3.*t229;
t271 = X23.*X25.*fi4.*t230;
t272 = X24.*X25.*fi4.*t230;
t304 = t98.*t224;
t305 = t99.*t225;
t306 = t100.*t226;
t307 = fi1.*t88.*t227;
t308 = fi2.*t91.*t228;
t309 = fi3.*t94.*t229;
t310 = fi4.*t97.*t230;
t187 = -t142;
t188 = -t148;
t189 = -t154;
t190 = -t160;
t202 = -t194;
t203 = -t195;
t204 = -t196;
t205 = -t197;
t206 = -t198;
t207 = -t199;
t208 = -t200;
t209 = -t201;
t231 = -t217;
t232 = -t218;
t233 = -t219;
t234 = -t220;
t273 = -t235;
t274 = -t237;
t275 = -t238;
t276 = -t241;
t277 = -t243;
t278 = -t244;
t279 = -t247;
t280 = -t249;
t281 = -t250;
t282 = -t253;
t283 = -t255;
t284 = -t256;
t285 = -t259;
t286 = -t262;
t287 = -t263;
t288 = t117+t144;
t289 = t123+t150;
t290 = t129+t156;
t291 = t135+t162;
t311 = -ddX5.*(t120-t139);
t314 = -ddX5.*(t126-t145);
t317 = -ddX5.*(t132-t151);
t320 = -ddX5.*(t138-t157);
t327 = t118+t236;
t328 = t116+t240;
t329 = t124+t242;
t330 = t122+t246;
t331 = t130+t248;
t332 = t128+t252;
t333 = t136+t254;
t334 = t134+t258;
t335 = t141+t239;
t336 = t147+t245;
t337 = t153+t251;
t338 = t159+t257;
t383 = t166+t221+t264;
t384 = t168+t222+t260;
t385 = t163+t223+t261;
t398 = t176+t193+t304;
t399 = t177+t191+t305;
t400 = t174+t192+t306;
t292 = ddX4.*t288;
t293 = ddX4.*t289;
t294 = ddX4.*t290;
t295 = ddX4.*t291;
t296 = t115+t187;
t298 = t121+t188;
t300 = t127+t189;
t302 = t133+t190;
t339 = ddX5.*t328;
t340 = ddX6.*t327;
t341 = ddX5.*t330;
t342 = ddX6.*t329;
t343 = ddX5.*t332;
t344 = ddX6.*t331;
t345 = ddX5.*t334;
t346 = ddX6.*t333;
t347 = t119+t274;
t348 = t125+t277;
t349 = t131+t280;
t350 = t137+t283;
t351 = ddX4.*t335;
t352 = ddX4.*t336;
t353 = ddX4.*t337;
t354 = ddX4.*t338;
t355 = t140+t275;
t356 = t143+t273;
t357 = t146+t278;
t358 = t149+t276;
t359 = t152+t281;
t360 = t155+t279;
t361 = t158+t284;
t362 = t161+t282;
t386 = rho1_1.*t385;
t387 = rho1_2.*t384;
t388 = rho1_3.*t383;
t389 = rho2_1.*t385;
t390 = rho2_2.*t384;
t391 = rho2_3.*t383;
t392 = rho3_1.*t385;
t393 = rho3_2.*t384;
t394 = rho3_3.*t383;
t395 = rho4_1.*t385;
t396 = rho4_2.*t384;
t397 = rho4_3.*t383;
t401 = t178+t214+t286;
t402 = t173+t215+t287;
t403 = t175+t216+t285;
t404 = t202+t210+t265;
t405 = t179+t231+t266;
t406 = t180+t203+t307;
t407 = t204+t211+t267;
t408 = t181+t232+t268;
t409 = t182+t205+t308;
t410 = t206+t212+t269;
t411 = t183+t233+t270;
t412 = t184+t207+t309;
t413 = t208+t213+t271;
t414 = t185+t234+t272;
t415 = t186+t209+t310;
t416 = rho1_1.*t400;
t418 = rho1_2.*t399;
t421 = rho1_3.*t398;
t423 = rho2_1.*t400;
t425 = rho2_2.*t399;
t428 = rho2_3.*t398;
t430 = rho3_1.*t400;
t432 = rho3_2.*t399;
t435 = rho3_3.*t398;
dX = ft_1({Mi1_1,Mi1_2,Mi1_3,Mi2_1,Mi2_2,Mi2_3,Mi3_1,Mi3_2,Mi3_3,Mi4_1,Mi4_2,Mi4_3,X10,X11,X12,X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,X26,X27,X28,X29,X30,X31,X32,X33,X34,X35,X36,X37,X38,X39,X4,X40,X41,X42,X43,X44,X45,X46,X47,X48,X49,X5,X50,X51,X52,X53,X54,X55,X56,X57,X58,X59,X6,X60,X61,X62,X63,X64,X65,X7,X8,X9,ddX1,ddX2,ddX4,ddX5,ddX6,g,ji1_1,ji1_2,ji1_3,ji2_1,ji2_2,ji2_3,ji3_1,ji3_2,ji3_3,ji4_1,ji4_2,ji4_3,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t292,t293,t294,t295,t296,t298,t300,t302,t311,t314,t317,t320,t339,t340,t341,t342,t343,t344,t345,t346,t347,t348,t349,t350,t351,t352,t353,t354,t355,t356,t357,t358,t359,t360,t361,t362,t386,t387,t388,t389,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t415,t416,t418,t421,t423,t425,t428,t430,t432,t435,t59,t60,t61,t62,t63,t64,t65,t66,t67});
end
function dX = ft_1(ct)
[Mi1_1,Mi1_2,Mi1_3,Mi2_1,Mi2_2,Mi2_3,Mi3_1,Mi3_2,Mi3_3,Mi4_1,Mi4_2,Mi4_3,X10,X11,X12,X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,X26,X27,X28,X29,X30,X31,X32,X33,X34,X35,X36,X37,X38,X39,X4,X40,X41,X42,X43,X44,X45,X46,X47,X48,X49,X5,X50,X51,X52,X53,X54,X55,X56,X57,X58,X59,X6,X60,X61,X62,X63,X64,X65,X7,X8,X9,ddX1,ddX2,ddX4,ddX5,ddX6,g,ji1_1,ji1_2,ji1_3,ji2_1,ji2_2,ji2_3,ji3_1,ji3_2,ji3_3,ji4_1,ji4_2,ji4_3,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t292,t293,t294,t295,t296,t298,t300,t302,t311,t314,t317,t320,t339,t340,t341,t342,t343,t344,t345,t346,t347,t348,t349,t350,t351,t352,t353,t354,t355,t356,t357,t358,t359,t360,t361,t362,t386,t387,t388,t389,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t415,t416,t418,t421,t423,t425,t428,t430,t432,t435,t59,t60,t61,t62,t63,t64,t65,t66,t67] = ct{:};
t437 = rho4_1.*t400;
t439 = rho4_2.*t399;
t442 = rho4_3.*t398;
t312 = ddX6.*t296;
t313 = -t292;
t315 = ddX6.*t298;
t316 = -t293;
t318 = ddX6.*t300;
t319 = -t294;
t321 = ddX6.*t302;
t322 = -t295;
t363 = ddX4.*t347;
t364 = ddX4.*t348;
t365 = ddX4.*t349;
t366 = ddX4.*t350;
t367 = ddX5.*t356;
t368 = ddX6.*t355;
t369 = -t351;
t370 = ddX5.*t358;
t371 = ddX6.*t357;
t372 = -t352;
t373 = ddX5.*t360;
t374 = ddX6.*t359;
t375 = -t353;
t376 = ddX5.*t362;
t377 = ddX6.*t361;
t378 = -t354;
t417 = rho1_1.*t403;
t419 = rho1_2.*t402;
t420 = -t387;
t422 = rho1_3.*t401;
t424 = rho2_1.*t403;
t426 = rho2_2.*t402;
t427 = -t390;
t429 = rho2_3.*t401;
t431 = rho3_1.*t403;
t433 = rho3_2.*t402;
t434 = -t393;
t436 = rho3_3.*t401;
t438 = rho4_1.*t403;
t440 = rho4_2.*t402;
t441 = -t396;
t443 = rho4_3.*t401;
t444 = -t416;
t445 = -t418;
t446 = -t423;
t447 = -t425;
t448 = -t430;
t449 = -t432;
t450 = -t437;
t451 = -t439;
t323 = -t312;
t324 = -t315;
t325 = -t318;
t326 = -t321;
t379 = -t367;
t380 = -t370;
t381 = -t373;
t382 = -t376;
t452 = ddX2+t311+t340+t369+t388+t417+t445;
t453 = ddX2+t314+t342+t372+t391+t424+t447;
t454 = ddX2+t317+t344+t375+t394+t431+t449;
t455 = ddX2+t320+t346+t378+t397+t438+t451;
t456 = ddX1+t313+t339+t368+t420+t422+t444;
t457 = ddX1+t316+t341+t371+t427+t429+t446;
t458 = ddX1+t319+t343+t374+t434+t436+t448;
t459 = ddX1+t322+t345+t377+t441+t443+t450;
t460 = g+t59+t323+t363+t379+t386+t419+t421;
t461 = g+t59+t324+t364+t380+t389+t426+t428;
t462 = g+t59+t325+t365+t381+t392+t433+t435;
t463 = g+t59+t326+t366+t382+t395+t440+t442;
mt1 = [X8;X9;X10;X5.*X11.*(-1.0./2.0)-(X6.*X12)./2.0-(X7.*X13)./2.0;(X4.*X11)./2.0+(X6.*X13)./2.0-(X7.*X12)./2.0;(X4.*X12)./2.0-(X5.*X13)./2.0+(X7.*X11)./2.0;(X4.*X13)./2.0+(X5.*X12)./2.0-(X6.*X11)./2.0;ddX1;-ddX2;t59;ddX4;-ddX5;-ddX6;-X15.*X28+X16.*X27;X14.*X28-X16.*X26;-X14.*X27+X15.*X26;-X18.*X31+X19.*X30;X17.*X31-X19.*X29;-X17.*X30+X18.*X29;-X21.*X34+X22.*X33;X20.*X34-X22.*X32;-X20.*X33+X21.*X32;-X24.*X37+X25.*X36;X23.*X37-X25.*X35];
mt2 = [-X23.*X36+X24.*X35;X16.*t60.*t452+X15.*t60.*t460+X15.*t60.*t64.*t406-X16.*t60.*t64.*t405;X16.*t60.*t456-X14.*t60.*t460-X14.*t60.*t64.*t406+X16.*t60.*t64.*t404;-X14.*t60.*t452-X15.*t60.*t456+X14.*t60.*t64.*t405-X15.*t60.*t64.*t404;X19.*t61.*t453+X18.*t61.*t461+X18.*t61.*t65.*t409-X19.*t61.*t65.*t408;X19.*t61.*t457-X17.*t61.*t461-X17.*t61.*t65.*t409+X19.*t61.*t65.*t407;-X17.*t61.*t453-X18.*t61.*t457+X17.*t61.*t65.*t408-X18.*t61.*t65.*t407;X22.*t62.*t454+X21.*t62.*t462+X21.*t62.*t66.*t412-X22.*t62.*t66.*t411;X22.*t62.*t458-X20.*t62.*t462-X20.*t62.*t66.*t412+X22.*t62.*t66.*t410];
mt3 = [-X20.*t62.*t454-X21.*t62.*t458+X20.*t62.*t66.*t411-X21.*t62.*t66.*t410;X25.*t63.*t455+X24.*t63.*t463+X24.*t63.*t67.*t415-X25.*t63.*t67.*t414;X25.*t63.*t459-X23.*t63.*t463-X23.*t63.*t67.*t415+X25.*t63.*t67.*t413;-X23.*t63.*t455-X24.*t63.*t459+X23.*t63.*t67.*t414-X24.*t63.*t67.*t413;X39.*X54.*(-1.0./2.0)-(X40.*X55)./2.0-(X41.*X56)./2.0;(X38.*X54)./2.0+(X40.*X56)./2.0-(X41.*X55)./2.0;(X38.*X55)./2.0-(X39.*X56)./2.0+(X41.*X54)./2.0;(X38.*X56)./2.0+(X39.*X55)./2.0-(X40.*X54)./2.0;X43.*X57.*(-1.0./2.0)-(X44.*X58)./2.0-(X45.*X59)./2.0];
mt4 = [(X42.*X57)./2.0+(X44.*X59)./2.0-(X45.*X58)./2.0;(X42.*X58)./2.0-(X43.*X59)./2.0+(X45.*X57)./2.0;(X42.*X59)./2.0+(X43.*X58)./2.0-(X44.*X57)./2.0;X47.*X60.*(-1.0./2.0)-(X48.*X61)./2.0-(X49.*X62)./2.0;(X46.*X60)./2.0+(X48.*X62)./2.0-(X49.*X61)./2.0;(X46.*X61)./2.0-(X47.*X62)./2.0+(X49.*X60)./2.0;(X46.*X62)./2.0+(X47.*X61)./2.0-(X48.*X60)./2.0;X51.*X63.*(-1.0./2.0)-(X52.*X64)./2.0-(X53.*X65)./2.0;(X50.*X63)./2.0+(X52.*X65)./2.0-(X53.*X64)./2.0];
mt5 = [(X50.*X64)./2.0-(X51.*X65)./2.0+(X53.*X63)./2.0;(X50.*X65)./2.0+(X51.*X64)./2.0-(X52.*X63)./2.0;(Mi1_1+X55.*X56.*ji1_2-X55.*X56.*ji1_3)./ji1_1;-(Mi1_2+X54.*X56.*ji1_1-X54.*X56.*ji1_3)./ji1_2;-(Mi1_3-X54.*X55.*ji1_1+X54.*X55.*ji1_2)./ji1_3;(Mi2_1+X58.*X59.*ji2_2-X58.*X59.*ji2_3)./ji2_1;-(Mi2_2+X57.*X59.*ji2_1-X57.*X59.*ji2_3)./ji2_2;-(Mi2_3-X57.*X58.*ji2_1+X57.*X58.*ji2_2)./ji2_3;(Mi3_1+X61.*X62.*ji3_2-X61.*X62.*ji3_3)./ji3_1;-(Mi3_2+X60.*X62.*ji3_1-X60.*X62.*ji3_3)./ji3_2];
mt6 = [-(Mi3_3-X60.*X61.*ji3_1+X60.*X61.*ji3_2)./ji3_3;(Mi4_1+X64.*X65.*ji4_2-X64.*X65.*ji4_3)./ji4_1;-(Mi4_2+X63.*X65.*ji4_1-X63.*X65.*ji4_3)./ji4_2;-(Mi4_3-X63.*X64.*ji4_1+X63.*X64.*ji4_2)./ji4_3];
dX = [mt1;mt2;mt3;mt4;mt5;mt6];
end
