function A = Addx0do0_5(in1,in2,in3,in4)
%Addx0do0_5
%    A = Addx0do0_5(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/08/15 14:21:45

R01_1 = in2(1);
R01_2 = in2(4);
R01_3 = in2(7);
R02_1 = in2(2);
R02_2 = in2(5);
R02_3 = in2(8);
R03_1 = in2(3);
R03_2 = in2(6);
R03_3 = in2(9);
j01 = in4(:,3);
j02 = in4(:,4);
j03 = in4(:,5);
m0 = in4(:,2);
mi1 = in4(:,26);
mi2 = in4(:,27);
mi3 = in4(:,28);
mi4 = in4(:,29);
mi5 = in4(:,30);
qi1_1 = in1(14,:);
qi1_2 = in1(17,:);
qi1_3 = in1(20,:);
qi1_4 = in1(23,:);
qi1_5 = in1(26,:);
qi2_1 = in1(15,:);
qi2_2 = in1(18,:);
qi2_3 = in1(21,:);
qi2_4 = in1(24,:);
qi2_5 = in1(27,:);
qi3_1 = in1(16,:);
qi3_2 = in1(19,:);
qi3_3 = in1(22,:);
qi3_4 = in1(25,:);
qi3_5 = in1(28,:);
rho1_1 = in4(:,6);
rho1_2 = in4(:,9);
rho1_3 = in4(:,12);
rho1_4 = in4(:,15);
rho1_5 = in4(:,18);
rho2_1 = in4(:,7);
rho2_2 = in4(:,10);
rho2_3 = in4(:,13);
rho2_4 = in4(:,16);
rho2_5 = in4(:,19);
rho3_1 = in4(:,8);
rho3_2 = in4(:,11);
rho3_3 = in4(:,14);
rho3_4 = in4(:,17);
rho3_5 = in4(:,20);
t2 = qi1_1.^2;
t3 = qi1_2.^2;
t4 = qi1_3.^2;
t5 = qi1_4.^2;
t6 = qi1_5.^2;
t7 = qi2_1.^2;
t8 = qi2_2.^2;
t9 = qi2_3.^2;
t10 = qi2_4.^2;
t11 = qi2_5.^2;
t12 = qi3_1.^2;
t13 = qi3_2.^2;
t14 = qi3_3.^2;
t15 = qi3_4.^2;
t16 = qi3_5.^2;
t17 = R01_2.*mi1.*rho1_1;
t18 = R01_3.*mi1.*rho1_1;
t19 = R01_2.*mi2.*rho1_2;
t20 = R01_3.*mi2.*rho1_2;
t21 = R01_2.*mi3.*rho1_3;
t22 = R01_3.*mi3.*rho1_3;
t23 = R01_2.*mi4.*rho1_4;
t24 = R01_3.*mi4.*rho1_4;
t25 = R01_2.*mi5.*rho1_5;
t26 = R01_1.*mi1.*rho2_1;
t27 = R01_3.*mi5.*rho1_5;
t28 = R02_2.*mi1.*rho1_1;
t29 = R01_1.*mi2.*rho2_2;
t30 = R01_3.*mi1.*rho2_1;
t31 = R02_3.*mi1.*rho1_1;
t32 = R02_2.*mi2.*rho1_2;
t33 = R01_1.*mi3.*rho2_3;
t34 = R01_3.*mi2.*rho2_2;
t35 = R02_3.*mi2.*rho1_2;
t36 = R02_2.*mi3.*rho1_3;
t37 = R01_1.*mi4.*rho2_4;
t38 = R01_3.*mi3.*rho2_3;
t39 = R02_3.*mi3.*rho1_3;
t40 = R02_2.*mi4.*rho1_4;
t41 = R01_1.*mi5.*rho2_5;
t42 = R01_3.*mi4.*rho2_4;
t43 = R02_3.*mi4.*rho1_4;
t44 = R02_2.*mi5.*rho1_5;
t45 = R01_1.*mi1.*rho3_1;
t46 = R01_3.*mi5.*rho2_5;
t47 = R02_1.*mi1.*rho2_1;
t48 = R02_3.*mi5.*rho1_5;
t49 = R01_2.*mi1.*rho3_1;
t50 = R03_2.*mi1.*rho1_1;
t51 = R01_1.*mi2.*rho3_2;
t52 = R02_1.*mi2.*rho2_2;
t53 = R02_3.*mi1.*rho2_1;
t54 = R03_3.*mi1.*rho1_1;
t55 = R01_2.*mi2.*rho3_2;
t56 = R03_2.*mi2.*rho1_2;
t57 = R01_1.*mi3.*rho3_3;
t58 = R02_1.*mi3.*rho2_3;
t59 = R02_3.*mi2.*rho2_2;
t60 = R03_3.*mi2.*rho1_2;
t61 = R01_2.*mi3.*rho3_3;
t62 = R03_2.*mi3.*rho1_3;
t63 = R01_1.*mi4.*rho3_4;
t64 = R02_1.*mi4.*rho2_4;
t65 = R02_3.*mi3.*rho2_3;
t66 = R03_3.*mi3.*rho1_3;
t67 = R01_2.*mi4.*rho3_4;
t68 = R03_2.*mi4.*rho1_4;
t69 = R01_1.*mi5.*rho3_5;
t70 = R02_1.*mi5.*rho2_5;
t71 = R02_3.*mi4.*rho2_4;
t72 = R03_3.*mi4.*rho1_4;
t73 = R01_2.*mi5.*rho3_5;
t74 = R03_2.*mi5.*rho1_5;
t75 = R02_1.*mi1.*rho3_1;
t76 = R02_3.*mi5.*rho2_5;
t77 = R03_1.*mi1.*rho2_1;
t78 = R03_3.*mi5.*rho1_5;
t79 = R02_2.*mi1.*rho3_1;
t80 = R02_1.*mi2.*rho3_2;
t81 = R03_1.*mi2.*rho2_2;
t82 = R03_3.*mi1.*rho2_1;
t83 = R02_2.*mi2.*rho3_2;
t84 = R02_1.*mi3.*rho3_3;
t85 = R03_1.*mi3.*rho2_3;
t86 = R03_3.*mi2.*rho2_2;
t87 = R02_2.*mi3.*rho3_3;
t88 = R02_1.*mi4.*rho3_4;
t89 = R03_1.*mi4.*rho2_4;
t90 = R03_3.*mi3.*rho2_3;
t91 = R02_2.*mi4.*rho3_4;
t92 = R02_1.*mi5.*rho3_5;
t93 = R03_1.*mi5.*rho2_5;
t94 = R03_3.*mi4.*rho2_4;
t95 = R02_2.*mi5.*rho3_5;
t96 = R03_1.*mi1.*rho3_1;
t97 = R03_3.*mi5.*rho2_5;
t98 = R03_2.*mi1.*rho3_1;
t99 = R03_1.*mi2.*rho3_2;
t100 = R03_2.*mi2.*rho3_2;
t101 = R03_1.*mi3.*rho3_3;
t102 = R03_2.*mi3.*rho3_3;
t103 = R03_1.*mi4.*rho3_4;
t104 = R03_2.*mi4.*rho3_4;
t105 = R03_1.*mi5.*rho3_5;
t106 = R03_2.*mi5.*rho3_5;
t107 = mi1.*qi1_1.*qi2_1;
t108 = mi2.*qi1_2.*qi2_2;
t109 = mi3.*qi1_3.*qi2_3;
t110 = mi4.*qi1_4.*qi2_4;
t111 = mi1.*qi1_1.*qi3_1;
t112 = mi5.*qi1_5.*qi2_5;
t113 = mi2.*qi1_2.*qi3_2;
t114 = mi3.*qi1_3.*qi3_3;
t115 = mi4.*qi1_4.*qi3_4;
t116 = mi1.*qi2_1.*qi3_1;
t117 = mi5.*qi1_5.*qi3_5;
t118 = mi2.*qi2_2.*qi3_2;
t119 = mi3.*qi2_3.*qi3_3;
t120 = mi4.*qi2_4.*qi3_4;
t121 = mi5.*qi2_5.*qi3_5;
t122 = R01_1.*t107;
t123 = R01_2.*t107;
t124 = R01_3.*t107;
t125 = R01_1.*t108;
t126 = R01_2.*t108;
t127 = R01_3.*t108;
t128 = R01_1.*t109;
t129 = R01_2.*t109;
t130 = R01_3.*t109;
t131 = R01_1.*t110;
t132 = R01_1.*t111;
t133 = R01_2.*t110;
t134 = R02_1.*t107;
t135 = R01_2.*t111;
t136 = R01_3.*t110;
t137 = R02_2.*t107;
t138 = R01_1.*t112;
t139 = R01_3.*t111;
t140 = R02_3.*t107;
t141 = R01_1.*t113;
t142 = R01_2.*t112;
t143 = R02_1.*t108;
t144 = R01_2.*t113;
t145 = R01_3.*t112;
t146 = R02_2.*t108;
t147 = R01_3.*t113;
t148 = R02_3.*t108;
t149 = R01_1.*t114;
t150 = R02_1.*t109;
t151 = R01_2.*t114;
t152 = R02_2.*t109;
t153 = R01_3.*t114;
t154 = R02_3.*t109;
t155 = R01_1.*t115;
t156 = R02_1.*t110;
t157 = R01_2.*t115;
t158 = R02_2.*t110;
t159 = R01_3.*t115;
t160 = R02_3.*t110;
t161 = R01_1.*t117;
t162 = R02_1.*t112;
t163 = R01_2.*t117;
t164 = R02_2.*t112;
t165 = R01_3.*t117;
t166 = R02_3.*t112;
t167 = R02_1.*t116;
t168 = R03_1.*t111;
t169 = R02_2.*t116;
t170 = R03_2.*t111;
t171 = R02_3.*t116;
t172 = R03_3.*t111;
t173 = R02_1.*t118;
t174 = R03_1.*t113;
t175 = R02_2.*t118;
t176 = R03_2.*t113;
t177 = R02_3.*t118;
t178 = R03_3.*t113;
t179 = R02_1.*t119;
t180 = R03_1.*t114;
t181 = R02_2.*t119;
t182 = R03_2.*t114;
t183 = R02_3.*t119;
t184 = R03_3.*t114;
t185 = R02_1.*t120;
t186 = R03_1.*t115;
t187 = R02_2.*t120;
t188 = R03_1.*t116;
t189 = R03_2.*t115;
t190 = R02_3.*t120;
t191 = R03_2.*t116;
t192 = R03_3.*t115;
t193 = R02_1.*t121;
t194 = R03_1.*t117;
t195 = R03_3.*t116;
t196 = R02_2.*t121;
t197 = R03_1.*t118;
t198 = R03_2.*t117;
t199 = R02_3.*t121;
t200 = R03_2.*t118;
t201 = R03_3.*t117;
t202 = R03_3.*t118;
t203 = R03_1.*t119;
t204 = R03_2.*t119;
t205 = R03_3.*t119;
t206 = R03_1.*t120;
t207 = R03_2.*t120;
t208 = R03_3.*t120;
t209 = R03_1.*t121;
t210 = R03_2.*t121;
t211 = R03_3.*t121;
t212 = -t26;
t213 = -t29;
t214 = -t33;
t215 = -t37;
t216 = -t41;
t217 = -t45;
t218 = -t47;
t219 = -t49;
t220 = -t51;
t221 = -t52;
t222 = -t55;
t223 = -t57;
t224 = -t58;
t225 = -t61;
t226 = -t63;
t227 = -t64;
t228 = -t67;
t229 = -t69;
t230 = -t70;
t231 = -t73;
t232 = -t75;
t233 = -t77;
t234 = -t79;
t235 = -t80;
t236 = -t81;
t237 = -t83;
t238 = -t84;
t239 = -t85;
t240 = -t87;
t241 = -t88;
t242 = -t89;
t243 = -t91;
t244 = -t92;
t245 = -t93;
t246 = -t95;
t247 = -t96;
t248 = -t98;
t249 = -t99;
t250 = -t100;
t251 = -t101;
t252 = -t102;
t253 = -t103;
t254 = -t104;
t255 = -t105;
t256 = -t106;
t257 = R01_1.*mi1.*t2;
t258 = R01_2.*mi1.*t2;
t259 = R01_1.*mi2.*t3;
t260 = R01_3.*mi1.*t2;
t261 = R01_2.*mi2.*t3;
t262 = R01_1.*mi3.*t4;
t263 = R01_3.*mi2.*t3;
t264 = R01_2.*mi3.*t4;
t265 = R01_1.*mi4.*t5;
t266 = R01_3.*mi3.*t4;
t267 = R01_2.*mi4.*t5;
t268 = R01_1.*mi5.*t6;
t269 = R01_3.*mi4.*t5;
t270 = R01_2.*mi5.*t6;
t271 = R01_3.*mi5.*t6;
t272 = R02_1.*mi1.*t7;
t273 = R02_2.*mi1.*t7;
t274 = R02_1.*mi2.*t8;
t275 = R02_3.*mi1.*t7;
t276 = R02_2.*mi2.*t8;
t277 = R02_1.*mi3.*t9;
t278 = R02_3.*mi2.*t8;
t279 = R02_2.*mi3.*t9;
t280 = R02_1.*mi4.*t10;
t281 = R02_3.*mi3.*t9;
t282 = R02_2.*mi4.*t10;
t283 = R02_1.*mi5.*t11;
t284 = R02_3.*mi4.*t10;
t285 = R02_2.*mi5.*t11;
t286 = R02_3.*mi5.*t11;
t287 = R03_1.*mi1.*t12;
t288 = R03_2.*mi1.*t12;
t289 = R03_1.*mi2.*t13;
t290 = R03_3.*mi1.*t12;
t291 = R03_2.*mi2.*t13;
t292 = R03_1.*mi3.*t14;
t293 = R03_3.*mi2.*t13;
t294 = R03_2.*mi3.*t14;
t295 = R03_1.*mi4.*t15;
t296 = R03_3.*mi3.*t14;
t297 = R03_2.*mi4.*t15;
t298 = R03_1.*mi5.*t16;
t299 = R03_3.*mi4.*t15;
t300 = R03_2.*mi5.*t16;
t301 = R03_3.*mi5.*t16;
t437 = t107+t108+t109+t110+t112;
t438 = t111+t113+t114+t115+t117;
t439 = t116+t118+t119+t120+t121;
t302 = t17+t212;
t303 = t19+t213;
t304 = t21+t214;
t305 = t18+t217;
t306 = t23+t215;
t307 = t20+t220;
t308 = t25+t216;
t309 = t22+t223;
t310 = t28+t218;
t311 = t30+t219;
t312 = t24+t226;
t313 = t32+t221;
t314 = t34+t222;
t315 = t27+t229;
t316 = t36+t224;
t317 = t38+t225;
t318 = t31+t232;
t319 = t40+t227;
t320 = t42+t228;
t321 = t35+t235;
t322 = t44+t230;
t323 = t46+t231;
t324 = t39+t238;
t325 = t50+t233;
t326 = t53+t234;
t327 = t43+t241;
t328 = t56+t236;
t329 = t59+t237;
t330 = t48+t244;
t331 = t62+t239;
t332 = t65+t240;
t333 = t54+t247;
t334 = t68+t242;
t335 = t71+t243;
t336 = t60+t249;
t337 = t74+t245;
t338 = t76+t246;
t339 = t66+t251;
t340 = t82+t248;
t341 = t72+t253;
t342 = t86+t250;
t343 = t78+t255;
t344 = t90+t252;
t345 = t94+t254;
t346 = t97+t256;
t392 = t134+t168+t257;
t393 = t137+t170+t258;
t394 = t140+t172+t260;
t395 = t143+t174+t259;
t396 = t146+t176+t261;
t397 = t148+t178+t263;
t398 = t150+t180+t262;
t399 = t152+t182+t264;
t400 = t122+t188+t272;
t401 = t154+t184+t266;
t402 = t123+t191+t273;
t403 = t156+t186+t265;
t404 = t124+t195+t275;
t405 = t158+t189+t267;
t406 = t125+t197+t274;
t407 = t160+t192+t269;
t408 = t126+t200+t276;
t409 = t162+t194+t268;
t410 = t127+t202+t278;
t411 = t164+t198+t270;
t412 = t128+t203+t277;
t413 = t166+t201+t271;
t414 = t129+t204+t279;
t415 = t132+t167+t287;
t416 = t130+t205+t281;
t417 = t135+t169+t288;
t418 = t131+t206+t280;
t419 = t139+t171+t290;
t420 = t133+t207+t282;
t421 = t141+t173+t289;
t422 = t136+t208+t284;
t423 = t144+t175+t291;
t424 = t138+t209+t283;
t425 = t147+t177+t293;
t426 = t142+t210+t285;
t427 = t149+t179+t292;
t428 = t145+t211+t286;
t429 = t151+t181+t294;
t430 = t153+t183+t296;
t431 = t155+t185+t295;
t432 = t157+t187+t297;
t433 = t159+t190+t299;
t434 = t161+t193+t298;
t435 = t163+t196+t300;
t436 = t165+t199+t301;
t347 = qi1_1.*t302;
t348 = qi1_2.*t303;
t349 = qi1_3.*t304;
t350 = qi1_1.*t305;
t351 = qi1_4.*t306;
t352 = qi1_2.*t307;
t353 = qi1_5.*t308;
t354 = qi1_3.*t309;
t355 = qi1_1.*t311;
t356 = qi1_4.*t312;
t357 = qi1_2.*t314;
t358 = qi2_1.*t310;
t359 = qi1_5.*t315;
t360 = qi1_3.*t317;
t361 = qi2_2.*t313;
t362 = qi1_4.*t320;
t363 = qi2_3.*t316;
t364 = qi2_1.*t318;
t365 = qi1_5.*t323;
t366 = qi2_4.*t319;
t367 = qi2_2.*t321;
t368 = qi2_5.*t322;
t369 = qi2_3.*t324;
t370 = qi2_1.*t326;
t371 = qi2_4.*t327;
t372 = qi2_2.*t329;
t373 = qi3_1.*t325;
t374 = qi2_5.*t330;
t375 = qi2_3.*t332;
t376 = qi3_2.*t328;
t377 = qi2_4.*t335;
t378 = qi3_3.*t331;
t379 = qi3_1.*t333;
t380 = qi2_5.*t338;
t381 = qi3_4.*t334;
t382 = qi3_2.*t336;
t383 = qi3_5.*t337;
t384 = qi3_3.*t339;
t385 = qi3_1.*t340;
t386 = qi3_4.*t341;
t387 = qi3_2.*t342;
t388 = qi3_5.*t343;
t389 = qi3_3.*t344;
t390 = qi3_4.*t345;
t391 = qi3_5.*t346;
t440 = t347+t358+t373;
t441 = t348+t361+t376;
t442 = t349+t363+t378;
t443 = t350+t364+t379;
t444 = t351+t366+t381;
t445 = t352+t367+t382;
t446 = t353+t368+t383;
t447 = t354+t369+t384;
t448 = t355+t370+t385;
t449 = t356+t371+t386;
t450 = t357+t372+t387;
t451 = t359+t374+t388;
t452 = t360+t375+t389;
t453 = t362+t377+t390;
A = ft_1({R01_1,R01_2,R01_3,R02_1,R02_2,R02_3,R03_1,R03_2,R03_3,j01,j02,j03,m0,mi1,mi2,mi3,mi4,mi5,qi1_1,qi1_2,qi1_3,qi1_4,qi1_5,qi2_1,qi2_2,qi2_3,qi2_4,qi2_5,qi3_1,qi3_2,qi3_3,qi3_4,qi3_5,rho1_1,rho1_2,rho1_3,rho1_4,rho1_5,rho2_1,rho2_2,rho2_3,rho2_4,rho2_5,rho3_1,rho3_2,rho3_3,rho3_4,rho3_5,t10,t11,t12,t13,t14,t15,t16,t2,t3,t365,t380,t391,t392,t393,t394,t395,t396,t397,t398,t399,t4,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t415,t416,t417,t418,t419,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t430,t431,t432,t433,t434,t435,t436,t437,t438,t439,t440,t441,t442,t443,t444,t445,t446,t447,t448,t449,t450,t451,t452,t453,t5,t6,t7,t8,t9});
end
function A = ft_1(ct)
[R01_1,R01_2,R01_3,R02_1,R02_2,R02_3,R03_1,R03_2,R03_3,j01,j02,j03,m0,mi1,mi2,mi3,mi4,mi5,qi1_1,qi1_2,qi1_3,qi1_4,qi1_5,qi2_1,qi2_2,qi2_3,qi2_4,qi2_5,qi3_1,qi3_2,qi3_3,qi3_4,qi3_5,rho1_1,rho1_2,rho1_3,rho1_4,rho1_5,rho2_1,rho2_2,rho2_3,rho2_4,rho2_5,rho3_1,rho3_2,rho3_3,rho3_4,rho3_5,t10,t11,t12,t13,t14,t15,t16,t2,t3,t365,t380,t391,t392,t393,t394,t395,t396,t397,t398,t399,t4,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t415,t416,t417,t418,t419,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t430,t431,t432,t433,t434,t435,t436,t437,t438,t439,t440,t441,t442,t443,t444,t445,t446,t447,t448,t449,t450,t451,t452,t453,t5,t6,t7,t8,t9] = ct{:};
t454 = t365+t380+t391;
t455 = R01_1.*qi1_1.*t440;
t456 = R01_2.*qi1_1.*t440;
t457 = R01_3.*qi1_1.*t440;
t458 = R01_1.*qi1_2.*t441;
t459 = R01_2.*qi1_2.*t441;
t460 = R01_3.*qi1_2.*t441;
t461 = R02_1.*qi2_1.*t440;
t462 = R02_2.*qi2_1.*t440;
t463 = R02_3.*qi2_1.*t440;
t464 = R01_1.*qi1_3.*t442;
t465 = R01_1.*qi1_1.*t443;
t466 = R01_2.*qi1_3.*t442;
t467 = R01_2.*qi1_1.*t443;
t468 = R01_3.*qi1_3.*t442;
t469 = R01_3.*qi1_1.*t443;
t470 = R02_1.*qi2_2.*t441;
t471 = R02_2.*qi2_2.*t441;
t472 = R02_3.*qi2_2.*t441;
t473 = R03_1.*qi3_1.*t440;
t474 = R03_2.*qi3_1.*t440;
t475 = R03_3.*qi3_1.*t440;
t476 = R01_1.*qi1_4.*t444;
t477 = R01_1.*qi1_2.*t445;
t478 = R01_2.*qi1_4.*t444;
t479 = R01_2.*qi1_2.*t445;
t480 = R01_3.*qi1_4.*t444;
t481 = R01_3.*qi1_2.*t445;
t482 = R02_1.*qi2_3.*t442;
t483 = R02_1.*qi2_1.*t443;
t484 = R02_2.*qi2_3.*t442;
t485 = R02_2.*qi2_1.*t443;
t486 = R02_3.*qi2_3.*t442;
t487 = R02_3.*qi2_1.*t443;
t488 = R03_1.*qi3_2.*t441;
t489 = R03_2.*qi3_2.*t441;
t490 = R03_3.*qi3_2.*t441;
t491 = R01_1.*qi1_5.*t446;
t492 = R01_1.*qi1_3.*t447;
t493 = R01_2.*qi1_5.*t446;
t494 = R01_1.*qi1_1.*t448;
t495 = R01_2.*qi1_3.*t447;
t496 = R01_3.*qi1_5.*t446;
t497 = R01_2.*qi1_1.*t448;
t498 = R01_3.*qi1_3.*t447;
t499 = R01_3.*qi1_1.*t448;
t500 = R02_1.*qi2_4.*t444;
t501 = R02_1.*qi2_2.*t445;
t502 = R02_2.*qi2_4.*t444;
t503 = R02_2.*qi2_2.*t445;
t504 = R02_3.*qi2_4.*t444;
t505 = R02_3.*qi2_2.*t445;
t506 = R03_1.*qi3_3.*t442;
t507 = R03_1.*qi3_1.*t443;
t508 = R03_2.*qi3_3.*t442;
t509 = R03_2.*qi3_1.*t443;
t510 = R03_3.*qi3_3.*t442;
t511 = R03_3.*qi3_1.*t443;
t512 = R01_1.*qi1_4.*t449;
t513 = R01_1.*qi1_2.*t450;
t514 = R01_2.*qi1_4.*t449;
t515 = R01_2.*qi1_2.*t450;
t516 = R01_3.*qi1_4.*t449;
t517 = R01_3.*qi1_2.*t450;
t518 = R02_1.*qi2_5.*t446;
t519 = R02_1.*qi2_3.*t447;
t520 = R02_2.*qi2_5.*t446;
t521 = R02_1.*qi2_1.*t448;
t522 = R02_2.*qi2_3.*t447;
t523 = R02_3.*qi2_5.*t446;
t524 = R02_2.*qi2_1.*t448;
t525 = R02_3.*qi2_3.*t447;
t526 = R02_3.*qi2_1.*t448;
t527 = R03_1.*qi3_4.*t444;
t528 = R03_1.*qi3_2.*t445;
t529 = R03_2.*qi3_4.*t444;
t530 = R03_2.*qi3_2.*t445;
t531 = R03_3.*qi3_4.*t444;
t532 = R03_3.*qi3_2.*t445;
t533 = R01_1.*qi1_5.*t451;
t534 = R01_1.*qi1_3.*t452;
t535 = R01_2.*qi1_5.*t451;
t536 = R01_2.*qi1_3.*t452;
t537 = R01_3.*qi1_5.*t451;
t538 = R01_3.*qi1_3.*t452;
t539 = R02_1.*qi2_4.*t449;
t540 = R02_1.*qi2_2.*t450;
t541 = R02_2.*qi2_4.*t449;
t542 = R02_2.*qi2_2.*t450;
t543 = R02_3.*qi2_4.*t449;
t544 = R02_3.*qi2_2.*t450;
t545 = R03_1.*qi3_5.*t446;
t546 = R03_1.*qi3_3.*t447;
t547 = R03_2.*qi3_5.*t446;
t548 = R03_1.*qi3_1.*t448;
t549 = R03_2.*qi3_3.*t447;
t550 = R03_3.*qi3_5.*t446;
t551 = R03_2.*qi3_1.*t448;
t552 = R03_3.*qi3_3.*t447;
t553 = R03_3.*qi3_1.*t448;
t554 = R01_1.*qi1_4.*t453;
t555 = R01_2.*qi1_4.*t453;
t556 = R01_3.*qi1_4.*t453;
t557 = R02_1.*qi2_5.*t451;
t558 = R02_1.*qi2_3.*t452;
t559 = R02_2.*qi2_5.*t451;
t560 = R02_2.*qi2_3.*t452;
t561 = R02_3.*qi2_5.*t451;
t562 = R02_3.*qi2_3.*t452;
t563 = R03_1.*qi3_4.*t449;
t564 = R03_1.*qi3_2.*t450;
t565 = R03_2.*qi3_4.*t449;
t566 = R03_2.*qi3_2.*t450;
t567 = R03_3.*qi3_4.*t449;
t568 = R03_3.*qi3_2.*t450;
t569 = R01_1.*qi1_5.*t454;
t570 = R01_2.*qi1_5.*t454;
t571 = R01_3.*qi1_5.*t454;
t572 = R02_1.*qi2_4.*t453;
t573 = R02_2.*qi2_4.*t453;
t574 = R02_3.*qi2_4.*t453;
t575 = R03_1.*qi3_5.*t451;
t576 = R03_1.*qi3_3.*t452;
t577 = R03_2.*qi3_5.*t451;
t578 = R03_2.*qi3_3.*t452;
t579 = R03_3.*qi3_5.*t451;
t580 = R03_3.*qi3_3.*t452;
t581 = R02_1.*qi2_5.*t454;
t582 = R02_2.*qi2_5.*t454;
t583 = R02_3.*qi2_5.*t454;
t584 = R03_1.*qi3_4.*t453;
t585 = R03_2.*qi3_4.*t453;
t586 = R03_3.*qi3_4.*t453;
t587 = R03_1.*qi3_5.*t454;
t588 = R03_2.*qi3_5.*t454;
t589 = R03_3.*qi3_5.*t454;
t590 = t455+t461+t473;
t591 = t456+t462+t474;
t592 = t457+t463+t475;
t593 = t458+t470+t488;
t594 = t459+t471+t489;
t595 = t460+t472+t490;
t596 = t464+t482+t506;
t597 = t465+t483+t507;
t598 = t466+t484+t508;
t599 = t467+t485+t509;
t600 = t468+t486+t510;
t601 = t469+t487+t511;
t602 = t476+t500+t527;
t603 = t477+t501+t528;
t604 = t478+t502+t529;
t605 = t479+t503+t530;
t606 = t480+t504+t531;
t607 = t481+t505+t532;
t608 = t491+t518+t545;
t609 = t492+t519+t546;
t610 = t493+t520+t547;
t611 = t494+t521+t548;
t612 = t495+t522+t549;
t613 = t496+t523+t550;
t614 = t497+t524+t551;
t615 = t498+t525+t552;
t616 = t499+t526+t553;
t617 = t512+t539+t563;
t618 = t513+t540+t564;
t619 = t514+t541+t565;
t620 = t515+t542+t566;
t621 = t516+t543+t567;
t622 = t517+t544+t568;
t623 = t533+t557+t575;
t624 = t534+t558+t576;
t625 = t535+t559+t577;
t626 = t536+t560+t578;
t627 = t537+t561+t579;
t628 = t538+t562+t580;
t629 = t554+t572+t584;
t630 = t555+t573+t585;
t631 = t556+t574+t586;
t632 = t569+t581+t587;
t633 = t570+t582+t588;
t634 = t571+t583+t589;
mt1 = [m0+mi1.*t2+mi2.*t3+mi3.*t4+mi4.*t5+mi5.*t6,t437,t438,qi1_1.*t448+qi1_2.*t450+qi1_3.*t452+qi1_4.*t453+qi1_5.*t454,-qi1_1.*t443-qi1_2.*t445-qi1_3.*t447-qi1_4.*t449-qi1_5.*t451,qi1_1.*t440+qi1_2.*t441+qi1_3.*t442+qi1_4.*t444+qi1_5.*t446,t437,m0+mi1.*t7+mi2.*t8+mi3.*t9+mi4.*t10+mi5.*t11,t439,qi2_1.*t448+qi2_2.*t450+qi2_3.*t452+qi2_4.*t453+qi2_5.*t454,-qi2_1.*t443-qi2_2.*t445-qi2_3.*t447-qi2_4.*t449-qi2_5.*t451,qi2_1.*t440+qi2_2.*t441+qi2_3.*t442+qi2_4.*t444+qi2_5.*t446,t438,t439,m0+mi1.*t12+mi2.*t13+mi3.*t14+mi4.*t15+mi5.*t16,qi3_1.*t448+qi3_2.*t450+qi3_3.*t452+qi3_4.*t453+qi3_5.*t454];
mt2 = [-qi3_1.*t443-qi3_2.*t445-qi3_3.*t447-qi3_4.*t449-qi3_5.*t451,qi3_1.*t440+qi3_2.*t441+qi3_3.*t442+qi3_4.*t444+qi3_5.*t446,rho2_1.*t394+rho2_2.*t397+rho2_3.*t401-rho3_1.*t393-rho3_2.*t396+rho2_4.*t407-rho3_3.*t399+rho2_5.*t413-rho3_4.*t405-rho3_5.*t411,rho2_1.*t404+rho2_2.*t410-rho3_1.*t402+rho2_3.*t416-rho3_2.*t408+rho2_4.*t422-rho3_3.*t414+rho2_5.*t428-rho3_4.*t420-rho3_5.*t426,rho2_1.*t419+rho2_2.*t425-rho3_1.*t417+rho2_3.*t430-rho3_2.*t423+rho2_4.*t433+rho2_5.*t436-rho3_3.*t429-rho3_4.*t432-rho3_5.*t435];
mt3 = [j01+rho2_1.*t616+rho2_2.*t622-rho3_1.*t614+rho2_3.*t628-rho3_2.*t620+rho2_4.*t631+rho2_5.*t634-rho3_3.*t626-rho3_4.*t630-rho3_5.*t633,-rho2_1.*t601-rho2_2.*t607+rho3_1.*t599+rho3_2.*t605-rho2_3.*t615-rho2_4.*t621+rho3_3.*t612-rho2_5.*t627+rho3_4.*t619+rho3_5.*t625,rho2_1.*t592+rho2_2.*t595-rho3_1.*t591+rho2_3.*t600-rho3_2.*t594+rho2_4.*t606-rho3_3.*t598+rho2_5.*t613-rho3_4.*t604-rho3_5.*t610,-rho1_1.*t394-rho1_2.*t397-rho1_3.*t401-rho1_4.*t407+rho3_1.*t392+rho3_2.*t395-rho1_5.*t413+rho3_3.*t398+rho3_4.*t403+rho3_5.*t409];
mt4 = [-rho1_1.*t404-rho1_2.*t410-rho1_3.*t416+rho3_1.*t400-rho1_4.*t422+rho3_2.*t406-rho1_5.*t428+rho3_3.*t412+rho3_4.*t418+rho3_5.*t424,-rho1_1.*t419-rho1_2.*t425-rho1_3.*t430+rho3_1.*t415-rho1_4.*t433-rho1_5.*t436+rho3_2.*t421+rho3_3.*t427+rho3_4.*t431+rho3_5.*t434,-rho1_1.*t616-rho1_2.*t622-rho1_3.*t628+rho3_1.*t611-rho1_4.*t631-rho1_5.*t634+rho3_2.*t618+rho3_3.*t624+rho3_4.*t629+rho3_5.*t632,j02+rho1_1.*t601+rho1_2.*t607+rho1_3.*t615-rho3_1.*t597+rho1_4.*t621-rho3_2.*t603+rho1_5.*t627-rho3_3.*t609-rho3_4.*t617-rho3_5.*t623];
mt5 = [-rho1_1.*t592-rho1_2.*t595-rho1_3.*t600-rho1_4.*t606+rho3_1.*t590+rho3_2.*t593-rho1_5.*t613+rho3_3.*t596+rho3_4.*t602+rho3_5.*t608,rho1_1.*t393+rho1_2.*t396+rho1_3.*t399-rho2_1.*t392-rho2_2.*t395+rho1_4.*t405-rho2_3.*t398+rho1_5.*t411-rho2_4.*t403-rho2_5.*t409,rho1_1.*t402+rho1_2.*t408-rho2_1.*t400+rho1_3.*t414-rho2_2.*t406+rho1_4.*t420-rho2_3.*t412+rho1_5.*t426-rho2_4.*t418-rho2_5.*t424,rho1_1.*t417+rho1_2.*t423-rho2_1.*t415+rho1_3.*t429-rho2_2.*t421+rho1_4.*t432+rho1_5.*t435-rho2_3.*t427-rho2_4.*t431-rho2_5.*t434];
mt6 = [rho1_1.*t614+rho1_2.*t620-rho2_1.*t611+rho1_3.*t626-rho2_2.*t618+rho1_4.*t630-rho2_3.*t624+rho1_5.*t633-rho2_4.*t629-rho2_5.*t632,-rho1_1.*t599-rho1_2.*t605+rho2_1.*t597-rho1_3.*t612+rho2_2.*t603+rho2_3.*t609-rho1_4.*t619-rho1_5.*t625+rho2_4.*t617+rho2_5.*t623,j03+rho1_1.*t591+rho1_2.*t594+rho1_3.*t598-rho2_1.*t590-rho2_2.*t593+rho1_4.*t604-rho2_3.*t596+rho1_5.*t610-rho2_4.*t602-rho2_5.*t608];
A = reshape([mt1,mt2,mt3,mt4,mt5,mt6],6,6);
end
