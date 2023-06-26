function A = zup_eul_Addx0do0_4(in1,in2,in3)
%zup_eul_Addx0do0_4
%    A = zup_eul_Addx0do0_4(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/06/26 22:46:31

X4 = in1(4,:);
X5 = in1(5,:);
X6 = in1(6,:);
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
j01 = in3(:,3);
j02 = in3(:,4);
j03 = in3(:,5);
m0 = in3(:,2);
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
t2 = X13.^2;
t3 = X14.^2;
t4 = X15.^2;
t5 = X16.^2;
t6 = X17.^2;
t7 = X18.^2;
t8 = X19.^2;
t9 = X20.^2;
t10 = X21.^2;
t11 = X22.^2;
t12 = X23.^2;
t13 = X24.^2;
t14 = X13.*X14.*mi1;
t15 = X13.*X15.*mi1;
t16 = X14.*X15.*mi1;
t17 = X16.*X17.*mi2;
t18 = X16.*X18.*mi2;
t19 = X17.*X18.*mi2;
t20 = X19.*X20.*mi3;
t21 = X19.*X21.*mi3;
t22 = X20.*X21.*mi3;
t23 = X22.*X23.*mi4;
t24 = X22.*X24.*mi4;
t25 = X23.*X24.*mi4;
t26 = X4./2.0;
t27 = X5./2.0;
t28 = X6./2.0;
t29 = cos(t26);
t30 = cos(t27);
t31 = cos(t28);
t32 = sin(t26);
t33 = sin(t27);
t34 = sin(t28);
t35 = -t14;
t36 = -t15;
t37 = -t17;
t38 = -t18;
t39 = -t20;
t40 = -t21;
t41 = -t23;
t42 = -t24;
t43 = t16+t19+t22+t25;
t44 = t29.*t30.*t31;
t45 = t29.*t30.*t34;
t46 = t29.*t31.*t33;
t47 = t30.*t31.*t32;
t48 = t29.*t33.*t34;
t49 = t30.*t32.*t34;
t50 = t31.*t32.*t33;
t51 = t32.*t33.*t34;
t54 = t35+t37+t39+t41;
t55 = t36+t38+t40+t42;
t52 = -t48;
t53 = -t50;
t56 = t44+t51;
t57 = t46+t49;
t58 = t56.^2;
t59 = t57.^2;
t60 = t45+t53;
t61 = t47+t52;
t67 = t56.*t57.*2.0;
t62 = t60.^2;
t63 = t61.^2;
t64 = -t59;
t68 = t56.*t60.*2.0;
t69 = t56.*t61.*2.0;
t70 = t57.*t60.*2.0;
t71 = t57.*t61.*2.0;
t74 = t60.*t61.*2.0;
t65 = -t62;
t66 = -t63;
t72 = -t70;
t73 = -t71;
t75 = -t74;
t76 = t67+t74;
t77 = t68+t71;
t78 = t69+t70;
t79 = t67+t75;
t80 = t68+t73;
t81 = t69+t72;
t82 = mi1.*rho1_1.*t76;
t83 = mi1.*rho1_1.*t78;
t84 = mi1.*rho1_2.*t76;
t85 = mi1.*rho1_2.*t77;
t86 = mi1.*rho1_3.*t77;
t87 = mi1.*rho1_3.*t78;
t88 = mi2.*rho2_1.*t76;
t89 = mi2.*rho2_1.*t78;
t90 = mi2.*rho2_2.*t76;
t91 = mi2.*rho2_2.*t77;
t92 = mi2.*rho2_3.*t77;
t93 = mi2.*rho2_3.*t78;
t94 = mi3.*rho3_1.*t76;
t95 = mi3.*rho3_1.*t78;
t96 = mi3.*rho3_2.*t76;
t97 = mi3.*rho3_2.*t77;
t98 = mi3.*rho3_3.*t77;
t99 = mi3.*rho3_3.*t78;
t100 = mi4.*rho4_1.*t76;
t101 = mi4.*rho4_1.*t78;
t102 = mi4.*rho4_2.*t76;
t103 = mi4.*rho4_2.*t77;
t104 = mi4.*rho4_3.*t77;
t105 = mi4.*rho4_3.*t78;
t130 = t14.*t76;
t131 = t14.*t77;
t132 = t15.*t76;
t133 = t15.*t78;
t134 = t16.*t77;
t135 = t16.*t78;
t136 = t17.*t76;
t137 = t17.*t77;
t138 = t18.*t76;
t139 = t18.*t78;
t140 = t19.*t77;
t141 = t19.*t78;
t142 = t20.*t76;
t143 = t20.*t77;
t144 = t21.*t76;
t145 = t21.*t78;
t146 = t22.*t77;
t147 = t22.*t78;
t148 = t23.*t76;
t149 = t23.*t77;
t150 = t24.*t76;
t151 = t24.*t78;
t152 = t25.*t77;
t153 = t25.*t78;
t182 = mi1.*t2.*t76;
t183 = mi1.*t3.*t77;
t184 = mi1.*t4.*t78;
t185 = mi2.*t5.*t76;
t186 = mi2.*t6.*t77;
t187 = mi2.*t7.*t78;
t188 = mi3.*t8.*t76;
t189 = mi3.*t9.*t77;
t190 = mi3.*t10.*t78;
t191 = mi4.*t11.*t76;
t192 = mi4.*t12.*t77;
t193 = mi4.*t13.*t78;
t246 = t58+t59+t65+t66;
t247 = t58+t62+t64+t66;
t248 = t58+t63+t64+t65;
t106 = mi1.*rho1_1.*t80;
t107 = mi1.*rho1_1.*t81;
t108 = mi1.*rho1_2.*t79;
t109 = mi1.*rho1_2.*t81;
t110 = mi1.*rho1_3.*t79;
t111 = mi1.*rho1_3.*t80;
t112 = mi2.*rho2_1.*t80;
t113 = mi2.*rho2_1.*t81;
t114 = mi2.*rho2_2.*t79;
t115 = mi2.*rho2_2.*t81;
t116 = mi2.*rho2_3.*t79;
t117 = mi2.*rho2_3.*t80;
t118 = mi3.*rho3_1.*t80;
t119 = mi3.*rho3_1.*t81;
t120 = mi3.*rho3_2.*t79;
t121 = mi3.*rho3_2.*t81;
t122 = mi3.*rho3_3.*t79;
t123 = mi3.*rho3_3.*t80;
t124 = mi4.*rho4_1.*t80;
t125 = mi4.*rho4_1.*t81;
t126 = mi4.*rho4_2.*t79;
t127 = mi4.*rho4_2.*t81;
t128 = mi4.*rho4_3.*t79;
t129 = mi4.*rho4_3.*t80;
t162 = t16.*t79;
t163 = t16.*t81;
t168 = t19.*t79;
t169 = t19.*t81;
t174 = t22.*t79;
t175 = t22.*t81;
t180 = t25.*t79;
t181 = t25.*t81;
t198 = mi1.*t2.*t80;
t199 = mi1.*t3.*t81;
t200 = mi1.*t4.*t79;
t201 = mi2.*t5.*t80;
t202 = mi2.*t6.*t81;
t203 = mi2.*t7.*t79;
t204 = mi3.*t8.*t80;
t205 = mi3.*t9.*t81;
t206 = mi3.*t10.*t79;
t207 = mi4.*t11.*t80;
t208 = mi4.*t12.*t81;
t209 = mi4.*t13.*t79;
t210 = t35.*t80;
t211 = t35.*t81;
t212 = t36.*t79;
t213 = t36.*t80;
t216 = t37.*t80;
t217 = t37.*t81;
t218 = t38.*t79;
t219 = t38.*t80;
t222 = t39.*t80;
t223 = t39.*t81;
t224 = t40.*t79;
t225 = t40.*t80;
t228 = t41.*t80;
t229 = t41.*t81;
t230 = t42.*t79;
t231 = t42.*t80;
t249 = mi1.*rho1_1.*t246;
t250 = mi1.*rho1_1.*t247;
t251 = mi1.*rho1_2.*t247;
t252 = mi1.*rho1_2.*t248;
t253 = mi1.*rho1_3.*t246;
t254 = mi1.*rho1_3.*t248;
t255 = mi2.*rho2_1.*t246;
t256 = mi2.*rho2_1.*t247;
t257 = mi2.*rho2_2.*t247;
t258 = mi2.*rho2_2.*t248;
t259 = mi2.*rho2_3.*t246;
t260 = mi2.*rho2_3.*t248;
t261 = mi3.*rho3_1.*t246;
t262 = mi3.*rho3_1.*t247;
t263 = mi3.*rho3_2.*t247;
t264 = mi3.*rho3_2.*t248;
t265 = mi3.*rho3_3.*t246;
t266 = mi3.*rho3_3.*t248;
t267 = mi4.*rho4_1.*t246;
t268 = mi4.*rho4_1.*t247;
t269 = mi4.*rho4_2.*t247;
t270 = mi4.*rho4_2.*t248;
t271 = mi4.*rho4_3.*t246;
t272 = mi4.*rho4_3.*t248;
t273 = t14.*t246;
t274 = t14.*t248;
t275 = t15.*t247;
t276 = t15.*t248;
t277 = t16.*t246;
t278 = t16.*t247;
t279 = t17.*t246;
t280 = t17.*t248;
t281 = t18.*t247;
t282 = t18.*t248;
t283 = t19.*t246;
t284 = t19.*t247;
t285 = t20.*t246;
t286 = t20.*t248;
t287 = t21.*t247;
t288 = t21.*t248;
t289 = t22.*t246;
t290 = t22.*t247;
t291 = t23.*t246;
t292 = t23.*t248;
t293 = t24.*t247;
t294 = t24.*t248;
t295 = t25.*t246;
t296 = t25.*t247;
t309 = mi1.*t2.*t248;
t310 = mi1.*t3.*t246;
t311 = mi1.*t4.*t247;
t312 = mi2.*t5.*t248;
t313 = mi2.*t6.*t246;
t314 = mi2.*t7.*t247;
t315 = mi3.*t8.*t248;
t316 = mi3.*t9.*t246;
t317 = mi3.*t10.*t247;
t318 = mi4.*t11.*t248;
t319 = mi4.*t12.*t246;
t320 = mi4.*t13.*t247;
t194 = -t108;
t195 = -t114;
t196 = -t120;
t197 = -t126;
t214 = -t162;
t215 = -t163;
t220 = -t168;
t221 = -t169;
t226 = -t174;
t227 = -t175;
t232 = -t180;
t233 = -t181;
t234 = -t198;
t235 = -t199;
t236 = -t200;
t237 = -t201;
t238 = -t202;
t239 = -t203;
t240 = -t204;
t241 = -t205;
t242 = -t206;
t243 = -t207;
t244 = -t208;
t245 = -t209;
t297 = -t250;
t298 = -t251;
t299 = -t252;
t300 = -t256;
t301 = -t257;
t302 = -t258;
t303 = -t262;
t304 = -t263;
t305 = -t264;
t306 = -t268;
t307 = -t269;
t308 = -t270;
t321 = t84+t111;
t322 = t90+t117;
t323 = t96+t123;
t324 = t102+t129;
t349 = t85+t249;
t350 = t82+t254;
t351 = t91+t255;
t352 = t88+t260;
t353 = t97+t261;
t354 = t94+t266;
t355 = t103+t267;
t356 = t100+t272;
t357 = t109+t253;
t358 = t115+t259;
t359 = t121+t265;
t360 = t127+t271;
t405 = t182+t211+t275;
t406 = t131+t212+t309;
t409 = t135+t210+t310;
t411 = t184+t213+t277;
t414 = t185+t217+t281;
t415 = t137+t218+t312;
t418 = t141+t216+t313;
t420 = t187+t219+t283;
t423 = t188+t223+t287;
t424 = t143+t224+t315;
t427 = t147+t222+t316;
t429 = t190+t225+t289;
t432 = t191+t229+t293;
t433 = t149+t230+t318;
t436 = t153+t228+t319;
t438 = t193+t231+t295;
t325 = X13.*t321;
t326 = X16.*t322;
t327 = X19.*t323;
t328 = X22.*t324;
t329 = t83+t194;
t331 = t89+t195;
t333 = t95+t196;
t335 = t101+t197;
t361 = t87+t298;
t362 = t93+t301;
t363 = t99+t304;
t364 = t105+t307;
t365 = X13.*t350;
t366 = X14.*t349;
t367 = X16.*t352;
t368 = X17.*t351;
t369 = X19.*t354;
t370 = X20.*t353;
t371 = X22.*t356;
t372 = X23.*t355;
t373 = t106+t299;
t374 = t110+t297;
t375 = t112+t302;
t376 = t116+t300;
t377 = t118+t305;
t378 = t122+t303;
t379 = t124+t308;
t380 = t128+t306;
t381 = X14.*t357;
t382 = X17.*t358;
t383 = X20.*t359;
t384 = X23.*t360;
t407 = t133+t234+t273;
t408 = t183+t214+t274;
t410 = t130+t235+t278;
t412 = t132+t215+t311;
t413 = t134+t236+t276;
t416 = t139+t237+t279;
t417 = t186+t220+t280;
t419 = t136+t238+t284;
t421 = t138+t221+t314;
t422 = t140+t239+t282;
t425 = t145+t240+t285;
t426 = t189+t226+t286;
t428 = t142+t241+t290;
t430 = t144+t227+t317;
t431 = t146+t242+t288;
t434 = t151+t243+t291;
t435 = t192+t232+t292;
t437 = t148+t244+t296;
t439 = t150+t233+t320;
t440 = t152+t245+t294;
t338 = X15.*t329;
t339 = -t325;
t341 = X18.*t331;
t342 = -t326;
t344 = X21.*t333;
t345 = -t327;
t347 = X24.*t335;
t348 = -t328;
t386 = X15.*t361;
t388 = X18.*t362;
t390 = X21.*t363;
t392 = X24.*t364;
t393 = X13.*t373;
t394 = X15.*t374;
t395 = X16.*t375;
t396 = X18.*t376;
t397 = X19.*t377;
t398 = X21.*t378;
t399 = X22.*t379;
t400 = X24.*t380;
t401 = -t393;
t402 = -t395;
t403 = -t397;
t404 = -t399;
t441 = t339+t381+t386;
t442 = t342+t382+t388;
t443 = t345+t383+t390;
t444 = t348+t384+t392;
t504 = -X13.*t80.*(t365-t394+X14.*(t86-t107));
t505 = -X14.*t81.*(t365-t394+X14.*(t86-t107));
t506 = -X15.*t79.*(t365-t394+X14.*(t86-t107));
t513 = -X16.*t80.*(t367-t396+X17.*(t92-t113));
t514 = -X17.*t81.*(t367-t396+X17.*(t92-t113));
t515 = -X18.*t79.*(t367-t396+X17.*(t92-t113));
t522 = -X19.*t80.*(t369-t398+X20.*(t98-t119));
t523 = -X20.*t81.*(t369-t398+X20.*(t98-t119));
t524 = -X21.*t79.*(t369-t398+X20.*(t98-t119));
t531 = -X22.*t80.*(t371-t400+X23.*(t104-t125));
t532 = -X23.*t81.*(t371-t400+X23.*(t104-t125));
t533 = -X24.*t79.*(t371-t400+X23.*(t104-t125));
t445 = t338+t366+t401;
t447 = t341+t368+t402;
t449 = t344+t370+t403;
t451 = t347+t372+t404;
t453 = X13.*t76.*t441;
t454 = X14.*t77.*t441;
t455 = X15.*t78.*t441;
t456 = X16.*t76.*t442;
t457 = X17.*t77.*t442;
t458 = X18.*t78.*t442;
t459 = X19.*t76.*t443;
t460 = X20.*t77.*t443;
t461 = X21.*t78.*t443;
t462 = X22.*t76.*t444;
t463 = X23.*t77.*t444;
t464 = X24.*t78.*t444;
t465 = X13.*t80.*t441;
t466 = X14.*t81.*t441;
t467 = X15.*t79.*t441;
t468 = X16.*t80.*t442;
t469 = X17.*t81.*t442;
t470 = X18.*t79.*t442;
t471 = X19.*t80.*t443;
t472 = X20.*t81.*t443;
t473 = X21.*t79.*t443;
t474 = X22.*t80.*t444;
t475 = X23.*t81.*t444;
t476 = X24.*t79.*t444;
t561 = X13.*t248.*t441;
t562 = X14.*t246.*t441;
t563 = X15.*t247.*t441;
t564 = X16.*t248.*t442;
t565 = X17.*t246.*t442;
t566 = X18.*t247.*t442;
t567 = X19.*t248.*t443;
t568 = X20.*t246.*t443;
t569 = X21.*t247.*t443;
t570 = X22.*t248.*t444;
t571 = X23.*t246.*t444;
t572 = X24.*t247.*t444;
t477 = X13.*t76.*t445;
t478 = X14.*t77.*t445;
A = ft_1({X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,j01,j02,j03,m0,mi1,mi2,mi3,mi4,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t10,t104,t107,t11,t113,t119,t12,t125,t13,t2,t246,t247,t248,t3,t365,t367,t369,t371,t394,t396,t398,t4,t400,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t415,t416,t417,t418,t419,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t43,t430,t431,t432,t433,t434,t435,t436,t437,t438,t439,t440,t441,t442,t443,t444,t445,t447,t449,t451,t453,t454,t455,t456,t457,t458,t459,t460,t461,t462,t463,t464,t465,t466,t467,t468,t469,t470,t471,t472,t473,t474,t475,t476,t477,t478,t5,t504,t505,t506,t513,t514,t515,t522,t523,t524,t531,t532,t533,t54,t55,t561,t562,t563,t564,t565,t566,t567,t568,t569,t570,t571,t572,t6,t7,t76,t77,t78,t79,t8,t80,t81,t86,t9,t92,t98});
end
function A = ft_1(ct)
[X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,j01,j02,j03,m0,mi1,mi2,mi3,mi4,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t10,t104,t107,t11,t113,t119,t12,t125,t13,t2,t246,t247,t248,t3,t365,t367,t369,t371,t394,t396,t398,t4,t400,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t415,t416,t417,t418,t419,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t43,t430,t431,t432,t433,t434,t435,t436,t437,t438,t439,t440,t441,t442,t443,t444,t445,t447,t449,t451,t453,t454,t455,t456,t457,t458,t459,t460,t461,t462,t463,t464,t465,t466,t467,t468,t469,t470,t471,t472,t473,t474,t475,t476,t477,t478,t5,t504,t505,t506,t513,t514,t515,t522,t523,t524,t531,t532,t533,t54,t55,t561,t562,t563,t564,t565,t566,t567,t568,t569,t570,t571,t572,t6,t7,t76,t77,t78,t79,t8,t80,t81,t86,t9,t92,t98] = ct{:};
t479 = X15.*t78.*t445;
t483 = X16.*t76.*t447;
t484 = X17.*t77.*t447;
t485 = X18.*t78.*t447;
t489 = X19.*t76.*t449;
t490 = X20.*t77.*t449;
t491 = X21.*t78.*t449;
t495 = X22.*t76.*t451;
t496 = X23.*t77.*t451;
t497 = X24.*t78.*t451;
t501 = X13.*t80.*t445;
t502 = X14.*t81.*t445;
t503 = X15.*t79.*t445;
t507 = -t465;
t508 = -t466;
t509 = -t467;
t510 = X16.*t80.*t447;
t511 = X17.*t81.*t447;
t512 = X18.*t79.*t447;
t516 = -t468;
t517 = -t469;
t518 = -t470;
t519 = X19.*t80.*t449;
t520 = X20.*t81.*t449;
t521 = X21.*t79.*t449;
t525 = -t471;
t526 = -t472;
t527 = -t473;
t528 = X22.*t80.*t451;
t529 = X23.*t81.*t451;
t530 = X24.*t79.*t451;
t534 = -t474;
t535 = -t475;
t536 = -t476;
t573 = X13.*t248.*t445;
t574 = X14.*t246.*t445;
t575 = X15.*t247.*t445;
t579 = X16.*t248.*t447;
t580 = X17.*t246.*t447;
t581 = X18.*t247.*t447;
t585 = X19.*t248.*t449;
t586 = X20.*t246.*t449;
t587 = X21.*t247.*t449;
t591 = X22.*t248.*t451;
t592 = X23.*t246.*t451;
t593 = X24.*t247.*t451;
t537 = -t501;
t538 = -t502;
t539 = -t503;
t543 = -t510;
t544 = -t511;
t545 = -t512;
t549 = -t519;
t550 = -t520;
t551 = -t521;
t555 = -t528;
t556 = -t529;
t557 = -t530;
t597 = t454+t509+t561;
t598 = t455+t507+t562;
t599 = t453+t508+t563;
t600 = t457+t518+t564;
t601 = t458+t516+t565;
t602 = t456+t517+t566;
t603 = t460+t527+t567;
t604 = t461+t525+t568;
t605 = t459+t526+t569;
t606 = t463+t536+t570;
t607 = t464+t534+t571;
t608 = t462+t535+t572;
t609 = t478+t539+t573;
t610 = t479+t537+t574;
t611 = t477+t538+t575;
t615 = t484+t545+t579;
t616 = t485+t543+t580;
t617 = t483+t544+t581;
t621 = t490+t551+t585;
t622 = t491+t549+t586;
t623 = t489+t550+t587;
t627 = t496+t557+t591;
t628 = t497+t555+t592;
t629 = t495+t556+t593;
et1 = -rho1_2.*(t505+X13.*t76.*(t365-t394+X14.*(t86-t107))+X15.*t247.*(t365-t394+X14.*(t86-t107)))+rho1_3.*(t504+X15.*t78.*(t365-t394+X14.*(t86-t107))+X14.*t246.*(t365-t394+X14.*(t86-t107)))-rho2_2.*(t514+X16.*t76.*(t367-t396+X17.*(t92-t113))+X18.*t247.*(t367-t396+X17.*(t92-t113)))+rho2_3.*(t513+X18.*t78.*(t367-t396+X17.*(t92-t113))+X17.*t246.*(t367-t396+X17.*(t92-t113)))-rho3_2.*(t523+X19.*t76.*(t369-t398+X20.*(t98-t119))+X21.*t247.*(t369-t398+X20.*(t98-t119)));
et2 = rho3_3.*(t522+X21.*t78.*(t369-t398+X20.*(t98-t119))+X20.*t246.*(t369-t398+X20.*(t98-t119)))-rho4_2.*(t532+X22.*t76.*(t371-t400+X23.*(t104-t125))+X24.*t247.*(t371-t400+X23.*(t104-t125)))+rho4_3.*(t531+X24.*t78.*(t371-t400+X23.*(t104-t125))+X23.*t246.*(t371-t400+X23.*(t104-t125)));
et3 = j02+rho1_1.*(t505+X13.*t76.*(t365-t394+X14.*(t86-t107))+X15.*t247.*(t365-t394+X14.*(t86-t107)))+rho1_3.*(t506+X14.*t77.*(t365-t394+X14.*(t86-t107))+X13.*t248.*(t365-t394+X14.*(t86-t107)))+rho2_1.*(t514+X16.*t76.*(t367-t396+X17.*(t92-t113))+X18.*t247.*(t367-t396+X17.*(t92-t113)))+rho2_3.*(t515+X17.*t77.*(t367-t396+X17.*(t92-t113))+X16.*t248.*(t367-t396+X17.*(t92-t113)))+rho3_1.*(t523+X19.*t76.*(t369-t398+X20.*(t98-t119))+X21.*t247.*(t369-t398+X20.*(t98-t119)));
et4 = rho3_3.*(t524+X20.*t77.*(t369-t398+X20.*(t98-t119))+X19.*t248.*(t369-t398+X20.*(t98-t119)))+rho4_1.*(t532+X22.*t76.*(t371-t400+X23.*(t104-t125))+X24.*t247.*(t371-t400+X23.*(t104-t125)))+rho4_3.*(t533+X23.*t77.*(t371-t400+X23.*(t104-t125))+X22.*t248.*(t371-t400+X23.*(t104-t125)));
et5 = -rho1_1.*(t504+X15.*t78.*(t365-t394+X14.*(t86-t107))+X14.*t246.*(t365-t394+X14.*(t86-t107)))-rho1_2.*(t506+X14.*t77.*(t365-t394+X14.*(t86-t107))+X13.*t248.*(t365-t394+X14.*(t86-t107)))-rho2_1.*(t513+X18.*t78.*(t367-t396+X17.*(t92-t113))+X17.*t246.*(t367-t396+X17.*(t92-t113)))-rho2_2.*(t515+X17.*t77.*(t367-t396+X17.*(t92-t113))+X16.*t248.*(t367-t396+X17.*(t92-t113)))-rho3_1.*(t522+X21.*t78.*(t369-t398+X20.*(t98-t119))+X20.*t246.*(t369-t398+X20.*(t98-t119)));
et6 = -rho3_2.*(t524+X20.*t77.*(t369-t398+X20.*(t98-t119))+X19.*t248.*(t369-t398+X20.*(t98-t119)))-rho4_1.*(t531+X24.*t78.*(t371-t400+X23.*(t104-t125))+X23.*t246.*(t371-t400+X23.*(t104-t125)))-rho4_2.*(t533+X23.*t77.*(t371-t400+X23.*(t104-t125))+X22.*t248.*(t371-t400+X23.*(t104-t125)));
mt1 = [m0+mi1.*t2+mi2.*t5+mi3.*t8+mi4.*t11,t54,t55,X13.*t441+X16.*t442+X19.*t443+X22.*t444,X13.*(t365-t394+X14.*(t86-t107))+X16.*(t367-t396+X17.*(t92-t113))+X19.*(t369-t398+X20.*(t98-t119))+X22.*(t371-t400+X23.*(t104-t125)),-X13.*t445-X16.*t447-X19.*t449-X22.*t451,t54,m0+mi1.*t3+mi2.*t6+mi3.*t9+mi4.*t12,t43,-X14.*t441-X17.*t442-X20.*t443-X23.*t444];
mt2 = [-X14.*(t365-t394+X14.*(t86-t107))-X17.*(t367-t396+X17.*(t92-t113))-X20.*(t369-t398+X20.*(t98-t119))-X23.*(t371-t400+X23.*(t104-t125)),X14.*t445+X17.*t447+X20.*t449+X23.*t451,t55,t43,m0+mi1.*t4+mi2.*t7+mi3.*t10+mi4.*t13,-X15.*t441-X18.*t442-X21.*t443-X24.*t444,-X15.*(t365-t394+X14.*(t86-t107))-X18.*(t367-t396+X17.*(t92-t113))-X21.*(t369-t398+X20.*(t98-t119))-X24.*(t371-t400+X23.*(t104-t125))];
mt3 = [X15.*t445+X18.*t447+X21.*t449+X24.*t451,-rho1_2.*t405+rho1_3.*t407-rho2_2.*t414+rho2_3.*t416-rho3_2.*t423+rho3_3.*t425-rho4_2.*t432+rho4_3.*t434,rho1_2.*t410-rho1_3.*t409+rho2_2.*t419-rho2_3.*t418+rho3_2.*t428-rho3_3.*t427+rho4_2.*t437-rho4_3.*t436,rho1_2.*t412-rho1_3.*t411+rho2_2.*t421-rho2_3.*t420+rho3_2.*t430-rho3_3.*t429+rho4_2.*t439-rho4_3.*t438,j01-rho1_2.*t599+rho1_3.*t598-rho2_2.*t602+rho2_3.*t601-rho3_2.*t605+rho3_3.*t604-rho4_2.*t608+rho4_3.*t607,et1+et2,rho1_2.*t611-rho1_3.*t610+rho2_2.*t617-rho2_3.*t616+rho3_2.*t623-rho3_3.*t622+rho4_2.*t629-rho4_3.*t628];
mt4 = [rho1_1.*t405+rho1_3.*t406+rho2_1.*t414+rho2_3.*t415+rho3_1.*t423+rho3_3.*t424+rho4_1.*t432+rho4_3.*t433,-rho1_1.*t410-rho1_3.*t408-rho2_1.*t419-rho2_3.*t417-rho3_1.*t428-rho3_3.*t426-rho4_1.*t437-rho4_3.*t435,-rho1_1.*t412-rho1_3.*t413-rho2_1.*t421-rho2_3.*t422-rho3_1.*t430-rho3_3.*t431-rho4_1.*t439-rho4_3.*t440,rho1_1.*t599+rho1_3.*t597+rho2_1.*t602+rho2_3.*t600+rho3_1.*t605+rho3_3.*t603+rho4_1.*t608+rho4_3.*t606,et3+et4,-rho1_1.*t611-rho1_3.*t609-rho2_1.*t617-rho2_3.*t615-rho3_1.*t623-rho3_3.*t621-rho4_1.*t629-rho4_3.*t627];
mt5 = [-rho1_1.*t407-rho1_2.*t406-rho2_1.*t416-rho2_2.*t415-rho3_1.*t425-rho3_2.*t424-rho4_1.*t434-rho4_2.*t433,rho1_1.*t409+rho1_2.*t408+rho2_1.*t418+rho2_2.*t417+rho3_1.*t427+rho3_2.*t426+rho4_1.*t436+rho4_2.*t435,rho1_1.*t411+rho1_2.*t413+rho2_1.*t420+rho2_2.*t422+rho3_1.*t429+rho3_2.*t431+rho4_1.*t438+rho4_2.*t440,-rho1_1.*t598-rho1_2.*t597-rho2_1.*t601-rho2_2.*t600-rho3_1.*t604-rho3_2.*t603-rho4_1.*t607-rho4_2.*t606,et5+et6,j03+rho1_1.*t610+rho1_2.*t609+rho2_1.*t616+rho2_2.*t615+rho3_1.*t622+rho3_2.*t621+rho4_1.*t628+rho4_2.*t627];
A = reshape([mt1,mt2,mt3,mt4,mt5],6,6);
end
