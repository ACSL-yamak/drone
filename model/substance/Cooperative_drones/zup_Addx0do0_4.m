function A = zup_Addx0do0_4(in1,in2,in3)
%zup_Addx0do0_4
%    A = zup_Addx0do0_4(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/06/24 11:59:32

X4 = in1(4,:);
X5 = in1(5,:);
X6 = in1(6,:);
X7 = in1(7,:);
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
t2 = X4.^2;
t3 = X5.^2;
t4 = X6.^2;
t5 = X7.^2;
t6 = X14.^2;
t7 = X15.^2;
t8 = X16.^2;
t9 = X17.^2;
t10 = X18.^2;
t11 = X19.^2;
t12 = X20.^2;
t13 = X21.^2;
t14 = X22.^2;
t15 = X23.^2;
t16 = X24.^2;
t17 = X25.^2;
t18 = X4.*X5.*2.0;
t19 = X4.*X6.*2.0;
t20 = X4.*X7.*2.0;
t21 = X5.*X6.*2.0;
t22 = X5.*X7.*2.0;
t23 = X6.*X7.*2.0;
t24 = X14.*X15.*mi1;
t25 = X14.*X16.*mi1;
t26 = X15.*X16.*mi1;
t27 = X17.*X18.*mi2;
t28 = X17.*X19.*mi2;
t29 = X18.*X19.*mi2;
t30 = X20.*X21.*mi3;
t31 = X20.*X22.*mi3;
t32 = X21.*X22.*mi3;
t33 = X23.*X24.*mi4;
t34 = X23.*X25.*mi4;
t35 = X24.*X25.*mi4;
t36 = -t21;
t37 = -t22;
t38 = -t23;
t39 = -t3;
t40 = -t4;
t41 = -t5;
t42 = -t24;
t43 = -t25;
t44 = -t27;
t45 = -t28;
t46 = -t30;
t47 = -t31;
t48 = -t33;
t49 = -t34;
t50 = t18+t23;
t51 = t19+t22;
t52 = t20+t21;
t232 = t26+t29+t32+t35;
t53 = t18+t38;
t54 = t19+t37;
t55 = t20+t36;
t56 = mi1.*rho1_1.*t50;
t57 = mi1.*rho1_1.*t51;
t58 = mi1.*rho1_2.*t51;
t59 = mi1.*rho1_2.*t52;
t60 = mi1.*rho1_3.*t50;
t61 = mi1.*rho1_3.*t52;
t62 = mi2.*rho2_1.*t50;
t63 = mi2.*rho2_1.*t51;
t64 = mi2.*rho2_2.*t51;
t65 = mi2.*rho2_2.*t52;
t66 = mi2.*rho2_3.*t50;
t67 = mi2.*rho2_3.*t52;
t68 = mi3.*rho3_1.*t50;
t69 = mi3.*rho3_1.*t51;
t70 = mi3.*rho3_2.*t51;
t71 = mi3.*rho3_2.*t52;
t72 = mi3.*rho3_3.*t50;
t73 = mi3.*rho3_3.*t52;
t74 = mi4.*rho4_1.*t50;
t75 = mi4.*rho4_1.*t51;
t76 = mi4.*rho4_2.*t51;
t77 = mi4.*rho4_2.*t52;
t78 = mi4.*rho4_3.*t50;
t79 = mi4.*rho4_3.*t52;
t104 = t24.*t51;
t105 = t24.*t52;
t106 = t25.*t50;
t107 = t25.*t51;
t108 = t26.*t50;
t109 = t26.*t52;
t110 = t27.*t51;
t111 = t27.*t52;
t112 = t28.*t50;
t113 = t28.*t51;
t114 = t29.*t50;
t115 = t29.*t52;
t116 = t30.*t51;
t117 = t30.*t52;
t118 = t31.*t50;
t119 = t31.*t51;
t120 = t32.*t50;
t121 = t32.*t52;
t122 = t33.*t51;
t123 = t33.*t52;
t124 = t34.*t50;
t125 = t34.*t51;
t126 = t35.*t50;
t127 = t35.*t52;
t128 = mi1.*t6.*t51;
t129 = mi1.*t7.*t52;
t130 = mi1.*t8.*t50;
t131 = mi2.*t9.*t51;
t132 = mi2.*t10.*t52;
t133 = mi2.*t11.*t50;
t134 = mi3.*t12.*t51;
t135 = mi3.*t13.*t52;
t136 = mi3.*t14.*t50;
t137 = mi4.*t15.*t51;
t138 = mi4.*t16.*t52;
t139 = mi4.*t17.*t50;
t233 = t2+t5+t39+t40;
t234 = t2+t4+t39+t41;
t235 = t2+t3+t40+t41;
t296 = t42+t44+t46+t48;
t297 = t43+t45+t47+t49;
t80 = mi1.*rho1_1.*t53;
t81 = mi1.*rho1_1.*t55;
t82 = mi1.*rho1_2.*t53;
t83 = mi1.*rho1_2.*t54;
t84 = mi1.*rho1_3.*t54;
t85 = mi1.*rho1_3.*t55;
t86 = mi2.*rho2_1.*t53;
t87 = mi2.*rho2_1.*t55;
t88 = mi2.*rho2_2.*t53;
t89 = mi2.*rho2_2.*t54;
t90 = mi2.*rho2_3.*t54;
t91 = mi2.*rho2_3.*t55;
t92 = mi3.*rho3_1.*t53;
t93 = mi3.*rho3_1.*t55;
t94 = mi3.*rho3_2.*t53;
t95 = mi3.*rho3_2.*t54;
t96 = mi3.*rho3_3.*t54;
t97 = mi3.*rho3_3.*t55;
t98 = mi4.*rho4_1.*t53;
t99 = mi4.*rho4_1.*t55;
t100 = mi4.*rho4_2.*t53;
t101 = mi4.*rho4_2.*t54;
t102 = mi4.*rho4_3.*t54;
t103 = mi4.*rho4_3.*t55;
t152 = t26.*t53;
t153 = t26.*t54;
t158 = t29.*t53;
t159 = t29.*t54;
t164 = t32.*t53;
t165 = t32.*t54;
t170 = t35.*t53;
t171 = t35.*t54;
t172 = mi1.*t6.*t55;
t173 = mi1.*t7.*t53;
t174 = mi1.*t8.*t54;
t175 = mi2.*t9.*t55;
t176 = mi2.*t10.*t53;
t177 = mi2.*t11.*t54;
t178 = mi3.*t12.*t55;
t179 = mi3.*t13.*t53;
t180 = mi3.*t14.*t54;
t181 = mi4.*t15.*t55;
t182 = mi4.*t16.*t53;
t183 = mi4.*t17.*t54;
t196 = t42.*t53;
t197 = t42.*t55;
t198 = t43.*t54;
t199 = t43.*t55;
t202 = t44.*t53;
t203 = t44.*t55;
t204 = t45.*t54;
t205 = t45.*t55;
t208 = t46.*t53;
t209 = t46.*t55;
t210 = t47.*t54;
t211 = t47.*t55;
t214 = t48.*t53;
t215 = t48.*t55;
t216 = t49.*t54;
t217 = t49.*t55;
t236 = mi1.*rho1_1.*t233;
t237 = mi1.*rho1_1.*t234;
t238 = mi1.*rho1_2.*t233;
t239 = mi1.*rho1_2.*t235;
t240 = mi1.*rho1_3.*t234;
t241 = mi1.*rho1_3.*t235;
t242 = mi2.*rho2_1.*t233;
t243 = mi2.*rho2_1.*t234;
t244 = mi2.*rho2_2.*t233;
t245 = mi2.*rho2_2.*t235;
t246 = mi2.*rho2_3.*t234;
t247 = mi2.*rho2_3.*t235;
t248 = mi3.*rho3_1.*t233;
t249 = mi3.*rho3_1.*t234;
t250 = mi3.*rho3_2.*t233;
t251 = mi3.*rho3_2.*t235;
t252 = mi3.*rho3_3.*t234;
t253 = mi3.*rho3_3.*t235;
t254 = mi4.*rho4_1.*t233;
t255 = mi4.*rho4_1.*t234;
t256 = mi4.*rho4_2.*t233;
t257 = mi4.*rho4_2.*t235;
t258 = mi4.*rho4_3.*t234;
t259 = mi4.*rho4_3.*t235;
t260 = t24.*t234;
t261 = t24.*t235;
t262 = t25.*t233;
t263 = t25.*t235;
t264 = t26.*t233;
t265 = t26.*t234;
t266 = t27.*t234;
t267 = t27.*t235;
t268 = t28.*t233;
t269 = t28.*t235;
t270 = t29.*t233;
t271 = t29.*t234;
t272 = t30.*t234;
t273 = t30.*t235;
t274 = t31.*t233;
t275 = t31.*t235;
t276 = t32.*t233;
t277 = t32.*t234;
t278 = t33.*t234;
t279 = t33.*t235;
t280 = t34.*t233;
t281 = t34.*t235;
t282 = t35.*t233;
t283 = t35.*t234;
t284 = mi1.*t6.*t235;
t285 = mi1.*t7.*t234;
t286 = mi1.*t8.*t233;
t287 = mi2.*t9.*t235;
t288 = mi2.*t10.*t234;
t289 = mi2.*t11.*t233;
t290 = mi3.*t12.*t235;
t291 = mi3.*t13.*t234;
t292 = mi3.*t14.*t233;
t293 = mi4.*t15.*t235;
t294 = mi4.*t16.*t234;
t295 = mi4.*t17.*t233;
t185 = -t83;
t188 = -t89;
t191 = -t95;
t194 = -t101;
t200 = -t152;
t201 = -t153;
t206 = -t158;
t207 = -t159;
t212 = -t164;
t213 = -t165;
t218 = -t170;
t219 = -t171;
t220 = -t172;
t221 = -t173;
t222 = -t174;
t223 = -t175;
t224 = -t176;
t225 = -t177;
t226 = -t178;
t227 = -t179;
t228 = -t180;
t229 = -t181;
t230 = -t182;
t231 = -t183;
t298 = t58+t85;
t299 = t64+t91;
t300 = t70+t97;
t301 = t76+t103;
t322 = X15.*(t61-t80);
t323 = X18.*(t67-t86);
t324 = X21.*(t73-t92);
t325 = X24.*(t79-t98);
t326 = t59+t237;
t327 = t57+t241;
t328 = t65+t243;
t329 = t63+t247;
t330 = t71+t249;
t331 = t69+t253;
t332 = t77+t255;
t333 = t75+t259;
t334 = t82+t240;
t335 = t88+t246;
t336 = t94+t252;
t337 = t100+t258;
t366 = -X14.*(t81-t239);
t367 = -X16.*(t84-t236);
t369 = -X17.*(t87-t245);
t370 = -X19.*(t90-t242);
t372 = -X20.*(t93-t251);
t373 = -X22.*(t96-t248);
t375 = -X23.*(t99-t257);
t376 = -X25.*(t102-t254);
t378 = t128+t196+t262;
t379 = t105+t198+t284;
t382 = t108+t197+t285;
t384 = t130+t199+t265;
t387 = t131+t202+t268;
t388 = t111+t204+t287;
t391 = t114+t203+t288;
t393 = t133+t205+t271;
t396 = t134+t208+t274;
t397 = t117+t210+t290;
t400 = t120+t209+t291;
t402 = t136+t211+t277;
t405 = t137+t214+t280;
t406 = t123+t216+t293;
t409 = t126+t215+t294;
t411 = t139+t217+t283;
t302 = X14.*t298;
t303 = X17.*t299;
t304 = X20.*t300;
t305 = X23.*t301;
t306 = t56+t185;
t308 = t62+t188;
t310 = t68+t191;
t312 = t74+t194;
t338 = X14.*t327;
t339 = X15.*t326;
t340 = X17.*t329;
t341 = X18.*t328;
t342 = X20.*t331;
t343 = X21.*t330;
t344 = X23.*t333;
t345 = X24.*t332;
t350 = X15.*t334;
t351 = X18.*t335;
t352 = X21.*t336;
t353 = X24.*t337;
t380 = t106+t220+t260;
t381 = t129+t201+t261;
t383 = t104+t221+t264;
t385 = t107+t200+t286;
t386 = t109+t222+t263;
t389 = t112+t223+t266;
t390 = t132+t207+t267;
t392 = t110+t224+t270;
t394 = t113+t206+t289;
t395 = t115+t225+t269;
t398 = t118+t226+t272;
t399 = t135+t213+t273;
t401 = t116+t227+t276;
t403 = t119+t212+t292;
t404 = t121+t228+t275;
t407 = t124+t229+t278;
t408 = t138+t219+t279;
t410 = t122+t230+t282;
t412 = t125+t218+t295;
t413 = t127+t231+t281;
t315 = X16.*t306;
t317 = X19.*t308;
t319 = X22.*t310;
t321 = X25.*t312;
t422 = t322+t338+t367;
t423 = t323+t340+t370;
t424 = t324+t342+t373;
t425 = t325+t344+t376;
t453 = -X14.*t55.*(-t302+t350+X16.*(t60-t238));
t454 = -X15.*t53.*(-t302+t350+X16.*(t60-t238));
t455 = -X16.*t54.*(-t302+t350+X16.*(t60-t238));
t459 = -X17.*t55.*(-t303+t351+X19.*(t66-t244));
t460 = -X18.*t53.*(-t303+t351+X19.*(t66-t244));
t461 = -X19.*t54.*(-t303+t351+X19.*(t66-t244));
t465 = -X20.*t55.*(-t304+t352+X22.*(t72-t250));
t466 = -X21.*t53.*(-t304+t352+X22.*(t72-t250));
t467 = -X22.*t54.*(-t304+t352+X22.*(t72-t250));
t471 = -X23.*t55.*(-t305+t353+X25.*(t78-t256));
t472 = -X24.*t53.*(-t305+t353+X25.*(t78-t256));
t473 = -X25.*t54.*(-t305+t353+X25.*(t78-t256));
t414 = t315+t339+t366;
t416 = t317+t341+t369;
t418 = t319+t343+t372;
t420 = t321+t345+t375;
t474 = X14.*t51.*t422;
t475 = X15.*t52.*t422;
t476 = X16.*t50.*t422;
t477 = X17.*t51.*t423;
t478 = X18.*t52.*t423;
t479 = X19.*t50.*t423;
t480 = X20.*t51.*t424;
t481 = X21.*t52.*t424;
t482 = X22.*t50.*t424;
t483 = X23.*t51.*t425;
t484 = X24.*t52.*t425;
t485 = X25.*t50.*t425;
t489 = X14.*t55.*t422;
t490 = X15.*t53.*t422;
t491 = X16.*t54.*t422;
t498 = X17.*t55.*t423;
t499 = X18.*t53.*t423;
t500 = X19.*t54.*t423;
t507 = X20.*t55.*t424;
t508 = X21.*t53.*t424;
t509 = X22.*t54.*t424;
t516 = X23.*t55.*t425;
t517 = X24.*t53.*t425;
t518 = X25.*t54.*t425;
t558 = X14.*t235.*t422;
t559 = X15.*t234.*t422;
t560 = X16.*t233.*t422;
t561 = X17.*t235.*t423;
t562 = X18.*t234.*t423;
t563 = X19.*t233.*t423;
t564 = X20.*t235.*t424;
t565 = X21.*t234.*t424;
t566 = X22.*t233.*t424;
t567 = X23.*t235.*t425;
t568 = X24.*t234.*t425;
t569 = X25.*t233.*t425;
t426 = X14.*t51.*t414;
t427 = X15.*t52.*t414;
t428 = X16.*t50.*t414;
t432 = X17.*t51.*t416;
t433 = X18.*t52.*t416;
t434 = X19.*t50.*t416;
t438 = X20.*t51.*t418;
t439 = X21.*t52.*t418;
t440 = X22.*t50.*t418;
t444 = X23.*t51.*t420;
t445 = X24.*t52.*t420;
t446 = X25.*t50.*t420;
t450 = X14.*t55.*t414;
t451 = X15.*t53.*t414;
t452 = X16.*t54.*t414;
t456 = X17.*t55.*t416;
t457 = X18.*t53.*t416;
t458 = X19.*t54.*t416;
t462 = X20.*t55.*t418;
t463 = X21.*t53.*t418;
t464 = X22.*t54.*t418;
t468 = X23.*t55.*t420;
t469 = X24.*t53.*t420;
t470 = X25.*t54.*t420;
t522 = -t489;
t523 = -t490;
t524 = -t491;
t525 = -t498;
t526 = -t499;
t527 = -t500;
t528 = -t507;
t529 = -t508;
t530 = -t509;
t531 = -t516;
t532 = -t517;
t533 = -t518;
t534 = X14.*t235.*t414;
t535 = X15.*t234.*t414;
t536 = X16.*t233.*t414;
t540 = X17.*t235.*t416;
t541 = X18.*t234.*t416;
t542 = X19.*t233.*t416;
t546 = X20.*t235.*t418;
t547 = X21.*t234.*t418;
t548 = X22.*t233.*t418;
t552 = X23.*t235.*t420;
t553 = X24.*t234.*t420;
t554 = X25.*t233.*t420;
t486 = -t450;
t487 = -t451;
t488 = -t452;
t495 = -t456;
t496 = -t457;
t497 = -t458;
t504 = -t462;
t505 = -t463;
A = ft_1({X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,j01,j02,j03,m0,mi1,mi2,mi3,mi4,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t10,t11,t12,t13,t14,t15,t16,t17,t232,t233,t234,t235,t238,t244,t250,t256,t296,t297,t302,t303,t304,t305,t350,t351,t352,t353,t378,t379,t380,t381,t382,t383,t384,t385,t386,t387,t388,t389,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t416,t418,t420,t422,t423,t424,t425,t426,t427,t428,t432,t433,t434,t438,t439,t440,t444,t445,t446,t453,t454,t455,t459,t460,t461,t464,t465,t466,t467,t468,t469,t470,t471,t472,t473,t474,t475,t476,t477,t478,t479,t480,t481,t482,t483,t484,t485,t486,t487,t488,t495,t496,t497,t50,t504,t505,t51,t52,t522,t523,t524,t525,t526,t527,t528,t529,t530,t531,t532,t533,t534,t535,t536,t540,t541,t542,t546,t547,t548,t552,t553,t554,t558,t559,t560,t561,t562,t563,t564,t565,t566,t567,t568,t569,t6,t60,t66,t7,t72,t78,t8,t9});
end
function A = ft_1(ct)
[X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,j01,j02,j03,m0,mi1,mi2,mi3,mi4,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t10,t11,t12,t13,t14,t15,t16,t17,t232,t233,t234,t235,t238,t244,t250,t256,t296,t297,t302,t303,t304,t305,t350,t351,t352,t353,t378,t379,t380,t381,t382,t383,t384,t385,t386,t387,t388,t389,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t416,t418,t420,t422,t423,t424,t425,t426,t427,t428,t432,t433,t434,t438,t439,t440,t444,t445,t446,t453,t454,t455,t459,t460,t461,t464,t465,t466,t467,t468,t469,t470,t471,t472,t473,t474,t475,t476,t477,t478,t479,t480,t481,t482,t483,t484,t485,t486,t487,t488,t495,t496,t497,t50,t504,t505,t51,t52,t522,t523,t524,t525,t526,t527,t528,t529,t530,t531,t532,t533,t534,t535,t536,t540,t541,t542,t546,t547,t548,t552,t553,t554,t558,t559,t560,t561,t562,t563,t564,t565,t566,t567,t568,t569,t6,t60,t66,t7,t72,t78,t8,t9] = ct{:};
t506 = -t464;
t513 = -t468;
t514 = -t469;
t515 = -t470;
t594 = t474+t523+t560;
t595 = t475+t524+t558;
t596 = t476+t522+t559;
t597 = t477+t526+t563;
t598 = t478+t527+t561;
t599 = t479+t525+t562;
t600 = t480+t529+t566;
t601 = t481+t530+t564;
t602 = t482+t528+t565;
t603 = t483+t532+t569;
t604 = t484+t533+t567;
t605 = t485+t531+t568;
t570 = t426+t487+t536;
t571 = t427+t488+t534;
t572 = t428+t486+t535;
t576 = t432+t496+t542;
t577 = t433+t497+t540;
t578 = t434+t495+t541;
t582 = t438+t505+t548;
t583 = t439+t506+t546;
t584 = t440+t504+t547;
t588 = t444+t514+t554;
t589 = t445+t515+t552;
t590 = t446+t513+t553;
et1 = j01-rho1_2.*(t454+X14.*t51.*(-t302+t350+X16.*(t60-t238))+X16.*t233.*(-t302+t350+X16.*(t60-t238)))+rho1_3.*(t453+X16.*t50.*(-t302+t350+X16.*(t60-t238))+X15.*t234.*(-t302+t350+X16.*(t60-t238)))-rho2_2.*(t460+X17.*t51.*(-t303+t351+X19.*(t66-t244))+X19.*t233.*(-t303+t351+X19.*(t66-t244)))+rho2_3.*(t459+X19.*t50.*(-t303+t351+X19.*(t66-t244))+X18.*t234.*(-t303+t351+X19.*(t66-t244)))-rho3_2.*(t466+X20.*t51.*(-t304+t352+X22.*(t72-t250))+X22.*t233.*(-t304+t352+X22.*(t72-t250)));
et2 = rho3_3.*(t465+X22.*t50.*(-t304+t352+X22.*(t72-t250))+X21.*t234.*(-t304+t352+X22.*(t72-t250)))-rho4_2.*(t472+X23.*t51.*(-t305+t353+X25.*(t78-t256))+X25.*t233.*(-t305+t353+X25.*(t78-t256)))+rho4_3.*(t471+X25.*t50.*(-t305+t353+X25.*(t78-t256))+X24.*t234.*(-t305+t353+X25.*(t78-t256)));
et3 = rho1_1.*(t454+X14.*t51.*(-t302+t350+X16.*(t60-t238))+X16.*t233.*(-t302+t350+X16.*(t60-t238)))+rho1_3.*(t455+X15.*t52.*(-t302+t350+X16.*(t60-t238))+X14.*t235.*(-t302+t350+X16.*(t60-t238)))+rho2_1.*(t460+X17.*t51.*(-t303+t351+X19.*(t66-t244))+X19.*t233.*(-t303+t351+X19.*(t66-t244)))+rho2_3.*(t461+X18.*t52.*(-t303+t351+X19.*(t66-t244))+X17.*t235.*(-t303+t351+X19.*(t66-t244)))+rho3_1.*(t466+X20.*t51.*(-t304+t352+X22.*(t72-t250))+X22.*t233.*(-t304+t352+X22.*(t72-t250)));
et4 = rho3_3.*(t467+X21.*t52.*(-t304+t352+X22.*(t72-t250))+X20.*t235.*(-t304+t352+X22.*(t72-t250)))+rho4_1.*(t472+X23.*t51.*(-t305+t353+X25.*(t78-t256))+X25.*t233.*(-t305+t353+X25.*(t78-t256)))+rho4_3.*(t473+X24.*t52.*(-t305+t353+X25.*(t78-t256))+X23.*t235.*(-t305+t353+X25.*(t78-t256)));
et5 = -rho1_1.*(t453+X16.*t50.*(-t302+t350+X16.*(t60-t238))+X15.*t234.*(-t302+t350+X16.*(t60-t238)))-rho1_2.*(t455+X15.*t52.*(-t302+t350+X16.*(t60-t238))+X14.*t235.*(-t302+t350+X16.*(t60-t238)))-rho2_1.*(t459+X19.*t50.*(-t303+t351+X19.*(t66-t244))+X18.*t234.*(-t303+t351+X19.*(t66-t244)))-rho2_2.*(t461+X18.*t52.*(-t303+t351+X19.*(t66-t244))+X17.*t235.*(-t303+t351+X19.*(t66-t244)))-rho3_1.*(t465+X22.*t50.*(-t304+t352+X22.*(t72-t250))+X21.*t234.*(-t304+t352+X22.*(t72-t250)));
et6 = -rho3_2.*(t467+X21.*t52.*(-t304+t352+X22.*(t72-t250))+X20.*t235.*(-t304+t352+X22.*(t72-t250)))-rho4_1.*(t471+X25.*t50.*(-t305+t353+X25.*(t78-t256))+X24.*t234.*(-t305+t353+X25.*(t78-t256)))-rho4_2.*(t473+X24.*t52.*(-t305+t353+X25.*(t78-t256))+X23.*t235.*(-t305+t353+X25.*(t78-t256)));
mt1 = [m0+mi1.*t6+mi2.*t9+mi3.*t12+mi4.*t15,t296,t297,X14.*(-t302+t350+X16.*(t60-t238))+X17.*(-t303+t351+X19.*(t66-t244))+X20.*(-t304+t352+X22.*(t72-t250))+X23.*(-t305+t353+X25.*(t78-t256)),X14.*t422+X17.*t423+X20.*t424+X23.*t425,-X14.*t414-X17.*t416-X20.*t418-X23.*t420,t296,m0+mi1.*t7+mi2.*t10+mi3.*t13+mi4.*t16,t232];
mt2 = [-X15.*(-t302+t350+X16.*(t60-t238))-X18.*(-t303+t351+X19.*(t66-t244))-X21.*(-t304+t352+X22.*(t72-t250))-X24.*(-t305+t353+X25.*(t78-t256)),-X15.*t422-X18.*t423-X21.*t424-X24.*t425,X15.*t414+X18.*t416+X21.*t418+X24.*t420,t297,t232,m0+mi1.*t8+mi2.*t11+mi3.*t14+mi4.*t17,-X16.*(-t302+t350+X16.*(t60-t238))-X19.*(-t303+t351+X19.*(t66-t244))-X22.*(-t304+t352+X22.*(t72-t250))-X25.*(-t305+t353+X25.*(t78-t256))];
mt3 = [-X16.*t422-X19.*t423-X22.*t424-X25.*t425,X16.*t414+X19.*t416+X22.*t418+X25.*t420,-rho1_2.*t378+rho1_3.*t380-rho2_2.*t387+rho2_3.*t389-rho3_2.*t396+rho3_3.*t398-rho4_2.*t405+rho4_3.*t407,rho1_2.*t383-rho1_3.*t382+rho2_2.*t392-rho2_3.*t391+rho3_2.*t401-rho3_3.*t400+rho4_2.*t410-rho4_3.*t409,rho1_2.*t385-rho1_3.*t384+rho2_2.*t394-rho2_3.*t393+rho3_2.*t403-rho3_3.*t402+rho4_2.*t412-rho4_3.*t411,et1+et2,-rho1_2.*t594+rho1_3.*t596-rho2_2.*t597+rho2_3.*t599-rho3_2.*t600+rho3_3.*t602-rho4_2.*t603+rho4_3.*t605];
mt4 = [rho1_2.*t570-rho1_3.*t572+rho2_2.*t576-rho2_3.*t578+rho3_2.*t582-rho3_3.*t584+rho4_2.*t588-rho4_3.*t590,rho1_1.*t378+rho1_3.*t379+rho2_1.*t387+rho2_3.*t388+rho3_1.*t396+rho3_3.*t397+rho4_1.*t405+rho4_3.*t406,-rho1_1.*t383-rho1_3.*t381-rho2_1.*t392-rho2_3.*t390-rho3_1.*t401-rho3_3.*t399-rho4_1.*t410-rho4_3.*t408,-rho1_1.*t385-rho1_3.*t386-rho2_1.*t394-rho2_3.*t395-rho3_1.*t403-rho3_3.*t404-rho4_1.*t412-rho4_3.*t413,et3+et4,j02+rho1_1.*t594+rho1_3.*t595+rho2_1.*t597+rho2_3.*t598+rho3_1.*t600+rho3_3.*t601+rho4_1.*t603+rho4_3.*t604];
mt5 = [-rho1_1.*t570-rho1_3.*t571-rho2_1.*t576-rho2_3.*t577-rho3_1.*t582-rho3_3.*t583-rho4_1.*t588-rho4_3.*t589,-rho1_1.*t380-rho1_2.*t379-rho2_1.*t389-rho2_2.*t388-rho3_1.*t398-rho3_2.*t397-rho4_1.*t407-rho4_2.*t406,rho1_1.*t382+rho1_2.*t381+rho2_1.*t391+rho2_2.*t390+rho3_1.*t400+rho3_2.*t399+rho4_1.*t409+rho4_2.*t408,rho1_1.*t384+rho1_2.*t386+rho2_1.*t393+rho2_2.*t395+rho3_1.*t402+rho3_2.*t404+rho4_1.*t411+rho4_2.*t413,et5+et6,-rho1_1.*t596-rho1_2.*t595-rho2_1.*t599-rho2_2.*t598-rho3_1.*t602-rho3_2.*t601-rho4_1.*t605-rho4_2.*t604];
mt6 = [j03+rho1_1.*t572+rho1_2.*t571+rho2_1.*t578+rho2_2.*t577+rho3_1.*t584+rho3_2.*t583+rho4_1.*t590+rho4_2.*t589];
A = reshape([mt1,mt2,mt3,mt4,mt5,mt6],6,6);
end
