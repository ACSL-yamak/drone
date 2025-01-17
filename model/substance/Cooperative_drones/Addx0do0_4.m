function A = Addx0do0_4(in1,in2,in3)
%Addx0do0_4
%    A = Addx0do0_4(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    09-Jun-2023 17:51:25

j01 = in3(:,3);
j02 = in3(:,4);
j03 = in3(:,5);
m0 = in3(:,2);
mi1 = in3(:,22);
mi2 = in3(:,23);
mi3 = in3(:,24);
mi4 = in3(:,25);
qi1_1 = in1(14,:);
qi1_2 = in1(15,:);
qi1_3 = in1(16,:);
qi2_1 = in1(17,:);
qi2_2 = in1(18,:);
qi2_3 = in1(19,:);
qi3_1 = in1(20,:);
qi3_2 = in1(21,:);
qi3_3 = in1(22,:);
qi4_1 = in1(23,:);
qi4_2 = in1(24,:);
qi4_3 = in1(25,:);
r01 = in1(4,:);
r02 = in1(5,:);
r03 = in1(6,:);
r04 = in1(7,:);
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
t2 = qi1_1.^2;
t3 = qi1_2.^2;
t4 = qi1_3.^2;
t5 = qi2_1.^2;
t6 = qi2_2.^2;
t7 = qi2_3.^2;
t8 = qi3_1.^2;
t9 = qi3_2.^2;
t10 = qi3_3.^2;
t11 = qi4_1.^2;
t12 = qi4_2.^2;
t13 = qi4_3.^2;
t14 = r01.^2;
t15 = r02.^2;
t16 = r03.^2;
t17 = r04.^2;
t18 = r01.*r02.*2.0;
t19 = r01.*r03.*2.0;
t20 = r01.*r04.*2.0;
t21 = r02.*r03.*2.0;
t22 = r02.*r04.*2.0;
t23 = r03.*r04.*2.0;
t24 = mi1.*qi1_1.*qi1_2;
t25 = mi1.*qi1_1.*qi1_3;
t26 = mi1.*qi1_2.*qi1_3;
t27 = mi2.*qi2_1.*qi2_2;
t28 = mi2.*qi2_1.*qi2_3;
t29 = mi2.*qi2_2.*qi2_3;
t30 = mi3.*qi3_1.*qi3_2;
t31 = mi3.*qi3_1.*qi3_3;
t32 = mi3.*qi3_2.*qi3_3;
t33 = mi4.*qi4_1.*qi4_2;
t34 = mi4.*qi4_1.*qi4_3;
t35 = mi4.*qi4_2.*qi4_3;
t36 = -t21;
t37 = -t22;
t38 = -t23;
t39 = -t15;
t40 = -t16;
t41 = -t17;
t42 = t18+t23;
t43 = t19+t22;
t44 = t20+t21;
t192 = t24+t27+t30+t33;
t193 = t25+t28+t31+t34;
t194 = t26+t29+t32+t35;
t45 = t18+t38;
t46 = t19+t37;
t47 = t20+t36;
t48 = mi1.*rho1_1.*t42;
t49 = mi1.*rho1_1.*t43;
t50 = mi1.*rho1_2.*t43;
t51 = mi1.*rho1_2.*t44;
t52 = mi1.*rho1_3.*t42;
t53 = mi1.*rho1_3.*t44;
t54 = mi2.*rho2_1.*t42;
t55 = mi2.*rho2_1.*t43;
t56 = mi2.*rho2_2.*t43;
t57 = mi2.*rho2_2.*t44;
t58 = mi2.*rho2_3.*t42;
t59 = mi2.*rho2_3.*t44;
t60 = mi3.*rho3_1.*t42;
t61 = mi3.*rho3_1.*t43;
t62 = mi3.*rho3_2.*t43;
t63 = mi3.*rho3_2.*t44;
t64 = mi3.*rho3_3.*t42;
t65 = mi3.*rho3_3.*t44;
t66 = mi4.*rho4_1.*t42;
t67 = mi4.*rho4_1.*t43;
t68 = mi4.*rho4_2.*t43;
t69 = mi4.*rho4_2.*t44;
t70 = mi4.*rho4_3.*t42;
t71 = mi4.*rho4_3.*t44;
t96 = t24.*t43;
t97 = t24.*t44;
t98 = t25.*t42;
t99 = t25.*t43;
t100 = t26.*t42;
t101 = t26.*t44;
t102 = t27.*t43;
t103 = t27.*t44;
t104 = t28.*t42;
t105 = t28.*t43;
t106 = t29.*t42;
t107 = t29.*t44;
t108 = t30.*t43;
t109 = t30.*t44;
t110 = t31.*t42;
t111 = t31.*t43;
t112 = t32.*t42;
t113 = t32.*t44;
t114 = t33.*t43;
t115 = t33.*t44;
t116 = t34.*t42;
t117 = t34.*t43;
t118 = t35.*t42;
t119 = t35.*t44;
t120 = mi1.*t2.*t43;
t121 = mi1.*t3.*t44;
t122 = mi1.*t4.*t42;
t123 = mi2.*t5.*t43;
t124 = mi2.*t6.*t44;
t125 = mi2.*t7.*t42;
t126 = mi3.*t8.*t43;
t127 = mi3.*t9.*t44;
t128 = mi3.*t10.*t42;
t129 = mi4.*t11.*t43;
t130 = mi4.*t12.*t44;
t131 = mi4.*t13.*t42;
t207 = t14+t17+t39+t40;
t208 = t14+t16+t39+t41;
t209 = t14+t15+t40+t41;
t72 = mi1.*rho1_1.*t45;
t73 = mi1.*rho1_1.*t47;
t74 = mi1.*rho1_2.*t45;
t75 = mi1.*rho1_2.*t46;
t76 = mi1.*rho1_3.*t46;
t77 = mi1.*rho1_3.*t47;
t78 = mi2.*rho2_1.*t45;
t79 = mi2.*rho2_1.*t47;
t80 = mi2.*rho2_2.*t45;
t81 = mi2.*rho2_2.*t46;
t82 = mi2.*rho2_3.*t46;
t83 = mi2.*rho2_3.*t47;
t84 = mi3.*rho3_1.*t45;
t85 = mi3.*rho3_1.*t47;
t86 = mi3.*rho3_2.*t45;
t87 = mi3.*rho3_2.*t46;
t88 = mi3.*rho3_3.*t46;
t89 = mi3.*rho3_3.*t47;
t90 = mi4.*rho4_1.*t45;
t91 = mi4.*rho4_1.*t47;
t92 = mi4.*rho4_2.*t45;
t93 = mi4.*rho4_2.*t46;
t94 = mi4.*rho4_3.*t46;
t95 = mi4.*rho4_3.*t47;
t132 = t24.*t45;
t133 = t24.*t47;
t134 = t25.*t46;
t135 = t25.*t47;
t136 = t26.*t45;
t137 = t26.*t46;
t138 = t27.*t45;
t139 = t27.*t47;
t140 = t28.*t46;
t141 = t28.*t47;
t142 = t29.*t45;
t143 = t29.*t46;
t144 = t30.*t45;
t145 = t30.*t47;
t146 = t31.*t46;
t147 = t31.*t47;
t148 = t32.*t45;
t149 = t32.*t46;
t150 = t33.*t45;
t151 = t33.*t47;
t152 = t34.*t46;
t153 = t34.*t47;
t154 = t35.*t45;
t155 = t35.*t46;
t156 = mi1.*t2.*t47;
t157 = mi1.*t3.*t45;
t158 = mi1.*t4.*t46;
t159 = mi2.*t5.*t47;
t160 = mi2.*t6.*t45;
t161 = mi2.*t7.*t46;
t162 = mi3.*t8.*t47;
t163 = mi3.*t9.*t45;
t164 = mi3.*t10.*t46;
t165 = mi4.*t11.*t47;
t166 = mi4.*t12.*t45;
t167 = mi4.*t13.*t46;
t210 = mi1.*rho1_1.*t207;
t211 = mi1.*rho1_1.*t208;
t212 = mi1.*rho1_2.*t207;
t213 = mi1.*rho1_2.*t209;
t214 = mi1.*rho1_3.*t208;
t215 = mi1.*rho1_3.*t209;
t216 = mi2.*rho2_1.*t207;
t217 = mi2.*rho2_1.*t208;
t218 = mi2.*rho2_2.*t207;
t219 = mi2.*rho2_2.*t209;
t220 = mi2.*rho2_3.*t208;
t221 = mi2.*rho2_3.*t209;
t222 = mi3.*rho3_1.*t207;
t223 = mi3.*rho3_1.*t208;
t224 = mi3.*rho3_2.*t207;
t225 = mi3.*rho3_2.*t209;
t226 = mi3.*rho3_3.*t208;
t227 = mi3.*rho3_3.*t209;
t228 = mi4.*rho4_1.*t207;
t229 = mi4.*rho4_1.*t208;
t230 = mi4.*rho4_2.*t207;
t231 = mi4.*rho4_2.*t209;
t232 = mi4.*rho4_3.*t208;
t233 = mi4.*rho4_3.*t209;
t234 = t24.*t208;
t235 = t24.*t209;
t236 = t25.*t207;
t237 = t25.*t209;
t238 = t26.*t207;
t239 = t26.*t208;
t240 = t27.*t208;
t241 = t27.*t209;
t242 = t28.*t207;
t243 = t28.*t209;
t244 = t29.*t207;
t245 = t29.*t208;
t246 = t30.*t208;
t247 = t30.*t209;
t248 = t31.*t207;
t249 = t31.*t209;
t250 = t32.*t207;
t251 = t32.*t208;
t252 = t33.*t208;
t253 = t33.*t209;
t254 = t34.*t207;
t255 = t34.*t209;
t256 = t35.*t207;
t257 = t35.*t208;
t258 = mi1.*t2.*t209;
t259 = mi1.*t3.*t208;
t260 = mi1.*t4.*t207;
t261 = mi2.*t5.*t209;
t262 = mi2.*t6.*t208;
t263 = mi2.*t7.*t207;
t264 = mi3.*t8.*t209;
t265 = mi3.*t9.*t208;
t266 = mi3.*t10.*t207;
t267 = mi4.*t11.*t209;
t268 = mi4.*t12.*t208;
t269 = mi4.*t13.*t207;
t168 = -t132;
t169 = -t133;
t170 = -t134;
t171 = -t135;
t172 = -t136;
t173 = -t137;
t174 = -t138;
t175 = -t139;
t176 = -t140;
t177 = -t141;
t178 = -t142;
t179 = -t143;
t180 = -t144;
t181 = -t145;
t182 = -t146;
t183 = -t147;
t184 = -t148;
t185 = -t149;
t186 = -t150;
t187 = -t151;
t188 = -t152;
t189 = -t153;
t190 = -t154;
t191 = -t155;
t195 = -t156;
t196 = -t157;
t197 = -t158;
t198 = -t159;
t199 = -t160;
t200 = -t161;
t201 = -t162;
t202 = -t163;
t203 = -t164;
t204 = -t165;
t205 = -t166;
t206 = -t167;
t270 = -t211;
t271 = -t212;
t272 = -t215;
t273 = -t217;
t274 = -t218;
t275 = -t221;
t276 = -t223;
t277 = -t224;
t278 = -t227;
t279 = -t229;
t280 = -t230;
t281 = -t233;
t282 = t48+t75;
t283 = t53+t72;
t284 = t50+t77;
t285 = t54+t81;
t286 = t59+t78;
t287 = t56+t83;
t288 = t60+t87;
t289 = t65+t84;
t290 = t62+t89;
t291 = t66+t93;
t292 = t71+t90;
t293 = t68+t95;
t318 = t73+t213;
t319 = t76+t210;
t320 = t74+t214;
t321 = t79+t219;
t322 = t82+t216;
t323 = t80+t220;
t324 = t85+t225;
t325 = t88+t222;
t326 = t86+t226;
t327 = t91+t231;
t328 = t94+t228;
t329 = t92+t232;
t294 = qi1_1.*t284;
t295 = qi1_2.*t283;
t296 = qi1_3.*t282;
t297 = qi2_1.*t287;
t298 = qi2_2.*t286;
t299 = qi2_3.*t285;
t300 = qi3_1.*t290;
t301 = qi3_2.*t289;
t302 = qi3_3.*t288;
t303 = qi4_1.*t293;
t304 = qi4_2.*t292;
t305 = qi4_3.*t291;
t330 = t51+t270;
t331 = t49+t272;
t332 = t52+t271;
t333 = t57+t273;
t334 = t55+t275;
t335 = t58+t274;
t336 = t63+t276;
t337 = t61+t278;
t338 = t64+t277;
t339 = t69+t279;
t340 = t67+t281;
t341 = t70+t280;
t342 = qi1_1.*t318;
t343 = qi1_2.*t320;
t344 = qi1_3.*t319;
t345 = qi2_1.*t321;
t346 = qi2_2.*t323;
t347 = qi2_3.*t322;
t348 = qi3_1.*t324;
t349 = qi3_2.*t326;
t350 = qi3_3.*t325;
t351 = qi4_1.*t327;
t352 = qi4_2.*t329;
t353 = qi4_3.*t328;
t366 = t97+t170+t258;
t367 = t120+t168+t236;
t368 = t98+t195+t234;
t369 = t100+t169+t259;
t370 = t121+t173+t235;
t371 = t96+t196+t238;
t372 = t99+t172+t260;
t373 = t122+t171+t239;
t374 = t101+t197+t237;
t375 = t103+t176+t261;
t376 = t123+t174+t242;
t377 = t104+t198+t240;
t378 = t106+t175+t262;
t379 = t124+t179+t241;
t380 = t102+t199+t244;
t381 = t105+t178+t263;
t382 = t125+t177+t245;
t383 = t107+t200+t243;
t384 = t109+t182+t264;
t385 = t126+t180+t248;
t386 = t110+t201+t246;
t387 = t112+t181+t265;
t388 = t127+t185+t247;
t389 = t108+t202+t250;
t390 = t111+t184+t266;
t391 = t128+t183+t251;
t392 = t113+t203+t249;
t393 = t115+t188+t267;
t394 = t129+t186+t254;
t395 = t116+t204+t252;
t396 = t118+t187+t268;
t397 = t130+t191+t253;
t398 = t114+t205+t256;
t399 = t117+t190+t269;
t400 = t131+t189+t257;
t401 = t119+t206+t255;
t306 = -t294;
t307 = -t295;
t308 = -t296;
t309 = -t297;
t310 = -t298;
t311 = -t299;
t312 = -t300;
t313 = -t301;
t314 = -t302;
t315 = -t303;
t316 = -t304;
t317 = -t305;
t354 = qi1_1.*t331;
t355 = qi1_2.*t330;
t356 = qi1_3.*t332;
t357 = qi2_1.*t334;
t358 = qi2_2.*t333;
t359 = qi2_3.*t335;
t360 = qi3_1.*t337;
t361 = qi3_2.*t336;
t362 = qi3_3.*t338;
t363 = qi4_1.*t340;
t364 = qi4_2.*t339;
t365 = qi4_3.*t341;
t402 = t308+t342+t355;
t403 = t307+t344+t354;
t404 = t306+t343+t356;
t405 = t311+t345+t358;
t406 = t310+t347+t357;
t407 = t309+t346+t359;
t408 = t314+t348+t361;
t409 = t313+t350+t360;
t410 = t312+t349+t362;
t411 = t317+t351+t364;
t412 = t316+t353+t363;
t413 = t315+t352+t365;
t414 = qi1_1.*t43.*t402;
t415 = qi1_2.*t44.*t402;
t416 = qi1_3.*t42.*t402;
t417 = qi1_1.*t43.*t403;
t418 = qi1_2.*t44.*t403;
t419 = qi1_3.*t42.*t403;
t420 = qi1_1.*t43.*t404;
t421 = qi1_2.*t44.*t404;
t422 = qi1_3.*t42.*t404;
t423 = qi2_1.*t43.*t405;
t424 = qi2_2.*t44.*t405;
t425 = qi2_3.*t42.*t405;
t426 = qi2_1.*t43.*t406;
t427 = qi2_2.*t44.*t406;
t428 = qi2_3.*t42.*t406;
t429 = qi2_1.*t43.*t407;
t430 = qi2_2.*t44.*t407;
t431 = qi2_3.*t42.*t407;
t432 = qi3_1.*t43.*t408;
t433 = qi3_2.*t44.*t408;
t434 = qi3_3.*t42.*t408;
t435 = qi3_1.*t43.*t409;
t436 = qi3_2.*t44.*t409;
t437 = qi3_3.*t42.*t409;
t438 = qi3_1.*t43.*t410;
t439 = qi3_2.*t44.*t410;
t440 = qi3_3.*t42.*t410;
t441 = qi4_1.*t43.*t411;
t442 = qi4_2.*t44.*t411;
t443 = qi4_3.*t42.*t411;
t444 = qi4_1.*t43.*t412;
t445 = qi4_2.*t44.*t412;
t446 = qi4_3.*t42.*t412;
t447 = qi4_1.*t43.*t413;
t448 = qi4_2.*t44.*t413;
t449 = qi4_3.*t42.*t413;
t450 = qi1_1.*t47.*t402;
t451 = qi1_2.*t45.*t402;
t452 = qi1_3.*t46.*t402;
t453 = qi1_1.*t47.*t403;
t454 = qi1_2.*t45.*t403;
t455 = qi1_3.*t46.*t403;
t456 = qi1_1.*t47.*t404;
t457 = qi1_2.*t45.*t404;
t458 = qi1_3.*t46.*t404;
t459 = qi2_1.*t47.*t405;
t460 = qi2_2.*t45.*t405;
t461 = qi2_3.*t46.*t405;
t462 = qi2_1.*t47.*t406;
t463 = qi2_2.*t45.*t406;
t464 = qi2_3.*t46.*t406;
t465 = qi2_1.*t47.*t407;
A = ft_1({j01,j02,j03,m0,mi1,mi2,mi3,mi4,qi1_1,qi1_2,qi1_3,qi2_1,qi2_2,qi2_3,qi3_1,qi3_2,qi3_3,qi4_1,qi4_2,qi4_3,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,rho4_1,rho4_2,rho4_3,t10,t11,t12,t13,t192,t193,t194,t2,t207,t208,t209,t3,t366,t367,t368,t369,t370,t371,t372,t373,t374,t375,t376,t377,t378,t379,t380,t381,t382,t383,t384,t385,t386,t387,t388,t389,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t4,t400,t401,t402,t403,t404,t405,t406,t407,t408,t409,t410,t411,t412,t413,t414,t415,t416,t417,t418,t419,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t430,t431,t432,t433,t434,t435,t436,t437,t438,t439,t440,t441,t442,t443,t444,t445,t446,t447,t448,t449,t45,t450,t451,t452,t453,t454,t455,t456,t457,t458,t459,t46,t460,t461,t462,t463,t464,t465,t47,t5,t6,t7,t8,t9});
end
function A = ft_1(ct)
j01 = ct{1};
j02 = ct{2};
j03 = ct{3};
m0 = ct{4};
mi1 = ct{5};
mi2 = ct{6};
mi3 = ct{7};
mi4 = ct{8};
qi1_1 = ct{9};
qi1_2 = ct{10};
qi1_3 = ct{11};
qi2_1 = ct{12};
qi2_2 = ct{13};
qi2_3 = ct{14};
qi3_1 = ct{15};
qi3_2 = ct{16};
qi3_3 = ct{17};
qi4_1 = ct{18};
qi4_2 = ct{19};
qi4_3 = ct{20};
rho1_1 = ct{21};
rho1_2 = ct{22};
rho1_3 = ct{23};
rho2_1 = ct{24};
rho2_2 = ct{25};
rho2_3 = ct{26};
rho3_1 = ct{27};
rho3_2 = ct{28};
rho3_3 = ct{29};
rho4_1 = ct{30};
rho4_2 = ct{31};
rho4_3 = ct{32};
t10 = ct{33};
t11 = ct{34};
t12 = ct{35};
t13 = ct{36};
t192 = ct{37};
t193 = ct{38};
t194 = ct{39};
t2 = ct{40};
t207 = ct{41};
t208 = ct{42};
t209 = ct{43};
t3 = ct{44};
t366 = ct{45};
t367 = ct{46};
t368 = ct{47};
t369 = ct{48};
t370 = ct{49};
t371 = ct{50};
t372 = ct{51};
t373 = ct{52};
t374 = ct{53};
t375 = ct{54};
t376 = ct{55};
t377 = ct{56};
t378 = ct{57};
t379 = ct{58};
t380 = ct{59};
t381 = ct{60};
t382 = ct{61};
t383 = ct{62};
t384 = ct{63};
t385 = ct{64};
t386 = ct{65};
t387 = ct{66};
t388 = ct{67};
t389 = ct{68};
t390 = ct{69};
t391 = ct{70};
t392 = ct{71};
t393 = ct{72};
t394 = ct{73};
t395 = ct{74};
t396 = ct{75};
t397 = ct{76};
t398 = ct{77};
t399 = ct{78};
t4 = ct{79};
t400 = ct{80};
t401 = ct{81};
t402 = ct{82};
t403 = ct{83};
t404 = ct{84};
t405 = ct{85};
t406 = ct{86};
t407 = ct{87};
t408 = ct{88};
t409 = ct{89};
t410 = ct{90};
t411 = ct{91};
t412 = ct{92};
t413 = ct{93};
t414 = ct{94};
t415 = ct{95};
t416 = ct{96};
t417 = ct{97};
t418 = ct{98};
t419 = ct{99};
t420 = ct{100};
t421 = ct{101};
t422 = ct{102};
t423 = ct{103};
t424 = ct{104};
t425 = ct{105};
t426 = ct{106};
t427 = ct{107};
t428 = ct{108};
t429 = ct{109};
t430 = ct{110};
t431 = ct{111};
t432 = ct{112};
t433 = ct{113};
t434 = ct{114};
t435 = ct{115};
t436 = ct{116};
t437 = ct{117};
t438 = ct{118};
t439 = ct{119};
t440 = ct{120};
t441 = ct{121};
t442 = ct{122};
t443 = ct{123};
t444 = ct{124};
t445 = ct{125};
t446 = ct{126};
t447 = ct{127};
t448 = ct{128};
t449 = ct{129};
t45 = ct{130};
t450 = ct{131};
t451 = ct{132};
t452 = ct{133};
t453 = ct{134};
t454 = ct{135};
t455 = ct{136};
t456 = ct{137};
t457 = ct{138};
t458 = ct{139};
t459 = ct{140};
t46 = ct{141};
t460 = ct{142};
t461 = ct{143};
t462 = ct{144};
t463 = ct{145};
t464 = ct{146};
t465 = ct{147};
t47 = ct{148};
t5 = ct{149};
t6 = ct{150};
t7 = ct{151};
t8 = ct{152};
t9 = ct{153};
t466 = qi2_2.*t45.*t407;
t467 = qi2_3.*t46.*t407;
t468 = qi3_1.*t47.*t408;
t469 = qi3_2.*t45.*t408;
t470 = qi3_3.*t46.*t408;
t471 = qi3_1.*t47.*t409;
t472 = qi3_2.*t45.*t409;
t473 = qi3_3.*t46.*t409;
t474 = qi3_1.*t47.*t410;
t475 = qi3_2.*t45.*t410;
t476 = qi3_3.*t46.*t410;
t477 = qi4_1.*t47.*t411;
t478 = qi4_2.*t45.*t411;
t479 = qi4_3.*t46.*t411;
t480 = qi4_1.*t47.*t412;
t481 = qi4_2.*t45.*t412;
t482 = qi4_3.*t46.*t412;
t483 = qi4_1.*t47.*t413;
t484 = qi4_2.*t45.*t413;
t485 = qi4_3.*t46.*t413;
t522 = qi1_1.*t209.*t402;
t523 = qi1_2.*t208.*t402;
t524 = qi1_3.*t207.*t402;
t525 = qi1_1.*t209.*t403;
t526 = qi1_2.*t208.*t403;
t527 = qi1_3.*t207.*t403;
t528 = qi1_1.*t209.*t404;
t529 = qi1_2.*t208.*t404;
t530 = qi1_3.*t207.*t404;
t531 = qi2_1.*t209.*t405;
t532 = qi2_2.*t208.*t405;
t533 = qi2_3.*t207.*t405;
t534 = qi2_1.*t209.*t406;
t535 = qi2_2.*t208.*t406;
t536 = qi2_3.*t207.*t406;
t537 = qi2_1.*t209.*t407;
t538 = qi2_2.*t208.*t407;
t539 = qi2_3.*t207.*t407;
t540 = qi3_1.*t209.*t408;
t541 = qi3_2.*t208.*t408;
t542 = qi3_3.*t207.*t408;
t543 = qi3_1.*t209.*t409;
t544 = qi3_2.*t208.*t409;
t545 = qi3_3.*t207.*t409;
t546 = qi3_1.*t209.*t410;
t547 = qi3_2.*t208.*t410;
t548 = qi3_3.*t207.*t410;
t549 = qi4_1.*t209.*t411;
t550 = qi4_2.*t208.*t411;
t551 = qi4_3.*t207.*t411;
t552 = qi4_1.*t209.*t412;
t553 = qi4_2.*t208.*t412;
t554 = qi4_3.*t207.*t412;
t555 = qi4_1.*t209.*t413;
t556 = qi4_2.*t208.*t413;
t557 = qi4_3.*t207.*t413;
t486 = -t450;
t487 = -t451;
t488 = -t452;
t489 = -t453;
t490 = -t454;
t491 = -t455;
t492 = -t456;
t493 = -t457;
t494 = -t458;
t495 = -t459;
t496 = -t460;
t497 = -t461;
t498 = -t462;
t499 = -t463;
t500 = -t464;
t501 = -t465;
t502 = -t466;
t503 = -t467;
t504 = -t468;
t505 = -t469;
t506 = -t470;
t507 = -t471;
t508 = -t472;
t509 = -t473;
t510 = -t474;
t511 = -t475;
t512 = -t476;
t513 = -t477;
t514 = -t478;
t515 = -t479;
t516 = -t480;
t517 = -t481;
t518 = -t482;
t519 = -t483;
t520 = -t484;
t521 = -t485;
t558 = t415+t488+t522;
t559 = t416+t486+t523;
t560 = t414+t487+t524;
t561 = t418+t491+t525;
t562 = t419+t489+t526;
t563 = t417+t490+t527;
t564 = t421+t494+t528;
t565 = t422+t492+t529;
t566 = t420+t493+t530;
t567 = t424+t497+t531;
t568 = t425+t495+t532;
t569 = t423+t496+t533;
t570 = t427+t500+t534;
t571 = t428+t498+t535;
t572 = t426+t499+t536;
t573 = t430+t503+t537;
t574 = t431+t501+t538;
t575 = t429+t502+t539;
t576 = t433+t506+t540;
t577 = t434+t504+t541;
t578 = t432+t505+t542;
t579 = t436+t509+t543;
t580 = t437+t507+t544;
t581 = t435+t508+t545;
t582 = t439+t512+t546;
t583 = t440+t510+t547;
t584 = t438+t511+t548;
t585 = t442+t515+t549;
t586 = t443+t513+t550;
t587 = t441+t514+t551;
t588 = t445+t518+t552;
t589 = t446+t516+t553;
t590 = t444+t517+t554;
t591 = t448+t521+t555;
t592 = t449+t519+t556;
t593 = t447+t520+t557;
mt1 = [m0+mi1.*t2+mi2.*t5+mi3.*t8+mi4.*t11,t192,t193,-qi1_1.*t404-qi2_1.*t407-qi3_1.*t410-qi4_1.*t413,-qi1_1.*t403-qi2_1.*t406-qi3_1.*t409-qi4_1.*t412,-qi1_1.*t402-qi2_1.*t405-qi3_1.*t408-qi4_1.*t411,t192,m0+mi1.*t3+mi2.*t6+mi3.*t9+mi4.*t12,t194,-qi1_2.*t404-qi2_2.*t407-qi3_2.*t410-qi4_2.*t413,-qi1_2.*t403-qi2_2.*t406-qi3_2.*t409-qi4_2.*t412,-qi1_2.*t402-qi2_2.*t405-qi3_2.*t408-qi4_2.*t411,t193,t194,m0+mi1.*t4+mi2.*t7+mi3.*t10+mi4.*t13,-qi1_3.*t404-qi2_3.*t407-qi3_3.*t410-qi4_3.*t413];
mt2 = [-qi1_3.*t403-qi2_3.*t406-qi3_3.*t409-qi4_3.*t412,-qi1_3.*t402-qi2_3.*t405-qi3_3.*t408-qi4_3.*t411,rho1_2.*t367-rho1_3.*t368+rho2_2.*t376-rho2_3.*t377+rho3_2.*t385-rho3_3.*t386+rho4_2.*t394-rho4_3.*t395,-rho1_3.*t369+rho1_2.*t371-rho2_3.*t378+rho2_2.*t380-rho3_3.*t387+rho3_2.*t389-rho4_3.*t396+rho4_2.*t398,rho1_2.*t372-rho1_3.*t373+rho2_2.*t381-rho2_3.*t382+rho3_2.*t390-rho3_3.*t391+rho4_2.*t399-rho4_3.*t400,j01-rho1_2.*t566+rho1_3.*t565-rho2_2.*t575+rho2_3.*t574-rho3_2.*t584+rho3_3.*t583-rho4_2.*t593+rho4_3.*t592];
mt3 = [-rho1_2.*t563+rho1_3.*t562-rho2_2.*t572+rho2_3.*t571-rho3_2.*t581+rho3_3.*t580-rho4_2.*t590+rho4_3.*t589,-rho1_2.*t560+rho1_3.*t559-rho2_2.*t569+rho2_3.*t568-rho3_2.*t578+rho3_3.*t577-rho4_2.*t587+rho4_3.*t586,-rho1_1.*t367+rho1_3.*t366-rho2_1.*t376+rho2_3.*t375-rho3_1.*t385+rho3_3.*t384-rho4_1.*t394+rho4_3.*t393,-rho1_1.*t371+rho1_3.*t370-rho2_1.*t380+rho2_3.*t379-rho3_1.*t389+rho3_3.*t388-rho4_1.*t398+rho4_3.*t397,-rho1_1.*t372+rho1_3.*t374-rho2_1.*t381+rho2_3.*t383-rho3_1.*t390+rho3_3.*t392-rho4_1.*t399+rho4_3.*t401,rho1_1.*t566-rho1_3.*t564+rho2_1.*t575-rho2_3.*t573+rho3_1.*t584-rho3_3.*t582+rho4_1.*t593-rho4_3.*t591];
mt4 = [j02+rho1_1.*t563-rho1_3.*t561+rho2_1.*t572-rho2_3.*t570+rho3_1.*t581-rho3_3.*t579+rho4_1.*t590-rho4_3.*t588,rho1_1.*t560-rho1_3.*t558+rho2_1.*t569-rho2_3.*t567+rho3_1.*t578-rho3_3.*t576+rho4_1.*t587-rho4_3.*t585,-rho1_2.*t366+rho1_1.*t368-rho2_2.*t375+rho2_1.*t377-rho3_2.*t384+rho3_1.*t386-rho4_2.*t393+rho4_1.*t395,rho1_1.*t369-rho1_2.*t370+rho2_1.*t378-rho2_2.*t379+rho3_1.*t387-rho3_2.*t388+rho4_1.*t396-rho4_2.*t397,rho1_1.*t373-rho1_2.*t374+rho2_1.*t382-rho2_2.*t383+rho3_1.*t391-rho3_2.*t392+rho4_1.*t400-rho4_2.*t401,-rho1_1.*t565+rho1_2.*t564-rho2_1.*t574+rho2_2.*t573-rho3_1.*t583+rho3_2.*t582-rho4_1.*t592+rho4_2.*t591];
mt5 = [-rho1_1.*t562+rho1_2.*t561-rho2_1.*t571+rho2_2.*t570-rho3_1.*t580+rho3_2.*t579-rho4_1.*t589+rho4_2.*t588,j03-rho1_1.*t559+rho1_2.*t558-rho2_1.*t568+rho2_2.*t567-rho3_1.*t577+rho3_2.*t576-rho4_1.*t586+rho4_2.*t585];
A = reshape([mt1,mt2,mt3,mt4,mt5],6,6);
end
