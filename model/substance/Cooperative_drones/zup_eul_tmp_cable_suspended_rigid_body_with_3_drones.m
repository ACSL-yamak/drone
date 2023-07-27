function dX = zup_eul_tmp_cable_suspended_rigid_body_with_3_drones(in1,in2,in3,in4)
%ZUP_EUL_TMP_CABLE_SUSPENDED_RIGID_BODY_WITH_3_DRONES
%    dX = ZUP_EUL_TMP_CABLE_SUSPENDED_RIGID_BODY_WITH_3_DRONES(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2023/06/26 22:07:48

Mi1_1 = in2(2,:);
Mi1_2 = in2(3,:);
Mi1_3 = in2(4,:);
Mi2_1 = in2(6,:);
Mi2_2 = in2(7,:);
Mi2_3 = in2(8,:);
Mi3_1 = in2(10,:);
Mi3_2 = in2(11,:);
Mi3_3 = in2(12,:);
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
ddX1 = in4(1,:);
ddX2 = in4(2,:);
ddX3 = in4(3,:);
ddX4 = in4(4,:);
ddX5 = in4(5,:);
ddX6 = in4(6,:);
fi1 = in2(1,:);
fi2 = in2(5,:);
fi3 = in2(9,:);
g = in3(:,1);
ji1_1 = in3(:,21);
ji1_2 = in3(:,22);
ji1_3 = in3(:,23);
ji2_1 = in3(:,24);
ji2_2 = in3(:,25);
ji2_3 = in3(:,26);
ji3_1 = in3(:,27);
ji3_2 = in3(:,28);
ji3_3 = in3(:,29);
li1 = in3(:,15);
li2 = in3(:,16);
li3 = in3(:,17);
mi1 = in3(:,18);
mi2 = in3(:,19);
mi3 = in3(:,20);
rho1_1 = in3(:,6);
rho1_2 = in3(:,7);
rho1_3 = in3(:,8);
rho2_1 = in3(:,9);
rho2_2 = in3(:,10);
rho2_3 = in3(:,11);
rho3_1 = in3(:,12);
rho3_2 = in3(:,13);
rho3_3 = in3(:,14);
t2 = cos(X4);
t3 = cos(X5);
t4 = cos(X31);
t5 = cos(X32);
t6 = cos(X34);
t7 = cos(X35);
t8 = cos(X37);
t9 = cos(X38);
t10 = sin(X4);
t11 = sin(X5);
t12 = sin(X31);
t13 = sin(X32);
t14 = sin(X34);
t15 = sin(X35);
t16 = sin(X37);
t17 = sin(X38);
t18 = X10.^2;
t19 = X11.^2;
t20 = X12.^2;
t21 = X13.^2;
t22 = X14.^2;
t23 = X15.^2;
t24 = X16.^2;
t25 = X17.^2;
t26 = X18.^2;
t27 = X19.^2;
t28 = X20.^2;
t29 = X21.^2;
t30 = -ddX1;
t31 = -ddX3;
t32 = 1.0./li1;
t33 = 1.0./li2;
t34 = 1.0./li3;
t35 = 1.0./mi1;
t36 = 1.0./mi2;
t37 = 1.0./mi3;
t42 = X4./2.0;
t43 = X5./2.0;
t44 = X6./2.0;
t45 = X31./2.0;
t46 = X32./2.0;
t47 = X33./2.0;
t48 = X34./2.0;
t49 = X35./2.0;
t50 = X36./2.0;
t51 = X37./2.0;
t52 = X38./2.0;
t53 = X39./2.0;
t38 = 1.0./t3;
t39 = 1.0./t5;
t40 = 1.0./t7;
t41 = 1.0./t9;
t54 = t21-1.0;
t55 = t22-1.0;
t56 = t23-1.0;
t57 = t24-1.0;
t58 = t25-1.0;
t59 = t26-1.0;
t60 = t27-1.0;
t61 = t28-1.0;
t62 = t29-1.0;
t63 = cos(t42);
t64 = cos(t43);
t65 = cos(t44);
t66 = cos(t45);
t67 = cos(t46);
t68 = cos(t47);
t69 = cos(t48);
t70 = cos(t49);
t71 = cos(t50);
t72 = cos(t51);
t73 = cos(t52);
t74 = cos(t53);
t75 = sin(t42);
t76 = sin(t43);
t77 = sin(t44);
t78 = sin(t45);
t79 = sin(t46);
t80 = sin(t47);
t81 = sin(t48);
t82 = sin(t49);
t83 = sin(t50);
t84 = sin(t51);
t85 = sin(t52);
t86 = sin(t53);
t87 = t18+t19;
t88 = t18+t20;
t89 = t19+t20;
t90 = t63.*t64.*t65;
t91 = t66.*t67.*t68;
t92 = t69.*t70.*t71;
t93 = t72.*t73.*t74;
t94 = t63.*t64.*t77;
t95 = t63.*t65.*t76;
t96 = t64.*t65.*t75;
t97 = t66.*t67.*t80;
t98 = t66.*t68.*t79;
t99 = t67.*t68.*t78;
t100 = t69.*t70.*t83;
t101 = t69.*t71.*t82;
t102 = t70.*t71.*t81;
t103 = t72.*t73.*t86;
t104 = t72.*t74.*t85;
t105 = t73.*t74.*t84;
t106 = t63.*t76.*t77;
t107 = t64.*t75.*t77;
t108 = t65.*t75.*t76;
t109 = t66.*t79.*t80;
t110 = t67.*t78.*t80;
t111 = t68.*t78.*t79;
t112 = t69.*t82.*t83;
t113 = t70.*t81.*t83;
t114 = t71.*t81.*t82;
t115 = t72.*t85.*t86;
t116 = t73.*t84.*t86;
t117 = t74.*t84.*t85;
t118 = t75.*t76.*t77;
t119 = t78.*t79.*t80;
t120 = t81.*t82.*t83;
t121 = t84.*t85.*t86;
t122 = -t106;
t123 = -t108;
t124 = -t109;
t125 = -t111;
t126 = -t112;
t127 = -t114;
t128 = -t115;
t129 = -t117;
t130 = t90+t118;
t131 = t95+t107;
t132 = t91+t119;
t133 = t98+t110;
t134 = t92+t120;
t135 = t101+t113;
t136 = t93+t121;
t137 = t104+t116;
t138 = t130.^2;
t139 = t131.^2;
t140 = t132.^2;
t141 = t133.^2;
t142 = t134.^2;
t143 = t135.^2;
t144 = t136.^2;
t145 = t137.^2;
t146 = t94+t123;
t147 = t96+t122;
t148 = t97+t125;
t149 = t99+t124;
t150 = t100+t127;
t151 = t102+t126;
t152 = t103+t129;
t153 = t105+t128;
t171 = t130.*t131.*2.0;
t172 = t132.*t133.*2.0;
t173 = t134.*t135.*2.0;
t174 = t136.*t137.*2.0;
t154 = t146.^2;
t155 = t147.^2;
t156 = t148.^2;
t157 = t149.^2;
t158 = t150.^2;
t159 = t151.^2;
t160 = t152.^2;
t161 = t153.^2;
t162 = -t139;
t163 = -t141;
t164 = -t143;
t165 = -t145;
t175 = t130.*t146.*2.0;
t176 = t130.*t147.*2.0;
t177 = t131.*t146.*2.0;
t178 = t131.*t147.*2.0;
t179 = t132.*t149.*2.0;
t180 = t133.*t148.*2.0;
t181 = t134.*t151.*2.0;
t182 = t135.*t150.*2.0;
t183 = t136.*t153.*2.0;
t184 = t137.*t152.*2.0;
t190 = t146.*t147.*2.0;
t191 = t148.*t149.*2.0;
t192 = t150.*t151.*2.0;
t193 = t152.*t153.*2.0;
t166 = -t154;
t167 = -t155;
t168 = -t157;
t169 = -t159;
t170 = -t161;
t185 = -t177;
t186 = -t178;
t187 = -t180;
t188 = -t182;
t189 = -t184;
t194 = -t190;
t195 = t171+t190;
t196 = t175+t178;
t197 = t176+t177;
t198 = t172+t191;
t199 = t173+t192;
t200 = t174+t193;
t201 = t171+t194;
t202 = t175+t186;
t203 = t176+t185;
t204 = t179+t187;
t205 = t181+t188;
t206 = t183+t189;
t207 = rho1_1.*t195;
t208 = rho1_1.*t197;
t209 = rho1_2.*t195;
t210 = rho1_2.*t196;
t211 = rho1_3.*t196;
t212 = rho1_3.*t197;
t213 = rho2_1.*t195;
t214 = rho2_1.*t197;
t215 = rho2_2.*t195;
t216 = rho2_2.*t196;
t217 = rho2_3.*t196;
t218 = rho2_3.*t197;
t219 = rho3_1.*t195;
t220 = rho3_1.*t197;
t221 = rho3_2.*t195;
t222 = rho3_2.*t196;
t223 = rho3_3.*t196;
t224 = rho3_3.*t197;
t243 = X10.*X11.*t196;
t244 = X10.*X11.*t197;
t245 = X10.*X12.*t195;
t246 = X10.*X12.*t196;
t247 = X11.*X12.*t195;
t248 = X11.*X12.*t197;
t249 = X13.*X14.*fi1.*t198;
t250 = X13.*X15.*fi1.*t198;
t251 = X16.*X17.*fi2.*t199;
t252 = X16.*X18.*fi2.*t199;
t253 = X19.*X20.*fi3.*t200;
t254 = X19.*X21.*fi3.*t200;
t291 = t87.*t195;
t292 = t88.*t197;
t293 = t89.*t196;
t294 = fi1.*t54.*t198;
t295 = fi2.*t57.*t199;
t296 = fi3.*t60.*t200;
t303 = t138+t139+t166+t167;
t304 = t138+t154+t162+t167;
t305 = t138+t155+t162+t166;
t306 = t140+t156+t163+t168;
t307 = t142+t158+t164+t169;
t308 = t144+t160+t165+t170;
t225 = rho1_1.*t202;
t226 = rho1_1.*t203;
t227 = rho1_2.*t201;
t228 = rho1_2.*t203;
t229 = rho1_3.*t201;
t230 = rho1_3.*t202;
t231 = rho2_1.*t202;
t232 = rho2_1.*t203;
t233 = rho2_2.*t201;
t234 = rho2_2.*t203;
t235 = rho2_3.*t201;
t236 = rho2_3.*t202;
t237 = rho3_1.*t202;
t238 = rho3_1.*t203;
t239 = rho3_2.*t201;
t240 = rho3_2.*t203;
t241 = rho3_3.*t201;
t242 = rho3_3.*t202;
t261 = X10.*X11.*t201;
t262 = X10.*X11.*t202;
t263 = X10.*X12.*t201;
t264 = X10.*X12.*t203;
t265 = X11.*X12.*t202;
t266 = X11.*X12.*t203;
t267 = X13.*X14.*fi1.*t204;
t268 = X14.*X15.*fi1.*t204;
t269 = X16.*X17.*fi2.*t205;
t270 = X17.*X18.*fi2.*t205;
t271 = X19.*X20.*fi3.*t206;
t272 = X20.*X21.*fi3.*t206;
t282 = -t243;
t283 = -t245;
t284 = -t248;
t297 = t87.*t203;
t298 = t88.*t202;
t299 = t89.*t201;
t300 = fi1.*t55.*t204;
t301 = fi2.*t58.*t205;
t302 = fi3.*t61.*t206;
t312 = rho1_1.*t303;
t313 = rho1_1.*t304;
t314 = rho1_2.*t304;
t315 = rho1_2.*t305;
t316 = rho1_3.*t303;
t317 = rho1_3.*t305;
t318 = rho2_1.*t303;
t319 = rho2_1.*t304;
t320 = rho2_2.*t304;
t321 = rho2_2.*t305;
t322 = rho2_3.*t303;
t323 = rho2_3.*t305;
t324 = rho3_1.*t303;
t325 = rho3_1.*t304;
t326 = rho3_2.*t304;
t327 = rho3_2.*t305;
t328 = rho3_3.*t303;
t329 = rho3_3.*t305;
t330 = X10.*X11.*t303;
t331 = X10.*X11.*t305;
t332 = X10.*X12.*t304;
t333 = X10.*X12.*t305;
t334 = X11.*X12.*t303;
t335 = X11.*X12.*t304;
t336 = X13.*X15.*fi1.*t306;
t337 = X14.*X15.*fi1.*t306;
t338 = X16.*X18.*fi2.*t307;
t339 = X17.*X18.*fi2.*t307;
t340 = X19.*X21.*fi3.*t308;
t341 = X20.*X21.*fi3.*t308;
t345 = fi1.*t56.*t306;
t346 = fi2.*t59.*t307;
t347 = fi3.*t62.*t308;
t348 = t87.*t304;
t349 = t88.*t303;
t350 = t89.*t305;
t274 = -t227;
t277 = -t233;
t280 = -t239;
t285 = -t267;
t286 = -t268;
t287 = -t269;
t288 = -t270;
t289 = -t271;
t290 = -t272;
t309 = -t300;
t310 = -t301;
t311 = -t302;
t342 = -t330;
t343 = -t333;
t344 = -t335;
t351 = t209+t230;
t352 = t215+t236;
t353 = t221+t242;
t363 = -ddX5.*(t211-t226);
t365 = -ddX5.*(t217-t232);
t367 = -ddX5.*(t223-t238);
t372 = t210+t312;
t373 = t207+t317;
t374 = t216+t318;
t375 = t213+t323;
t376 = t222+t324;
t377 = t219+t329;
t378 = t228+t316;
t379 = t234+t322;
t380 = t240+t328;
t405 = -ddX5.*(t229-t313);
t406 = -ddX6.*(t225-t315);
t408 = -ddX5.*(t235-t319);
t409 = -ddX6.*(t231-t321);
t411 = -ddX5.*(t241-t325);
t412 = -ddX6.*(t237-t327);
t414 = ddX4.*(t212-t314);
t415 = ddX4.*(t218-t320);
t416 = ddX4.*(t224-t326);
t417 = t246+t297+t334;
t418 = t247+t298+t331;
t419 = t244+t299+t332;
t432 = t263+t284+t348;
t433 = t266+t282+t349;
t434 = t262+t283+t350;
t354 = ddX4.*t351;
t355 = ddX4.*t352;
t356 = ddX4.*t353;
t357 = t208+t274;
t359 = t214+t277;
t361 = t220+t280;
t384 = ddX5.*t373;
t385 = ddX6.*t372;
t386 = ddX5.*t375;
t387 = ddX6.*t374;
t388 = ddX5.*t377;
t389 = ddX6.*t376;
t396 = ddX4.*t378;
t397 = ddX4.*t379;
t398 = ddX4.*t380;
t420 = rho1_1.*t419;
t421 = rho1_2.*t418;
t422 = rho1_3.*t417;
t423 = rho2_1.*t419;
t424 = rho2_2.*t418;
t425 = rho2_3.*t417;
t426 = rho3_1.*t419;
t427 = rho3_2.*t418;
t428 = rho3_3.*t417;
t429 = t265+t291+t343;
t430 = t261+t292+t344;
t431 = t264+t293+t342;
t435 = t285+t294+t336;
t436 = t249+t309+t337;
t437 = t250+t286+t345;
t438 = t287+t295+t338;
t439 = t251+t310+t339;
t440 = t252+t288+t346;
t441 = t289+t296+t340;
t442 = t253+t311+t341;
t443 = t254+t290+t347;
t445 = rho1_1.*t434;
t447 = rho1_2.*t433;
t449 = rho1_3.*t432;
t451 = rho2_1.*t434;
t453 = rho2_2.*t433;
t455 = rho2_3.*t432;
t457 = rho3_1.*t434;
t459 = rho3_2.*t433;
t461 = rho3_3.*t432;
dX = ft_1({Mi1_1,Mi1_2,Mi1_3,Mi2_1,Mi2_2,Mi2_3,Mi3_1,Mi3_2,Mi3_3,X10,X11,X12,X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,X26,X27,X28,X29,X30,X40,X41,X42,X43,X44,X45,X46,X47,X48,X7,X8,X9,ddX1,ddX2,ddX4,ddX5,ddX6,g,ji1_1,ji1_2,ji1_3,ji2_1,ji2_2,ji2_3,ji3_1,ji3_2,ji3_3,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,t10,t11,t12,t13,t14,t15,t16,t17,t2,t3,t30,t31,t32,t33,t34,t35,t354,t355,t356,t357,t359,t36,t361,t363,t365,t367,t37,t38,t384,t385,t386,t387,t388,t389,t39,t396,t397,t398,t4,t40,t405,t406,t408,t409,t41,t411,t412,t414,t415,t416,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t430,t431,t435,t436,t437,t438,t439,t440,t441,t442,t443,t445,t447,t449,t451,t453,t455,t457,t459,t461,t5,t6,t7,t8,t9});
end
function dX = ft_1(ct)
[Mi1_1,Mi1_2,Mi1_3,Mi2_1,Mi2_2,Mi2_3,Mi3_1,Mi3_2,Mi3_3,X10,X11,X12,X13,X14,X15,X16,X17,X18,X19,X20,X21,X22,X23,X24,X25,X26,X27,X28,X29,X30,X40,X41,X42,X43,X44,X45,X46,X47,X48,X7,X8,X9,ddX1,ddX2,ddX4,ddX5,ddX6,g,ji1_1,ji1_2,ji1_3,ji2_1,ji2_2,ji2_3,ji3_1,ji3_2,ji3_3,rho1_1,rho1_2,rho1_3,rho2_1,rho2_2,rho2_3,rho3_1,rho3_2,rho3_3,t10,t11,t12,t13,t14,t15,t16,t17,t2,t3,t30,t31,t32,t33,t34,t35,t354,t355,t356,t357,t359,t36,t361,t363,t365,t367,t37,t38,t384,t385,t386,t387,t388,t389,t39,t396,t397,t398,t4,t40,t405,t406,t408,t409,t41,t411,t412,t414,t415,t416,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t430,t431,t435,t436,t437,t438,t439,t440,t441,t442,t443,t445,t447,t449,t451,t453,t455,t457,t459,t461,t5,t6,t7,t8,t9] = ct{:};
t364 = ddX6.*t357;
t366 = ddX6.*t359;
t368 = ddX6.*t361;
t400 = -t384;
t402 = -t386;
t404 = -t388;
t407 = -t396;
t410 = -t397;
t413 = -t398;
t444 = rho1_1.*t431;
t446 = rho1_2.*t430;
t448 = rho1_3.*t429;
t450 = rho2_1.*t431;
t452 = rho2_2.*t430;
t454 = rho2_3.*t429;
t456 = rho3_1.*t431;
t458 = rho3_2.*t430;
t460 = rho3_3.*t429;
t462 = -t447;
t464 = -t453;
t466 = -t459;
t369 = -t364;
t370 = -t366;
t371 = -t368;
t463 = -t448;
t465 = -t454;
t467 = -t460;
t468 = ddX2+t363+t385+t407+t422+t444+t462;
t469 = ddX2+t365+t387+t410+t425+t450+t464;
t470 = ddX2+t367+t389+t413+t428+t456+t466;
t471 = t30+t354+t400+t406+t421+t445+t463;
t472 = t30+t355+t402+t409+t424+t451+t465;
t473 = t30+t356+t404+t412+t427+t457+t467;
t474 = g+t31+t369+t405+t414+t420+t446+t449;
t475 = g+t31+t370+t408+t415+t423+t452+t455;
t476 = g+t31+t371+t411+t416+t426+t458+t461;
mt1 = [X7;X8;X9;t38.*(X10.*t3+X12.*t2.*t11+X11.*t10.*t11);X11.*t2-X12.*t10;t38.*(X12.*t2+X11.*t10);ddX1;-ddX2;t31;ddX4;-ddX5;-ddX6;-X14.*X24+X15.*X23;X13.*X24-X15.*X22;-X13.*X23+X14.*X22;-X17.*X27+X18.*X26;X16.*X27-X18.*X25;-X16.*X26+X17.*X25;-X20.*X30+X21.*X29;X19.*X30-X21.*X28;-X19.*X29+X20.*X28;X15.*t32.*t468+X14.*t32.*t474+X14.*t32.*t35.*t437-X15.*t32.*t35.*t436;-X15.*t32.*t471-X13.*t32.*t474-X13.*t32.*t35.*t437+X15.*t32.*t35.*t435;-X13.*t32.*t468+X14.*t32.*t471+X13.*t32.*t35.*t436-X14.*t32.*t35.*t435];
mt2 = [X18.*t33.*t469+X17.*t33.*t475+X17.*t33.*t36.*t440-X18.*t33.*t36.*t439;-X18.*t33.*t472-X16.*t33.*t475-X16.*t33.*t36.*t440+X18.*t33.*t36.*t438;-X16.*t33.*t469+X17.*t33.*t472+X16.*t33.*t36.*t439-X17.*t33.*t36.*t438;X21.*t34.*t470+X20.*t34.*t476+X20.*t34.*t37.*t443-X21.*t34.*t37.*t442;-X21.*t34.*t473-X19.*t34.*t476-X19.*t34.*t37.*t443+X21.*t34.*t37.*t441;-X19.*t34.*t470+X20.*t34.*t473+X19.*t34.*t37.*t442-X20.*t34.*t37.*t441;t39.*(X40.*t5+X42.*t4.*t13+X41.*t12.*t13);X41.*t4-X42.*t12;t39.*(X42.*t4+X41.*t12);t40.*(X43.*t7+X45.*t6.*t15+X44.*t14.*t15);X44.*t6-X45.*t14];
mt3 = [t40.*(X45.*t6+X44.*t14);t41.*(X46.*t9+X48.*t8.*t17+X47.*t16.*t17);X47.*t8-X48.*t16;t41.*(X48.*t8+X47.*t16);(Mi1_1+X41.*X42.*ji1_2-X41.*X42.*ji1_3)./ji1_1;-(Mi1_2+X40.*X42.*ji1_1-X40.*X42.*ji1_3)./ji1_2;-(Mi1_3-X40.*X41.*ji1_1+X40.*X41.*ji1_2)./ji1_3;(Mi2_1+X44.*X45.*ji2_2-X44.*X45.*ji2_3)./ji2_1;-(Mi2_2+X43.*X45.*ji2_1-X43.*X45.*ji2_3)./ji2_2;-(Mi2_3-X43.*X44.*ji2_1+X43.*X44.*ji2_2)./ji2_3;(Mi3_1+X47.*X48.*ji3_2-X47.*X48.*ji3_3)./ji3_1;-(Mi3_2+X46.*X48.*ji3_1-X46.*X48.*ji3_3)./ji3_2];
mt4 = [-(Mi3_3-X46.*X47.*ji3_1+X46.*X47.*ji3_2)./ji3_3];
dX = [mt1;mt2;mt3;mt4];
end