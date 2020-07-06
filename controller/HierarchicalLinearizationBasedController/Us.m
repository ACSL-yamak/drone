function U2 = Us(in1,in2,in3,in4,in5)
%US
%    U2 = US(IN1,IN2,IN3,IN4,IN5)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    13-Mar-2020 20:01:15

V1 = in3(:,1);
V2 = in4(1,:);
V3 = in4(2,:);
V4 = in4(3,:);
dV1 = in3(:,2);
ddV1 = in3(:,3);
ddXd3 = in2(:,11);
ddXd4 = in2(:,12);
dddXd3 = in2(:,15);
ddddXd1 = in2(:,17);
ddddXd2 = in2(:,18);
ddddXd3 = in2(:,19);
gravity = in5(:,6);
jx = in5(:,3);
jy = in5(:,4);
jz = in5(:,5);
km1 = in5(:,7);
km2 = in5(:,8);
km3 = in5(:,9);
km4 = in5(:,10);
l = in5(:,2);
m = in5(:,1);
o1 = in1(11,:);
o2 = in1(12,:);
o3 = in1(13,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
q3 = in1(4,:);
t2 = dV1+dddXd3;
t3 = ddV1+ddddXd3;
t4 = q0.^2;
t5 = q0.^3;
t6 = q1.^2;
t8 = q1.^3;
t9 = q2.^2;
t11 = q2.^3;
t12 = q3.^2;
t14 = q3.^3;
t16 = q0.*q1.*2.0;
t17 = q0.*q2.*2.0;
t18 = q0.*q3.*2.0;
t19 = q1.*q2.*2.0;
t20 = q1.*q3.*2.0;
t21 = q2.*q3.*2.0;
t22 = V1+ddXd3+gravity;
t23 = -jy;
t24 = -jz;
t25 = 1.0./jx;
t26 = 1.0./jy;
t27 = 1.0./jz;
t29 = sqrt(2.0);
t81 = (o1.*q0)./2.0;
t82 = (o1.*q1)./2.0;
t83 = (o2.*q0)./2.0;
t84 = (o1.*q2)./2.0;
t85 = (o2.*q1)./2.0;
t86 = (o3.*q0)./2.0;
t87 = (o1.*q3)./2.0;
t88 = (o2.*q2)./2.0;
t89 = (o3.*q1)./2.0;
t90 = (o2.*q3)./2.0;
t91 = (o3.*q2)./2.0;
t92 = (o3.*q3)./2.0;
t96 = q0.*q1.*q2.*q3.*8.0;
t7 = t4.^2;
t10 = t6.^2;
t13 = t9.^2;
t15 = t12.^2;
t28 = -t21;
t30 = km1.*t4;
t31 = km1.*t6;
t32 = km2.*t4;
t33 = km1.*t9;
t34 = km2.*t6;
t35 = km3.*t4;
t36 = km1.*t12;
t37 = km2.*t9;
t38 = km3.*t6;
t39 = km4.*t4;
t40 = km2.*t12;
t41 = km3.*t9;
t42 = km4.*t6;
t43 = km3.*t12;
t44 = km4.*t9;
t45 = km4.*t12;
t78 = -t6;
t79 = -t9;
t80 = -t12;
t93 = jx+t23;
t94 = jx+t24;
t95 = jy+t24;
t97 = -t84;
t98 = -t89;
t99 = -t90;
t148 = t4.*t6.*2.0;
t149 = t4.*t9.*2.0;
t150 = t4.*t12.*2.0;
t151 = t6.*t9.*2.0;
t152 = t6.*t12.*2.0;
t153 = t9.*t12.*2.0;
t170 = jx.*km1.*q0.*t14.*2.0;
t171 = jx.*km1.*q1.*t11.*2.0;
t172 = jx.*km1.*q3.*t5.*2.0;
t173 = jx.*km1.*q2.*t8.*2.0;
t174 = jx.*km2.*q0.*t14.*2.0;
t175 = jx.*km2.*q1.*t11.*2.0;
t176 = jx.*km2.*q3.*t5.*2.0;
t177 = jx.*km2.*q2.*t8.*2.0;
t178 = jx.*km3.*q0.*t14.*2.0;
t179 = jx.*km3.*q1.*t11.*2.0;
t180 = jx.*km3.*q3.*t5.*2.0;
t181 = jx.*km3.*q2.*t8.*2.0;
t182 = jx.*km4.*q0.*t14.*2.0;
t183 = jx.*km4.*q1.*t11.*2.0;
t184 = jx.*km4.*q3.*t5.*2.0;
t185 = jx.*km4.*q2.*t8.*2.0;
t186 = jy.*km1.*q0.*t14.*2.0;
t187 = jy.*km1.*q1.*t11.*2.0;
t188 = jy.*km1.*q3.*t5.*2.0;
t189 = jy.*km1.*q2.*t8.*2.0;
t190 = jy.*km2.*q0.*t14.*2.0;
t191 = jy.*km2.*q1.*t11.*2.0;
t192 = jy.*km2.*q3.*t5.*2.0;
t193 = jy.*km2.*q2.*t8.*2.0;
t194 = jy.*km3.*q0.*t14.*2.0;
t195 = jy.*km3.*q1.*t11.*2.0;
t196 = jy.*km3.*q3.*t5.*2.0;
t197 = jy.*km3.*q2.*t8.*2.0;
t198 = jy.*km4.*q0.*t14.*2.0;
t199 = jy.*km4.*q1.*t11.*2.0;
t200 = jy.*km4.*q3.*t5.*2.0;
t201 = jy.*km4.*q2.*t8.*2.0;
t220 = t17+t20;
t221 = t18+t19;
t385 = jz.*l.*q0.*t8.*t29.*2.0;
t386 = jz.*l.*q1.*t5.*t29.*2.0;
t387 = jz.*l.*q2.*t14.*t29.*2.0;
t388 = jz.*l.*q3.*t11.*t29.*2.0;
t389 = jz.*l.*t9.*t16.*t29;
t390 = jz.*l.*t12.*t16.*t29;
t391 = jz.*l.*q0.*q2.*t6.*t29.*4.0;
t392 = jz.*l.*t4.*t21.*t29;
t393 = jz.*l.*q1.*q3.*t4.*t29.*4.0;
t394 = jz.*l.*t6.*t21.*t29;
t395 = jz.*l.*q0.*q2.*t12.*t29.*4.0;
t396 = jz.*l.*q1.*q3.*t9.*t29.*4.0;
t401 = jz.*l.*q0.*q1.*t9.*t29.*-2.0;
t402 = jz.*l.*q0.*q1.*t12.*t29.*-2.0;
t403 = jz.*l.*q2.*q3.*t4.*t29.*-2.0;
t404 = jz.*l.*q2.*q3.*t6.*t29.*-2.0;
t405 = t82+t88+t92;
t46 = jx.*km1.*t7;
t47 = jx.*km1.*t10;
t48 = jx.*km2.*t7;
t49 = jx.*km1.*t13;
t50 = jx.*km2.*t10;
t51 = jx.*km3.*t7;
t52 = jx.*km1.*t15;
t53 = jx.*km2.*t13;
t54 = jx.*km3.*t10;
t55 = jx.*km4.*t7;
t56 = jx.*km2.*t15;
t57 = jx.*km3.*t13;
t58 = jx.*km4.*t10;
t59 = jx.*km3.*t15;
t60 = jx.*km4.*t13;
t61 = jx.*km4.*t15;
t62 = jy.*km1.*t7;
t63 = jy.*km1.*t10;
t64 = jy.*km2.*t7;
t65 = jy.*km1.*t13;
t66 = jy.*km2.*t10;
t67 = jy.*km3.*t7;
t68 = jy.*km1.*t15;
t69 = jy.*km2.*t13;
t70 = jy.*km3.*t10;
t71 = jy.*km4.*t7;
t72 = jy.*km2.*t15;
t73 = jy.*km3.*t13;
t74 = jy.*km4.*t10;
t75 = jy.*km3.*t15;
t76 = jy.*km4.*t13;
t77 = jy.*km4.*t15;
t100 = V1.*km1.*l.*t7;
t101 = V1.*km1.*l.*t10;
t102 = V1.*km2.*l.*t7;
t103 = V1.*km1.*l.*t13;
t104 = V1.*km2.*l.*t10;
t105 = V1.*km3.*l.*t7;
t106 = V1.*km1.*l.*t15;
t107 = V1.*km2.*l.*t13;
t108 = V1.*km3.*l.*t10;
t109 = V1.*km4.*l.*t7;
t110 = V1.*km2.*l.*t15;
t111 = V1.*km3.*l.*t13;
t112 = V1.*km4.*l.*t10;
t113 = V1.*km3.*l.*t15;
t114 = V1.*km4.*l.*t13;
t115 = V1.*km4.*l.*t15;
t116 = ddXd3.*km1.*l.*t7;
t117 = ddXd3.*km1.*l.*t10;
t118 = ddXd3.*km2.*l.*t7;
t119 = ddXd3.*km1.*l.*t13;
t120 = ddXd3.*km2.*l.*t10;
t121 = ddXd3.*km3.*l.*t7;
t122 = ddXd3.*km1.*l.*t15;
t123 = ddXd3.*km2.*l.*t13;
t124 = ddXd3.*km3.*l.*t10;
t125 = ddXd3.*km4.*l.*t7;
t126 = ddXd3.*km2.*l.*t15;
t127 = ddXd3.*km3.*l.*t13;
t128 = ddXd3.*km4.*l.*t10;
t129 = ddXd3.*km3.*l.*t15;
t130 = ddXd3.*km4.*l.*t13;
t131 = ddXd3.*km4.*l.*t15;
t132 = gravity.*km1.*l.*t7;
t133 = gravity.*km1.*l.*t10;
t134 = gravity.*km2.*l.*t7;
t135 = gravity.*km1.*l.*t13;
t136 = gravity.*km2.*l.*t10;
t137 = gravity.*km3.*l.*t7;
t138 = gravity.*km1.*l.*t15;
t139 = gravity.*km2.*l.*t13;
t140 = gravity.*km3.*l.*t10;
t141 = gravity.*km4.*l.*t7;
t142 = gravity.*km2.*l.*t15;
t143 = gravity.*km3.*l.*t13;
t144 = gravity.*km4.*l.*t10;
t145 = gravity.*km3.*l.*t15;
t146 = gravity.*km4.*l.*t13;
t147 = gravity.*km4.*l.*t15;
t162 = km1.*t10.*t23;
t163 = km2.*t10.*t23;
t164 = km1.*t15.*t23;
t165 = km3.*t10.*t23;
t166 = km2.*t15.*t23;
t167 = km4.*t10.*t23;
t168 = km3.*t15.*t23;
t169 = km4.*t15.*t23;
t202 = -t149;
t203 = -t152;
t204 = -t170;
t205 = -t171;
t206 = -t172;
t207 = -t173;
t208 = -t178;
t209 = -t179;
t210 = -t180;
t211 = -t181;
t212 = -t187;
t213 = -t189;
t214 = -t191;
t215 = -t193;
t216 = -t194;
t217 = -t196;
t218 = -t198;
t219 = -t200;
t222 = jx.*t19.*t30;
t223 = jx.*t18.*t31;
t224 = jx.*t19.*t32;
t225 = jx.*t18.*t33;
t226 = jx.*t18.*t34;
t227 = jx.*t19.*t35;
t228 = jx.*t19.*t36;
t229 = jx.*t18.*t37;
t230 = jx.*t18.*t38;
t231 = jx.*t19.*t39;
t232 = jx.*t19.*t40;
t233 = jx.*t18.*t41;
t234 = jx.*t18.*t42;
t235 = jx.*t19.*t43;
t236 = jx.*t18.*t44;
t237 = jx.*t19.*t45;
t238 = jy.*t19.*t30;
t239 = jy.*t18.*t31;
t240 = jy.*t19.*t32;
t241 = jy.*t18.*t33;
t242 = jy.*t18.*t34;
t243 = jy.*t19.*t35;
t244 = jy.*t19.*t36;
t245 = jy.*t18.*t37;
t246 = jy.*t18.*t38;
t247 = jy.*t19.*t39;
t248 = jy.*t19.*t40;
t249 = jy.*t18.*t41;
t250 = jy.*t18.*t42;
t251 = jy.*t19.*t43;
t252 = jy.*t18.*t44;
t253 = jy.*t19.*t45;
t254 = jx.*t6.*t30.*2.0;
t255 = jx.*t6.*t32.*2.0;
t256 = jx.*t6.*t35.*2.0;
t257 = jx.*t6.*t39.*2.0;
t258 = jx.*t12.*t33.*2.0;
t259 = jx.*t12.*t37.*2.0;
t260 = jx.*t12.*t41.*2.0;
t261 = jx.*t12.*t44.*2.0;
t262 = jy.*t9.*t30.*2.0;
t263 = jy.*t9.*t32.*2.0;
t264 = jy.*t12.*t31.*2.0;
t265 = jy.*t9.*t35.*2.0;
t266 = jy.*t12.*t34.*2.0;
t267 = jy.*t9.*t39.*2.0;
t268 = jy.*t12.*t38.*2.0;
t269 = jy.*t12.*t42.*2.0;
t270 = t16+t28;
t271 = jx.*q1.*q2.*t30.*-2.0;
t272 = jx.*q0.*q3.*t31.*-2.0;
t273 = jx.*q1.*q2.*t32.*-2.0;
t274 = jx.*q0.*q3.*t33.*-2.0;
t275 = jx.*q0.*q3.*t34.*-2.0;
t276 = jx.*q1.*q2.*t35.*-2.0;
t277 = jx.*q1.*q2.*t36.*-2.0;
t278 = jx.*q0.*q3.*t37.*-2.0;
t279 = jx.*q0.*q3.*t38.*-2.0;
t280 = jx.*q1.*q2.*t39.*-2.0;
t281 = jx.*q1.*q2.*t40.*-2.0;
t282 = jx.*q0.*q3.*t41.*-2.0;
t283 = jx.*q0.*q3.*t42.*-2.0;
t284 = jx.*q1.*q2.*t43.*-2.0;
t285 = jx.*q0.*q3.*t44.*-2.0;
t286 = jx.*q1.*q2.*t45.*-2.0;
t287 = jy.*q0.*q3.*t31.*-2.0;
t288 = jy.*q0.*q3.*t33.*-2.0;
t289 = jy.*q0.*q3.*t34.*-2.0;
t290 = jy.*q1.*q2.*t35.*-2.0;
t291 = jy.*q0.*q3.*t37.*-2.0;
t292 = jy.*q1.*q2.*t39.*-2.0;
t293 = jy.*q1.*q2.*t43.*-2.0;
t294 = jy.*q1.*q2.*t45.*-2.0;
t295 = V1.*l.*t6.*t30.*2.0;
t296 = V1.*l.*t9.*t30.*2.0;
t297 = V1.*l.*t6.*t32.*2.0;
t298 = V1.*l.*t12.*t30.*2.0;
t299 = V1.*l.*t9.*t31.*2.0;
t300 = V1.*l.*t9.*t32.*2.0;
t301 = V1.*l.*t6.*t35.*2.0;
t302 = V1.*l.*t12.*t31.*2.0;
t303 = V1.*l.*t12.*t32.*2.0;
t304 = V1.*l.*t9.*t34.*2.0;
t305 = V1.*l.*t9.*t35.*2.0;
t306 = V1.*l.*t6.*t39.*2.0;
t307 = V1.*l.*t12.*t33.*2.0;
t308 = V1.*l.*t12.*t34.*2.0;
t309 = V1.*l.*t12.*t35.*2.0;
t310 = V1.*l.*t9.*t38.*2.0;
t311 = V1.*l.*t9.*t39.*2.0;
t312 = V1.*l.*t12.*t37.*2.0;
t313 = V1.*l.*t12.*t38.*2.0;
t314 = V1.*l.*t12.*t39.*2.0;
t315 = V1.*l.*t9.*t42.*2.0;
t316 = V1.*l.*t12.*t41.*2.0;
t317 = V1.*l.*t12.*t42.*2.0;
t318 = V1.*l.*t12.*t44.*2.0;
t319 = ddXd3.*l.*t6.*t30.*2.0;
t320 = ddXd3.*l.*t9.*t30.*2.0;
t321 = ddXd3.*l.*t6.*t32.*2.0;
t322 = ddXd3.*l.*t12.*t30.*2.0;
t323 = ddXd3.*l.*t9.*t31.*2.0;
t324 = ddXd3.*l.*t9.*t32.*2.0;
t325 = ddXd3.*l.*t6.*t35.*2.0;
t326 = ddXd3.*l.*t12.*t31.*2.0;
t327 = ddXd3.*l.*t12.*t32.*2.0;
t328 = ddXd3.*l.*t9.*t34.*2.0;
t329 = ddXd3.*l.*t9.*t35.*2.0;
t330 = ddXd3.*l.*t6.*t39.*2.0;
t331 = ddXd3.*l.*t12.*t33.*2.0;
t332 = ddXd3.*l.*t12.*t34.*2.0;
t333 = ddXd3.*l.*t12.*t35.*2.0;
t334 = ddXd3.*l.*t9.*t38.*2.0;
t335 = ddXd3.*l.*t9.*t39.*2.0;
t336 = ddXd3.*l.*t12.*t37.*2.0;
t337 = ddXd3.*l.*t12.*t38.*2.0;
t338 = ddXd3.*l.*t12.*t39.*2.0;
t339 = ddXd3.*l.*t9.*t42.*2.0;
t340 = ddXd3.*l.*t12.*t41.*2.0;
t341 = ddXd3.*l.*t12.*t42.*2.0;
t342 = ddXd3.*l.*t12.*t44.*2.0;
t343 = gravity.*l.*t6.*t30.*2.0;
t344 = gravity.*l.*t9.*t30.*2.0;
t345 = gravity.*l.*t6.*t32.*2.0;
t346 = gravity.*l.*t12.*t30.*2.0;
t347 = gravity.*l.*t9.*t31.*2.0;
t348 = gravity.*l.*t9.*t32.*2.0;
t349 = gravity.*l.*t6.*t35.*2.0;
t350 = gravity.*l.*t12.*t31.*2.0;
t351 = gravity.*l.*t12.*t32.*2.0;
t352 = gravity.*l.*t9.*t34.*2.0;
t353 = gravity.*l.*t9.*t35.*2.0;
t354 = gravity.*l.*t6.*t39.*2.0;
t355 = gravity.*l.*t12.*t33.*2.0;
t356 = gravity.*l.*t12.*t34.*2.0;
t357 = gravity.*l.*t12.*t35.*2.0;
t358 = gravity.*l.*t9.*t38.*2.0;
t359 = gravity.*l.*t9.*t39.*2.0;
t360 = gravity.*l.*t12.*t37.*2.0;
t361 = gravity.*l.*t12.*t38.*2.0;
t362 = gravity.*l.*t12.*t39.*2.0;
t363 = gravity.*l.*t9.*t42.*2.0;
t364 = gravity.*l.*t12.*t41.*2.0;
t365 = gravity.*l.*t12.*t42.*2.0;
t366 = gravity.*l.*t12.*t44.*2.0;
t367 = t221.^2;
t376 = q0.*t221.*4.0;
t377 = q1.*t221.*4.0;
t378 = q2.*t221.*4.0;
t379 = q3.*t221.*4.0;
t380 = o1.*o2.*t27.*t93;
t381 = o1.*o3.*t26.*t94;
t382 = o2.*o3.*t25.*t95;
t397 = -t385;
t398 = -t386;
t399 = -t387;
t400 = -t388;
t406 = t4+t12+t78+t79;
t407 = t4+t6+t79+t80;
t408 = t85+t86+t97;
t409 = t83+t87+t98;
t410 = t81+t91+t99;
t654 = t30+t31+t32+t33+t34+t35+t36+t37+t38+t39+t40+t41+t42+t43+t44+t45;
t154 = -t48;
t155 = -t49;
t156 = -t50;
t157 = -t52;
t158 = -t55;
t159 = -t57;
t160 = -t58;
t161 = -t59;
t368 = -t254;
t369 = -t256;
t370 = -t259;
t371 = -t261;
t372 = -t262;
t373 = -t263;
t374 = -t265;
t375 = -t267;
t383 = -t376;
t384 = -t377;
t411 = t407.^2;
t412 = q0.*t407.*4.0;
t413 = q1.*t407.*4.0;
t414 = q2.*t407.*4.0;
t415 = q3.*t407.*4.0;
t416 = 1.0./t406;
t417 = 1.0./t407;
t585 = t7+t10+t13+t15+t96+t148+t150+t151+t153+t202+t203;
t655 = 1.0./t654;
t893 = t100+t101+t102+t103+t104+t105+t106+t107+t108+t109+t110+t111+t112+t113+t114+t115+t116+t117+t118+t119+t120+t121+t122+t123+t124+t125+t126+t127+t128+t129+t130+t131+t132+t133+t134+t135+t136+t137+t138+t139+t140+t141+t142+t143+t144+t145+t146+t147+t295+t296+t297+t298+t299+t300+t301+t302+t303+t304+t305+t306+t307+t308+t309+t310+t311+t312+t313+t314+t315+t316+t317+t318+t319+t320+t321+t322+t323+t324+t325+t326+t327+t328+t329+t330+t331+t332+t333+t334+t335+t336+t337+t338+t339+t340+t341+t342+t343+t344+t345+t346+t347+t348+t349+t350+t351+t352+t353+t354+t355+t356+t357+t358+t359+t360+t361+t362+t363+t364+t365+t366;
t418 = t416.^2;
t419 = 1.0./t411;
t420 = t416.^3;
t421 = t417.^3;
t422 = t417.*2.0;
t438 = t22.*t416.*2.0;
t439 = q0.*t2.*t416.*2.0;
t440 = q1.*t2.*t416.*2.0;
t441 = q2.*t2.*t416.*2.0;
t442 = q3.*t2.*t416.*2.0;
t471 = km1.*m.*t22.*t27.*t416;
t472 = t367+t411;
t477 = t3.*t220.*t416;
t478 = t379+t412;
t479 = t378+t413;
t482 = t3.*t270.*t416;
t483 = t384+t414;
t484 = t383+t415;
t549 = (l.*m.*t22.*t25.*t29.*t416)./2.0;
t550 = (l.*m.*t22.*t26.*t29.*t416)./2.0;
t851 = t46+t47+t51+t54+t155+t157+t159+t161+t186+t188+t190+t192+t212+t213+t214+t215+t238+t240+t244+t248+t258+t260+t287+t288+t289+t291+t368+t369+t391+t393+t395+t396;
t852 = t53+t56+t60+t61+t154+t156+t158+t160+t186+t188+t190+t192+t212+t213+t214+t215+t238+t240+t244+t248+t255+t257+t287+t288+t289+t291+t370+t371+t391+t393+t395+t396;
t853 = t46+t47+t51+t54+t155+t157+t159+t161+t195+t197+t199+t201+t216+t217+t218+t219+t246+t249+t250+t252+t258+t260+t290+t292+t293+t294+t368+t369+t391+t393+t395+t396;
t860 = t62+t64+t65+t69+t162+t163+t164+t166+t204+t205+t206+t207+t208+t209+t210+t211+t222+t223+t225+t227+t228+t230+t233+t235+t264+t266+t372+t373+t385+t386+t392+t394+t399+t400+t401+t402;
t861 = t62+t64+t65+t69+t162+t163+t164+t166+t174+t175+t176+t177+t182+t183+t184+t185+t264+t266+t273+t275+t278+t280+t281+t283+t285+t286+t372+t373+t385+t386+t392+t394+t399+t400+t401+t402;
t862 = t67+t71+t73+t76+t165+t167+t168+t169+t170+t171+t172+t173+t178+t179+t180+t181+t268+t269+t271+t272+t274+t276+t277+t279+t282+t284+t374+t375+t387+t388+t389+t390+t397+t398+t403+t404;
t894 = 1.0./t893;
t423 = q0.*t422;
t424 = q1.*t422;
t425 = q2.*t422;
t426 = q3.*t422;
t427 = q0.*q1.*t419.*4.0;
t428 = q0.*q2.*t419.*4.0;
t429 = q1.*q3.*t419.*4.0;
t430 = q2.*q3.*t419.*4.0;
t432 = q0.*q3.*t419.*8.0;
t433 = q1.*q2.*t419.*8.0;
t434 = t4.*t419.*4.0;
t435 = t6.*t419.*4.0;
t436 = t9.*t419.*4.0;
t437 = t12.*t419.*4.0;
t445 = q0.*t438;
t446 = q1.*t438;
t447 = q2.*t438;
t448 = q3.*t438;
t449 = -t440;
t450 = -t442;
t451 = q0.*q1.*t22.*t418.*4.0;
t452 = q0.*q2.*t22.*t418.*4.0;
t453 = q0.*q3.*t22.*t418.*4.0;
t454 = q1.*q2.*t22.*t418.*4.0;
t455 = q1.*q3.*t22.*t418.*4.0;
t456 = q2.*q3.*t22.*t418.*4.0;
t457 = t4.*t22.*t418.*4.0;
t458 = t6.*t22.*t418.*4.0;
t459 = t9.*t22.*t418.*4.0;
t460 = t12.*t22.*t418.*4.0;
t463 = q0.*q1.*t22.*t418.*8.0;
t464 = q0.*q2.*t22.*t418.*8.0;
t465 = q1.*q3.*t22.*t418.*8.0;
t466 = q2.*q3.*t22.*t418.*8.0;
t470 = t221.*t419.*2.0;
t480 = q0.*t221.*t419.*-2.0;
t481 = q1.*t221.*t419.*-2.0;
t485 = 1.0./t472;
t487 = q0.*q1.*t221.*t421.*8.0;
t488 = q0.*q2.*t221.*t421.*8.0;
t489 = q0.*q3.*t221.*t421.*8.0;
t490 = q1.*q2.*t221.*t421.*8.0;
t491 = q1.*q3.*t221.*t421.*8.0;
t492 = q2.*q3.*t221.*t421.*8.0;
t493 = -t477;
t494 = t22.*t220.*t418.*2.0;
t495 = q0.*t2.*t220.*t418.*2.0;
t496 = q1.*t2.*t220.*t418.*2.0;
t497 = q2.*t2.*t220.*t418.*2.0;
t498 = q3.*t2.*t220.*t418.*2.0;
t499 = t4.*t221.*t421.*8.0;
t500 = t6.*t221.*t421.*8.0;
t501 = t9.*t221.*t421.*8.0;
t502 = t12.*t221.*t421.*8.0;
t506 = t22.*t270.*t418.*2.0;
t507 = q0.*t2.*t270.*t418.*2.0;
t509 = q1.*t2.*t270.*t418.*2.0;
t510 = q2.*t2.*t270.*t418.*2.0;
t511 = q3.*t2.*t270.*t418.*2.0;
t523 = q0.*t22.*t220.*t418.*-2.0;
t527 = q3.*t22.*t220.*t418.*-2.0;
t528 = q0.*q1.*t22.*t220.*t420.*8.0;
t529 = q0.*q2.*t22.*t220.*t420.*8.0;
t530 = q0.*q3.*t22.*t220.*t420.*8.0;
t531 = q1.*q2.*t22.*t220.*t420.*8.0;
t532 = q1.*q3.*t22.*t220.*t420.*8.0;
t533 = q2.*q3.*t22.*t220.*t420.*8.0;
t534 = q0.*t22.*t270.*t418.*-2.0;
t535 = q2.*t22.*t270.*t418.*-2.0;
t536 = t4.*t22.*t220.*t420.*8.0;
t537 = t6.*t22.*t220.*t420.*8.0;
t538 = t9.*t22.*t220.*t420.*8.0;
t539 = t12.*t22.*t220.*t420.*8.0;
t540 = q0.*q1.*t22.*t270.*t420.*8.0;
t541 = q0.*q2.*t22.*t270.*t420.*8.0;
t543 = q0.*q3.*t22.*t270.*t420.*8.0;
t545 = q1.*q2.*t22.*t270.*t420.*8.0;
t546 = q1.*q3.*t22.*t270.*t420.*8.0;
t548 = q2.*q3.*t22.*t270.*t420.*8.0;
t551 = t4.*t22.*t270.*t420.*8.0;
t553 = t6.*t22.*t270.*t420.*8.0;
t554 = t9.*t22.*t270.*t420.*8.0;
t555 = t12.*t22.*t270.*t420.*8.0;
t559 = -t549;
t560 = -t550;
t562 = t380+t471;
t431 = -t430;
t443 = -t434;
t444 = -t435;
t461 = -t454;
t462 = -t455;
t467 = -t457;
t468 = -t460;
t469 = -t466;
t473 = q0.*t470;
t474 = q1.*t470;
t475 = q2.*t470;
t476 = q3.*t470;
t486 = t485.^2;
t503 = -t487;
t504 = -t489;
t505 = -t490;
t508 = -t495;
t512 = -t498;
t513 = -t499;
t514 = -t500;
t515 = q0.*t494;
t516 = q1.*t494;
t517 = q2.*t494;
t518 = q3.*t494;
t519 = -t506;
t520 = -t507;
t521 = -t510;
t522 = q0.*t506;
t524 = q1.*t506;
t525 = q2.*t506;
t526 = q3.*t506;
t542 = -t529;
t544 = -t530;
t547 = -t532;
t552 = -t536;
t556 = -t539;
t557 = -t540;
t558 = -t541;
t561 = -t551;
t563 = t382+t559;
t564 = t381+t560;
t567 = t425+t481;
t568 = t426+t480;
t569 = t439+t497;
t570 = t442+t496;
t571 = t439+t509;
t574 = t441+t511;
t580 = t447+t523;
t581 = t446+t527;
t583 = t446+t534;
t584 = t448+t535;
t666 = t428+t429+t492;
t674 = t433+t470+t501;
t675 = t432+t470+t502;
t678 = t451+t456+t531;
t681 = t453+t454+t546;
t686 = t464+t494+t538;
t687 = t465+t494+t537;
t690 = t463+t506+t553;
t565 = t423+t476;
t566 = t424+t475;
t572 = t441+t508;
t573 = t440+t512;
t575 = t445+t517;
t576 = t448+t516;
t577 = t440+t520;
t578 = t442+t521;
t579 = t445+t524;
t582 = t447+t526;
t600 = (o1.*t580)./2.0;
t602 = (o2.*t580)./2.0;
t604 = (o1.*t581)./2.0;
t606 = (o3.*t580)./2.0;
t608 = (o2.*t581)./2.0;
t610 = (o3.*t581)./2.0;
t613 = (q1.*t580)./2.0;
t615 = (q0.*t581)./2.0;
t617 = (q2.*t580)./2.0;
t620 = (q1.*t581)./2.0;
t622 = (q3.*t580)./2.0;
t624 = (q2.*t581)./2.0;
t628 = (o1.*t583)./2.0;
t630 = (o2.*t583)./2.0;
t632 = (o3.*t583)./2.0;
t634 = (o1.*t584)./2.0;
t636 = (o2.*t584)./2.0;
t638 = (o3.*t584)./2.0;
t639 = (q1.*t583)./2.0;
t640 = (q2.*t583)./2.0;
t643 = (q3.*t583)./2.0;
t645 = (q0.*t584)./2.0;
t646 = (q1.*t584)./2.0;
t647 = (q3.*t584)./2.0;
t656 = t409.*t569;
t658 = t410.*t570;
t659 = t410.*t571;
t662 = t408.*t574;
t670 = t428+t429+t503;
t671 = t427+t431+t488;
t672 = t427+t431+t491;
t676 = t432+t470+t513;
t677 = t433+t470+t514;
t679 = t453+t461+t528;
t680 = t451+t456+t544;
t682 = t453+t461+t533;
t683 = t453+t454+t558;
t684 = t452+t462+t543;
t685 = t452+t462+t545;
t688 = t464+t494+t552;
t689 = t465+t494+t556;
t691 = t463+t506+t561;
t692 = t469+t506+t554;
t693 = t466+t519+t555;
t694 = t422+t437+t443+t504;
t695 = t422+t436+t444+t505;
t696 = t409.*t678;
t698 = t410.*t678;
t714 = t408.*t681;
t718 = t410.*t681;
t722 = (o1.*t411.*t485.*t567)./2.0;
t723 = (o1.*t411.*t485.*t568)./2.0;
t725 = (o2.*t411.*t485.*t567)./2.0;
t726 = (o2.*t411.*t485.*t568)./2.0;
t727 = (o3.*t411.*t485.*t567)./2.0;
t728 = (o3.*t411.*t485.*t568)./2.0;
t729 = (q0.*t411.*t485.*t567)./2.0;
t731 = (q1.*t411.*t485.*t568)./2.0;
t733 = (q2.*t411.*t485.*t567)./2.0;
t734 = (q2.*t411.*t485.*t568)./2.0;
t735 = (q3.*t411.*t485.*t567)./2.0;
t736 = (q3.*t411.*t485.*t568)./2.0;
t752 = t409.*t686;
t754 = t410.*t687;
t756 = t410.*t690;
t758 = t438+t459+t467+t542;
t759 = t438+t458+t468+t547;
t760 = t438+t459+t468+t548;
t763 = t438+t458+t467+t557;
t770 = t405.*t412.*t485.*t568;
t773 = t405.*t413.*t485.*t568;
t775 = t405.*t414.*t485.*t568;
t776 = t405.*t415.*t485.*t568;
t778 = t410.*t412.*t485.*t567;
t781 = q0.*t405.*t407.*t485.*t568.*-4.0;
t782 = t410.*t413.*t485.*t567;
t784 = t410.*t414.*t485.*t567;
t785 = t410.*t415.*t485.*t567;
t786 = q3.*t405.*t407.*t485.*t568.*-4.0;
t787 = q1.*t407.*t410.*t485.*t567.*-4.0;
t788 = q2.*t407.*t410.*t485.*t567.*-4.0;
t802 = t408.*t411.*t485.*t666;
t804 = t409.*t411.*t485.*t666;
t817 = t405.*t411.*t478.*t486.*t568;
t818 = t405.*t411.*t479.*t486.*t568;
t823 = t410.*t411.*t478.*t486.*t567;
t824 = t410.*t411.*t479.*t486.*t567;
t825 = -t405.*t411.*t486.*t568.*(t377-t414);
t826 = -t405.*t411.*t486.*t568.*(t376-t415);
t828 = -t410.*t411.*t486.*t567.*(t377-t414);
t829 = -t410.*t411.*t486.*t567.*(t376-t415);
t832 = t408.*t411.*t485.*t675;
t833 = t409.*t411.*t485.*t674;
t838 = t405.*t411.*t486.*t568.*(t377-t414);
t840 = t410.*t411.*t486.*t567.*(t376-t415);
t586 = (o1.*t575)./2.0;
t587 = (o2.*t575)./2.0;
t588 = (o1.*t576)./2.0;
t589 = (o3.*t575)./2.0;
t590 = (o2.*t576)./2.0;
t591 = (o3.*t576)./2.0;
t592 = (q0.*t575)./2.0;
t593 = (q1.*t575)./2.0;
t594 = (q0.*t576)./2.0;
t595 = (q3.*t575)./2.0;
t596 = (q2.*t576)./2.0;
t597 = (q3.*t576)./2.0;
t598 = (o1.*t579)./2.0;
t601 = (o2.*t579)./2.0;
t603 = (o3.*t579)./2.0;
t607 = (o1.*t582)./2.0;
t609 = (o2.*t582)./2.0;
t611 = (o3.*t582)./2.0;
t612 = (q0.*t579)./2.0;
t614 = (q2.*t579)./2.0;
t618 = (q3.*t579)./2.0;
t619 = (q0.*t582)./2.0;
t623 = (q1.*t582)./2.0;
t626 = (q2.*t582)./2.0;
t629 = -t600;
t631 = -t602;
t633 = -t604;
t641 = -t615;
t642 = -t617;
t648 = -t628;
t649 = -t630;
t650 = -t636;
t651 = -t639;
t652 = -t643;
t653 = -t647;
t657 = t405.*t572;
t660 = t405.*t577;
t661 = t408.*t573;
t663 = -t656;
t664 = t409.*t578;
t665 = -t658;
t669 = -t662;
t697 = t405.*t679;
t699 = t405.*t680;
t700 = (o1.*t411.*t485.*t565)./2.0;
t701 = (o1.*t411.*t485.*t566)./2.0;
t702 = (o2.*t411.*t485.*t565)./2.0;
t703 = (o2.*t411.*t485.*t566)./2.0;
t704 = (o3.*t411.*t485.*t565)./2.0;
t705 = (o3.*t411.*t485.*t566)./2.0;
t706 = (q0.*t411.*t485.*t565)./2.0;
t707 = (q0.*t411.*t485.*t566)./2.0;
t708 = (q1.*t411.*t485.*t565)./2.0;
t709 = (q1.*t411.*t485.*t566)./2.0;
t710 = (q2.*t411.*t485.*t565)./2.0;
t711 = (q3.*t411.*t485.*t566)./2.0;
t712 = t408.*t680;
t713 = t410.*t679;
t715 = t408.*t682;
t716 = t405.*t683;
t717 = t409.*t682;
t719 = t405.*t684;
t737 = t408.*t684;
t738 = t409.*t683;
t739 = t409.*t685;
t740 = t410.*t685;
t742 = -t726;
t743 = -t731;
t744 = -t734;
t745 = -t735;
t746 = -t736;
t749 = -t714;
t753 = t405.*t688;
t755 = t408.*t689;
t757 = t405.*t691;
t761 = t408.*t693;
t762 = t409.*t692;
t765 = t408.*t412.*t485.*t565;
t766 = t409.*t412.*t485.*t566;
t767 = t408.*t413.*t485.*t565;
t768 = t409.*t413.*t485.*t566;
t769 = t408.*t414.*t485.*t565;
t771 = t409.*t414.*t485.*t566;
t772 = t408.*t415.*t485.*t565;
t774 = t409.*t415.*t485.*t566;
t777 = q1.*t407.*t408.*t485.*t565.*-4.0;
t779 = q1.*t407.*t409.*t485.*t566.*-4.0;
t780 = q2.*t407.*t408.*t485.*t565.*-4.0;
t783 = q2.*t407.*t409.*t485.*t566.*-4.0;
t789 = t405.*t758;
t790 = t409.*t758;
t791 = t405.*t763;
t792 = t408.*t759;
t793 = t408.*t760;
t794 = t410.*t759;
t795 = t409.*t760;
t796 = t410.*t763;
t801 = t405.*t411.*t485.*t670;
t803 = t405.*t411.*t485.*t671;
t805 = t409.*t411.*t485.*t671;
t806 = t410.*t411.*t485.*t670;
t807 = t408.*t411.*t485.*t672;
t808 = t410.*t411.*t485.*t672;
t810 = -t804;
t813 = t408.*t411.*t478.*t486.*t565;
t814 = t408.*t411.*t479.*t486.*t565;
t815 = t409.*t411.*t478.*t486.*t566;
t816 = t409.*t411.*t479.*t486.*t566;
t819 = -t408.*t411.*t486.*t565.*(t377-t414);
t820 = -t408.*t411.*t486.*t565.*(t376-t415);
t821 = -t409.*t411.*t486.*t566.*(t377-t414);
t822 = -t409.*t411.*t486.*t566.*(t376-t415);
t831 = -t818;
t834 = t405.*t411.*t485.*t676;
t835 = t408.*t411.*t486.*t565.*(t376-t415);
t836 = t409.*t411.*t486.*t566.*(t376-t415);
t837 = -t823;
t839 = t410.*t411.*t485.*t677;
t841 = -t832;
t842 = t405.*t411.*t485.*t694;
t843 = t408.*t411.*t485.*t694;
t844 = t409.*t411.*t485.*t695;
t845 = t410.*t411.*t485.*t695;
t599 = -t586;
t605 = -t589;
t616 = -t594;
t621 = -t595;
t625 = -t596;
t627 = -t597;
t635 = -t609;
t637 = -t611;
t644 = -t619;
t667 = -t660;
t668 = -t661;
t673 = -t664;
t720 = -t700;
t721 = -t701;
t724 = -t702;
t730 = -t709;
t732 = -t710;
t741 = -t699;
t747 = -t712;
t748 = -t713;
t750 = -t715;
t751 = -t716;
t764 = -t761;
t797 = -t789;
t798 = -t791;
t799 = -t793;
t800 = -t794;
t809 = -t801;
t811 = -t805;
t812 = -t806;
t827 = -t813;
t830 = -t815;
t846 = -t844;
t850 = t618+t623+t640+t645;
t854 = t612+t626+t651+t653;
t858 = -t563.*(t594+t595-t613-t624);
t866 = t707+t708+t744+t745;
t870 = -t563.*(t710-t711-t729+t731);
t872 = t563.*(t710-t711-t729+t731);
t901 = -t409.*(t700+t726-t727+t769+t771+t784-t802-t803+t825-t833-t845-q2.*t405.*t407.*t485.*t568.*4.0+t408.*t411.*t486.*t565.*(t377-t414)+t409.*t411.*t486.*t566.*(t377-t414)+t410.*t411.*t486.*t567.*(t377-t414));
t902 = t409.*(t700+t726-t727+t769+t771+t784-t802-t803+t825-t833-t845-q2.*t405.*t407.*t485.*t568.*4.0+t408.*t411.*t486.*t565.*(t377-t414)+t409.*t411.*t486.*t566.*(t377-t414)+t410.*t411.*t486.*t567.*(t377-t414));
t847 = t592+t620+t627+t642;
t848 = t593+t622+t625+t641;
t849 = t613+t616+t621+t624;
t855 = t614+t644+t646+t652;
t863 = t564.*t850;
t864 = t563.*t854;
t865 = t706+t730+t733+t746;
t867 = t711+t729+t732+t743;
t871 = t564.*t866;
t873 = t570+t605+t608+t629+t696+t697+t754+t792;
t874 = t572+t587+t588+t610+t747+t748+t753+t790;
t875 = t569+t591+t631+t633+t698+t750+t752+t797;
t876 = t449+t498+t590+t599+t606+t717+t741+t755+t800;
t877 = t574+t601+t632+t634+t718+t719+t764+t795;
t878 = t577+t598+t637+t650+t737+t738+t757+t796;
t879 = t571+t635+t638+t648+t739+t749+t756+t798;
t880 = t450+t510+t603+t607+t649+t740+t751+t762+t799;
t895 = t705+t723+t724+t773+t777+t779+t787+t807+t809+t814+t816+t824+t831+t839+t846;
t896 = t720+t727+t742+t775+t780+t783+t788+t802+t803+t819+t821+t828+t833+t838+t845;
t897 = t703+t704+t722+t765+t766+t778+t781+t811+t812+t817+t827+t830+t834+t837+t843;
t898 = t721+t725+t728+t772+t774+t785+t786+t808+t810+t826+t835+t836+t840+t841+t842;
t856 = t562.*t848;
t857 = t562.*t855;
t859 = t564.*t847;
t868 = t562.*t865;
t881 = t410.*t873;
t883 = t405.*t874;
t884 = t409.*t875;
t885 = t408.*t876;
t887 = t408.*t877;
t888 = t405.*t878;
t891 = t410.*t879;
t892 = t409.*t880;
t899 = t405.*t897;
t900 = t410.*t895;
t903 = t408.*t898;
t869 = -t868;
t882 = -t881;
t886 = -t884;
t889 = -t887;
t890 = -t888;
t904 = V2+ddddXd1+t493+t657+t663+t665+t668+t856+t858+t859+t882+t883+t885+t886;
t905 = V3+ddddXd2+t482+t659+t667+t669+t673+t857+t863+t864+t889+t890+t891+t892;
t912 = V4+ddXd4+t869+t871+t872+t899+t900+t902+t903;
t906 = (t29.*t851.*t894.*t905)./2.0;
t907 = (t29.*t852.*t894.*t905)./2.0;
t908 = (t29.*t853.*t894.*t905)./2.0;
t909 = (t29.*t860.*t894.*t904)./2.0;
t910 = (t29.*t861.*t894.*t904)./2.0;
t911 = (t29.*t862.*t894.*t904)./2.0;
t913 = jz.*t416.*t585.*t655.*t912;
t914 = t24.*t416.*t585.*t655.*t912;
U2 = [t906-t907-t908+t909-t910+t911+t913;t908-t911+t914;t907+t910+t914;-t906-t909+t913];