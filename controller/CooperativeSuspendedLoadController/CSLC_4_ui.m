function ui = CSLC_4_ui(in1,in2,in3,in4,in5,in6,in7,in8,in9)
%CSLC_4_ui
%    UI = CSLC_4_ui(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    19-Jul-2023 17:18:21

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
do0d1 = in2(16,:);
do0d2 = in2(17,:);
do0d3 = in2(18,:);
j01 = in5(:,3);
j02 = in5(:,4);
j03 = in5(:,5);
kqi = in6(:,9);
kwi = in6(:,10);
li1 = in5(:,18);
li2 = in5(:,19);
li3 = in5(:,20);
li4 = in5(:,21);
m0 = in5(:,2);
mi1 = in5(:,22);
mi2 = in5(:,23);
mi3 = in5(:,24);
mi4 = in5(:,25);
mui1_1 = in8(1);
mui1_2 = in8(4);
mui1_3 = in8(7);
mui1_4 = in8(10);
mui2_1 = in8(2);
mui2_2 = in8(5);
mui2_3 = in8(8);
mui2_4 = in8(11);
mui3_1 = in8(3);
mui3_2 = in8(6);
mui3_3 = in8(9);
mui3_4 = in8(12);
o01 = in1(11,:);
o02 = in1(12,:);
o03 = in1(13,:);
o0d1 = in2(13,:);
o0d2 = in2(14,:);
o0d3 = in2(15,:);
qi1_1 = in1(14,:);
qi1_2 = in1(17,:);
qi1_3 = in1(20,:);
qi1_4 = in1(23,:);
qi2_1 = in1(15,:);
qi2_2 = in1(18,:);
qi2_3 = in1(21,:);
qi2_4 = in1(24,:);
qi3_1 = in1(16,:);
qi3_2 = in1(19,:);
qi3_3 = in1(22,:);
qi3_4 = in1(25,:);
qid1_1 = in9(1);
qid1_2 = in9(4);
qid1_3 = in9(7);
qid1_4 = in9(10);
qid2_1 = in9(2);
qid2_2 = in9(5);
qid2_3 = in9(8);
qid2_4 = in9(11);
qid3_1 = in9(3);
qid3_2 = in9(6);
qid3_3 = in9(9);
qid3_4 = in9(12);
rho1_1 = in5(:,6);
rho1_2 = in5(:,9);
rho1_3 = in5(:,12);
rho1_4 = in5(:,15);
rho2_1 = in5(:,7);
rho2_2 = in5(:,10);
rho2_3 = in5(:,13);
rho2_4 = in5(:,16);
rho3_1 = in5(:,8);
rho3_2 = in5(:,11);
rho3_3 = in5(:,14);
rho3_4 = in5(:,17);
wi1_1 = in1(26,:);
wi1_2 = in1(29,:);
wi1_3 = in1(32,:);
wi1_4 = in1(35,:);
wi2_1 = in1(27,:);
wi2_2 = in1(30,:);
wi2_3 = in1(33,:);
wi2_4 = in1(36,:);
wi3_1 = in1(28,:);
wi3_2 = in1(31,:);
wi3_3 = in1(34,:);
wi3_4 = in1(37,:);
t2 = R0d1_1.*o0d2;
t3 = R0d1_2.*o0d1;
t4 = R0d1_1.*o0d3;
t5 = R0d1_3.*o0d1;
t6 = R0d1_2.*o0d3;
t7 = R0d1_3.*o0d2;
t8 = R0d2_1.*o0d2;
t9 = R0d2_2.*o0d1;
t10 = R0d2_1.*o0d3;
t11 = R0d2_3.*o0d1;
t12 = R0d2_2.*o0d3;
t13 = R0d2_3.*o0d2;
t14 = R0d3_1.*o0d2;
t15 = R0d3_2.*o0d1;
t16 = R0d3_1.*o0d3;
t17 = R0d3_3.*o0d1;
t18 = R0d3_2.*o0d3;
t19 = R0d3_3.*o0d2;
t20 = R0d1_1.*qid1_1;
t21 = R0d1_1.*qid1_2;
t22 = R0d1_2.*qid1_1;
t23 = R0d1_1.*qid1_3;
t24 = R0d1_2.*qid1_2;
t25 = R0d1_3.*qid1_1;
t26 = R0d1_1.*qid1_4;
t27 = R0d1_2.*qid1_3;
t28 = R0d1_3.*qid1_2;
t29 = R0d1_2.*qid1_4;
t30 = R0d1_3.*qid1_3;
t31 = R0d1_3.*qid1_4;
t32 = R0d2_1.*qid2_1;
t33 = R0d2_1.*qid2_2;
t34 = R0d2_2.*qid2_1;
t35 = R0d2_1.*qid2_3;
t36 = R0d2_2.*qid2_2;
t37 = R0d2_3.*qid2_1;
t38 = R0d2_1.*qid2_4;
t39 = R0d2_2.*qid2_3;
t40 = R0d2_3.*qid2_2;
t41 = R0d2_2.*qid2_4;
t42 = R0d2_3.*qid2_3;
t43 = R0d2_3.*qid2_4;
t44 = R0d3_1.*qid3_1;
t45 = R0d3_1.*qid3_2;
t46 = R0d3_2.*qid3_1;
t47 = R0d3_1.*qid3_3;
t48 = R0d3_2.*qid3_2;
t49 = R0d3_3.*qid3_1;
t50 = R0d3_1.*qid3_4;
t51 = R0d3_2.*qid3_3;
t52 = R0d3_3.*qid3_2;
t53 = R0d3_2.*qid3_4;
t54 = R0d3_3.*qid3_3;
t55 = R0d3_3.*qid3_4;
t56 = R01_2.*rho1_1;
t57 = R01_2.*rho1_2;
t58 = R01_3.*rho1_1;
t59 = R01_2.*rho1_3;
t60 = R01_3.*rho1_2;
t61 = R01_2.*rho1_4;
t62 = R01_3.*rho1_3;
t63 = R01_3.*rho1_4;
t64 = R01_1.*rho2_1;
t65 = R01_1.*rho2_2;
t66 = R02_2.*rho1_1;
t67 = R01_1.*rho2_3;
t68 = R01_3.*rho2_1;
t69 = R02_2.*rho1_2;
t70 = R02_3.*rho1_1;
t71 = R01_1.*rho2_4;
t72 = R01_3.*rho2_2;
t73 = R02_2.*rho1_3;
t74 = R02_3.*rho1_2;
t75 = R01_3.*rho2_3;
t76 = R02_2.*rho1_4;
t77 = R02_3.*rho1_3;
t78 = R01_3.*rho2_4;
t79 = R02_3.*rho1_4;
t80 = R01_1.*rho3_1;
t81 = R02_1.*rho2_1;
t82 = R01_1.*rho3_2;
t83 = R01_2.*rho3_1;
t84 = R02_1.*rho2_2;
t85 = R03_2.*rho1_1;
t86 = R01_1.*rho3_3;
t87 = R01_2.*rho3_2;
t88 = R02_1.*rho2_3;
t89 = R02_3.*rho2_1;
t90 = R03_2.*rho1_2;
t91 = R03_3.*rho1_1;
t92 = R01_1.*rho3_4;
t93 = R01_2.*rho3_3;
t94 = R02_1.*rho2_4;
t95 = R02_3.*rho2_2;
t96 = R03_2.*rho1_3;
t97 = R03_3.*rho1_2;
t98 = R01_2.*rho3_4;
t99 = R02_3.*rho2_3;
t100 = R03_2.*rho1_4;
t101 = R03_3.*rho1_3;
t102 = R02_3.*rho2_4;
t103 = R03_3.*rho1_4;
t104 = R02_1.*rho3_1;
t105 = R03_1.*rho2_1;
t106 = R02_1.*rho3_2;
t107 = R02_2.*rho3_1;
t108 = R03_1.*rho2_2;
t109 = R02_1.*rho3_3;
t110 = R02_2.*rho3_2;
t111 = R03_1.*rho2_3;
t112 = R03_3.*rho2_1;
t113 = R02_1.*rho3_4;
t114 = R02_2.*rho3_3;
t115 = R03_1.*rho2_4;
t116 = R03_3.*rho2_2;
t117 = R02_2.*rho3_4;
t118 = R03_3.*rho2_3;
t119 = R03_3.*rho2_4;
t120 = R03_1.*rho3_1;
t121 = R03_1.*rho3_2;
t122 = R03_2.*rho3_1;
t123 = R03_1.*rho3_3;
t124 = R03_2.*rho3_2;
t125 = R03_1.*rho3_4;
t126 = R03_2.*rho3_3;
t127 = R03_2.*rho3_4;
t128 = o0d1.*o0d2;
t129 = o0d1.*o0d3;
t130 = o0d2.*o0d3;
t131 = qi1_1.*qid2_1;
t132 = qi2_1.*qid1_1;
t133 = qi1_2.*qid2_2;
t134 = qi2_2.*qid1_2;
t135 = qi1_3.*qid2_3;
t136 = qi2_3.*qid1_3;
t137 = qi1_4.*qid2_4;
t138 = qi2_4.*qid1_4;
t139 = qi1_1.*qid3_1;
t140 = qi3_1.*qid1_1;
t141 = qi1_2.*qid3_2;
t142 = qi3_2.*qid1_2;
t143 = qi1_3.*qid3_3;
t144 = qi3_3.*qid1_3;
t145 = qi1_4.*qid3_4;
t146 = qi3_4.*qid1_4;
t147 = qi2_1.*qid3_1;
t148 = qi3_1.*qid2_1;
t149 = qi2_2.*qid3_2;
t150 = qi3_2.*qid2_2;
t151 = qi2_3.*qid3_3;
t152 = qi3_3.*qid2_3;
t153 = qi2_4.*qid3_4;
t154 = qi3_4.*qid2_4;
t155 = qi1_1.*wi2_1;
t156 = qi2_1.*wi1_1;
t157 = qi1_2.*wi2_2;
t158 = qi2_2.*wi1_2;
t159 = qi1_3.*wi2_3;
t160 = qi2_3.*wi1_3;
t161 = qi1_4.*wi2_4;
t162 = qi2_4.*wi1_4;
t163 = qi1_1.*wi3_1;
t164 = qi3_1.*wi1_1;
t165 = qi1_2.*wi3_2;
t166 = qi3_2.*wi1_2;
t167 = qi1_3.*wi3_3;
t168 = qi3_3.*wi1_3;
t169 = qi1_4.*wi3_4;
t170 = qi3_4.*wi1_4;
t171 = qi2_1.*wi3_1;
t172 = qi3_1.*wi2_1;
t173 = qi2_2.*wi3_2;
t174 = qi3_2.*wi2_2;
t175 = qi2_3.*wi3_3;
t176 = qi3_3.*wi2_3;
t177 = qi2_4.*wi3_4;
t178 = qi3_4.*wi2_4;
t179 = o01.^2;
t180 = o02.^2;
t181 = o03.^2;
t182 = o0d1.^2;
t183 = o0d2.^2;
t184 = o0d3.^2;
t185 = qi1_1.^2;
t186 = qi1_2.^2;
t187 = qi1_3.^2;
t188 = qi1_4.^2;
t189 = qi2_1.^2;
t190 = qi2_2.^2;
t191 = qi2_3.^2;
t192 = qi2_4.^2;
t193 = qi3_1.^2;
t194 = qi3_2.^2;
t195 = qi3_3.^2;
t196 = qi3_4.^2;
t197 = wi1_1.^2;
t198 = wi1_2.^2;
t199 = wi1_3.^2;
t200 = wi1_4.^2;
t201 = wi2_1.^2;
t202 = wi2_2.^2;
t203 = wi2_3.^2;
t204 = wi2_4.^2;
t205 = wi3_1.^2;
t206 = wi3_2.^2;
t207 = wi3_3.^2;
t208 = wi3_4.^2;
t209 = R01_1.*o01.*o02;
t210 = R01_1.*o01.*o03;
t211 = R01_2.*o01.*o02;
t212 = R01_2.*o02.*o03;
t213 = R01_3.*o01.*o03;
t214 = R01_3.*o02.*o03;
t215 = R02_1.*o01.*o02;
t216 = R02_1.*o01.*o03;
t217 = R02_2.*o01.*o02;
t218 = R02_2.*o02.*o03;
t219 = R02_3.*o01.*o03;
t220 = R02_3.*o02.*o03;
t221 = R03_1.*o01.*o02;
t222 = R03_1.*o01.*o03;
t223 = R03_2.*o01.*o02;
t224 = R03_2.*o02.*o03;
t225 = R03_3.*o01.*o03;
t226 = R03_3.*o02.*o03;
t227 = j01.*o01.*o02;
t228 = j01.*o01.*o03;
t229 = j02.*o01.*o02;
t230 = j02.*o02.*o03;
t231 = j03.*o01.*o03;
t232 = j03.*o02.*o03;
t233 = 1.0./j01;
t234 = 1.0./j02;
t235 = 1.0./j03;
t236 = 1.0./m0;
t237 = -wi2_1;
t238 = -wi2_2;
t239 = -wi2_3;
t240 = -wi2_4;
t244 = mui1_1+mui1_2+mui1_3+mui1_4;
t245 = mui2_1+mui2_2+mui2_3+mui2_4;
t246 = mui3_1+mui3_2+mui3_3+mui3_4;
t241 = do0d1+t130;
t242 = do0d2+t129;
t243 = do0d3+t128;
t247 = -t3;
t248 = -t5;
t249 = -t7;
t250 = -t9;
t251 = -t11;
t252 = -t13;
t253 = -t15;
t254 = -t17;
t255 = -t19;
t256 = -t64;
t257 = -t65;
t258 = -t67;
t259 = -t71;
t260 = -t80;
t261 = -t81;
t262 = -t82;
t263 = -t83;
t264 = -t84;
t265 = -t86;
t266 = -t87;
t267 = -t88;
t268 = -t92;
t269 = -t93;
t270 = -t94;
t271 = -t98;
t272 = -t104;
t273 = -t105;
t274 = -t106;
t275 = -t107;
t276 = -t108;
t277 = -t109;
t278 = -t110;
t279 = -t111;
t280 = -t113;
t281 = -t114;
t282 = -t115;
t283 = -t117;
t284 = -t120;
t285 = -t121;
t286 = -t122;
t287 = -t123;
t288 = -t124;
t289 = -t125;
t290 = -t126;
t291 = -t127;
t292 = -t128;
t293 = -t129;
t294 = -t130;
t295 = -t132;
t296 = -t134;
t297 = -t136;
t298 = -t138;
t299 = -t140;
t300 = -t142;
t301 = -t144;
t302 = -t146;
t303 = -t148;
t304 = -t150;
t305 = -t152;
t306 = -t154;
t307 = -t156;
t308 = -t158;
t309 = -t160;
t310 = -t162;
t311 = -t164;
t312 = -t166;
t313 = -t168;
t314 = -t170;
t315 = -t172;
t316 = -t174;
t317 = -t176;
t318 = -t178;
t319 = -t229;
t320 = -t231;
t321 = -t232;
t322 = t185+t189;
t323 = t186+t190;
t324 = t187+t191;
t325 = t188+t192;
t326 = t185+t193;
t327 = t186+t194;
t328 = t187+t195;
t329 = t188+t196;
t330 = t189+t193;
t331 = t190+t194;
t332 = t191+t195;
t333 = t192+t196;
t346 = t179+t180;
t347 = t179+t181;
t348 = t180+t181;
t349 = t182+t183;
t350 = t182+t184;
t351 = t183+t184;
t457 = t236.*t244;
t458 = t236.*t245;
t459 = t236.*t246;
t460 = t20+t32+t44;
t461 = t21+t33+t45;
t462 = t22+t34+t46;
t463 = t23+t35+t47;
t464 = t24+t36+t48;
t465 = t25+t37+t49;
t466 = t26+t38+t50;
t467 = t27+t39+t51;
t468 = t28+t40+t52;
t469 = t29+t41+t53;
t470 = t30+t42+t54;
t471 = t31+t43+t55;
t472 = t197+t201+t205;
t473 = t198+t202+t206;
t474 = t199+t203+t207;
t475 = t200+t204+t208;
t334 = do0d1+t294;
t335 = do0d2+t293;
t336 = do0d3+t292;
t337 = R0d1_1.*t242;
t338 = R0d1_2.*t243;
t339 = R0d1_3.*t241;
t340 = R0d2_1.*t242;
t341 = R0d2_2.*t243;
t342 = R0d2_3.*t241;
t343 = R0d3_1.*t242;
t344 = R0d3_2.*t243;
t345 = R0d3_3.*t241;
t352 = t2+t247;
t353 = t4+t248;
t354 = t6+t249;
t355 = t8+t250;
t356 = t10+t251;
t357 = t12+t252;
t358 = t14+t253;
t359 = t16+t254;
t360 = t18+t255;
t361 = t56+t256;
t362 = t57+t257;
t363 = t59+t258;
t364 = t61+t259;
t365 = t58+t260;
t366 = t60+t262;
t367 = t62+t265;
t368 = t63+t268;
t369 = t66+t261;
t370 = t68+t263;
t371 = t69+t264;
t372 = t72+t266;
t373 = t73+t267;
t374 = t75+t269;
t375 = t76+t270;
t376 = t78+t271;
t377 = t70+t272;
t378 = t74+t274;
t379 = t77+t277;
t380 = t79+t280;
t381 = t85+t273;
ui = ft_1({R01_1,R01_2,R01_3,R02_1,R02_2,R02_3,R03_1,R03_2,R03_3,R0d1_1,R0d1_2,R0d1_3,R0d2_1,R0d2_2,R0d2_3,R0d3_1,R0d3_2,R0d3_3,kqi,kwi,li1,li2,li3,li4,mi1,mi2,mi3,mi4,mui1_1,mui1_2,mui1_3,mui1_4,mui2_1,mui2_2,mui2_3,mui2_4,mui3_1,mui3_2,mui3_3,mui3_4,qi1_1,qi1_2,qi1_3,qi1_4,qi2_1,qi2_2,qi2_3,qi2_4,qi3_1,qi3_2,qi3_3,qi3_4,qid1_1,qid1_2,qid1_3,qid1_4,qid2_1,qid2_2,qid2_3,qid2_4,qid3_1,qid3_2,qid3_3,qid3_4,rho1_1,rho1_2,rho1_3,rho1_4,rho2_1,rho2_2,rho2_3,rho2_4,rho3_1,rho3_2,rho3_3,rho3_4,t100,t101,t102,t103,t112,t116,t118,t119,t131,t133,t135,t137,t139,t141,t143,t145,t147,t149,t151,t153,t155,t157,t159,t161,t163,t165,t167,t169,t171,t173,t175,t177,t185,t186,t187,t188,t189,t190,t191,t192,t193,t194,t195,t196,t209,t210,t211,t212,t213,t214,t215,t216,t217,t218,t219,t220,t221,t222,t223,t224,t225,t226,t227,t228,t230,t233,t234,t235,t237,t238,t239,t240,t275,t276,t278,t279,t281,t282,t283,t284,t285,t286,t287,t288,t289,t290,t291,t295,t296,t297,t298,t299,t300,t301,t302,t303,t304,t305,t306,t307,t308,t309,t310,t311,t312,t313,t314,t315,t316,t317,t318,t319,t320,t321,t322,t323,t324,t325,t326,t327,t328,t329,t330,t331,t332,t333,t334,t335,t336,t337,t338,t339,t340,t341,t342,t343,t344,t345,t346,t347,t348,t349,t350,t351,t352,t353,t354,t355,t356,t357,t358,t359,t360,t361,t362,t363,t364,t365,t366,t367,t368,t369,t370,t371,t372,t373,t374,t375,t376,t377,t378,t379,t380,t381,t457,t458,t459,t460,t461,t462,t463,t464,t465,t466,t467,t468,t469,t470,t471,t472,t473,t474,t475,t89,t90,t91,t95,t96,t97,t99,wi1_1,wi1_2,wi1_3,wi1_4,wi3_1,wi3_2,wi3_3,wi3_4});
end
function ui = ft_1(ct)
R01_1 = ct{1};
R01_2 = ct{2};
R01_3 = ct{3};
R02_1 = ct{4};
R02_2 = ct{5};
R02_3 = ct{6};
R03_1 = ct{7};
R03_2 = ct{8};
R03_3 = ct{9};
R0d1_1 = ct{10};
R0d1_2 = ct{11};
R0d1_3 = ct{12};
R0d2_1 = ct{13};
R0d2_2 = ct{14};
R0d2_3 = ct{15};
R0d3_1 = ct{16};
R0d3_2 = ct{17};
R0d3_3 = ct{18};
kqi = ct{19};
kwi = ct{20};
li1 = ct{21};
li2 = ct{22};
li3 = ct{23};
li4 = ct{24};
mi1 = ct{25};
mi2 = ct{26};
mi3 = ct{27};
mi4 = ct{28};
mui1_1 = ct{29};
mui1_2 = ct{30};
mui1_3 = ct{31};
mui1_4 = ct{32};
mui2_1 = ct{33};
mui2_2 = ct{34};
mui2_3 = ct{35};
mui2_4 = ct{36};
mui3_1 = ct{37};
mui3_2 = ct{38};
mui3_3 = ct{39};
mui3_4 = ct{40};
qi1_1 = ct{41};
qi1_2 = ct{42};
qi1_3 = ct{43};
qi1_4 = ct{44};
qi2_1 = ct{45};
qi2_2 = ct{46};
qi2_3 = ct{47};
qi2_4 = ct{48};
qi3_1 = ct{49};
qi3_2 = ct{50};
qi3_3 = ct{51};
qi3_4 = ct{52};
qid1_1 = ct{53};
qid1_2 = ct{54};
qid1_3 = ct{55};
qid1_4 = ct{56};
qid2_1 = ct{57};
qid2_2 = ct{58};
qid2_3 = ct{59};
qid2_4 = ct{60};
qid3_1 = ct{61};
qid3_2 = ct{62};
qid3_3 = ct{63};
qid3_4 = ct{64};
rho1_1 = ct{65};
rho1_2 = ct{66};
rho1_3 = ct{67};
rho1_4 = ct{68};
rho2_1 = ct{69};
rho2_2 = ct{70};
rho2_3 = ct{71};
rho2_4 = ct{72};
rho3_1 = ct{73};
rho3_2 = ct{74};
rho3_3 = ct{75};
rho3_4 = ct{76};
t100 = ct{77};
t101 = ct{78};
t102 = ct{79};
t103 = ct{80};
t112 = ct{81};
t116 = ct{82};
t118 = ct{83};
t119 = ct{84};
t131 = ct{85};
t133 = ct{86};
t135 = ct{87};
t137 = ct{88};
t139 = ct{89};
t141 = ct{90};
t143 = ct{91};
t145 = ct{92};
t147 = ct{93};
t149 = ct{94};
t151 = ct{95};
t153 = ct{96};
t155 = ct{97};
t157 = ct{98};
t159 = ct{99};
t161 = ct{100};
t163 = ct{101};
t165 = ct{102};
t167 = ct{103};
t169 = ct{104};
t171 = ct{105};
t173 = ct{106};
t175 = ct{107};
t177 = ct{108};
t185 = ct{109};
t186 = ct{110};
t187 = ct{111};
t188 = ct{112};
t189 = ct{113};
t190 = ct{114};
t191 = ct{115};
t192 = ct{116};
t193 = ct{117};
t194 = ct{118};
t195 = ct{119};
t196 = ct{120};
t209 = ct{121};
t210 = ct{122};
t211 = ct{123};
t212 = ct{124};
t213 = ct{125};
t214 = ct{126};
t215 = ct{127};
t216 = ct{128};
t217 = ct{129};
t218 = ct{130};
t219 = ct{131};
t220 = ct{132};
t221 = ct{133};
t222 = ct{134};
t223 = ct{135};
t224 = ct{136};
t225 = ct{137};
t226 = ct{138};
t227 = ct{139};
t228 = ct{140};
t230 = ct{141};
t233 = ct{142};
t234 = ct{143};
t235 = ct{144};
t237 = ct{145};
t238 = ct{146};
t239 = ct{147};
t240 = ct{148};
t275 = ct{149};
t276 = ct{150};
t278 = ct{151};
t279 = ct{152};
t281 = ct{153};
t282 = ct{154};
t283 = ct{155};
t284 = ct{156};
t285 = ct{157};
t286 = ct{158};
t287 = ct{159};
t288 = ct{160};
t289 = ct{161};
t290 = ct{162};
t291 = ct{163};
t295 = ct{164};
t296 = ct{165};
t297 = ct{166};
t298 = ct{167};
t299 = ct{168};
t300 = ct{169};
t301 = ct{170};
t302 = ct{171};
t303 = ct{172};
t304 = ct{173};
t305 = ct{174};
t306 = ct{175};
t307 = ct{176};
t308 = ct{177};
t309 = ct{178};
t310 = ct{179};
t311 = ct{180};
t312 = ct{181};
t313 = ct{182};
t314 = ct{183};
t315 = ct{184};
t316 = ct{185};
t317 = ct{186};
t318 = ct{187};
t319 = ct{188};
t320 = ct{189};
t321 = ct{190};
t322 = ct{191};
t323 = ct{192};
t324 = ct{193};
t325 = ct{194};
t326 = ct{195};
t327 = ct{196};
t328 = ct{197};
t329 = ct{198};
t330 = ct{199};
t331 = ct{200};
t332 = ct{201};
t333 = ct{202};
t334 = ct{203};
t335 = ct{204};
t336 = ct{205};
t337 = ct{206};
t338 = ct{207};
t339 = ct{208};
t340 = ct{209};
t341 = ct{210};
t342 = ct{211};
t343 = ct{212};
t344 = ct{213};
t345 = ct{214};
t346 = ct{215};
t347 = ct{216};
t348 = ct{217};
t349 = ct{218};
t350 = ct{219};
t351 = ct{220};
t352 = ct{221};
t353 = ct{222};
t354 = ct{223};
t355 = ct{224};
t356 = ct{225};
t357 = ct{226};
t358 = ct{227};
t359 = ct{228};
t360 = ct{229};
t361 = ct{230};
t362 = ct{231};
t363 = ct{232};
t364 = ct{233};
t365 = ct{234};
t366 = ct{235};
t367 = ct{236};
t368 = ct{237};
t369 = ct{238};
t370 = ct{239};
t371 = ct{240};
t372 = ct{241};
t373 = ct{242};
t374 = ct{243};
t375 = ct{244};
t376 = ct{245};
t377 = ct{246};
t378 = ct{247};
t379 = ct{248};
t380 = ct{249};
t381 = ct{250};
t457 = ct{251};
t458 = ct{252};
t459 = ct{253};
t460 = ct{254};
t461 = ct{255};
t462 = ct{256};
t463 = ct{257};
t464 = ct{258};
t465 = ct{259};
t466 = ct{260};
t467 = ct{261};
t468 = ct{262};
t469 = ct{263};
t470 = ct{264};
t471 = ct{265};
t472 = ct{266};
t473 = ct{267};
t474 = ct{268};
t475 = ct{269};
t89 = ct{270};
t90 = ct{271};
t91 = ct{272};
t95 = ct{273};
t96 = ct{274};
t97 = ct{275};
t99 = ct{276};
wi1_1 = ct{277};
wi1_2 = ct{278};
wi1_3 = ct{279};
wi1_4 = ct{280};
wi3_1 = ct{281};
wi3_2 = ct{282};
wi3_3 = ct{283};
wi3_4 = ct{284};
t382 = t89+t275;
t383 = t90+t276;
t384 = t95+t278;
t385 = t96+t279;
t386 = t99+t281;
t387 = t100+t282;
t388 = t102+t283;
t389 = t91+t284;
t390 = t97+t285;
t391 = t101+t287;
t392 = t103+t289;
t393 = t112+t286;
t394 = t116+t288;
t395 = t118+t290;
t396 = t119+t291;
t397 = R01_1.*t348;
t398 = R01_2.*t347;
t399 = R01_3.*t346;
t400 = R02_1.*t348;
t401 = R02_2.*t347;
t402 = R02_3.*t346;
t403 = R03_1.*t348;
t404 = R03_2.*t347;
t405 = R03_3.*t346;
t406 = R0d1_1.*t351;
t407 = R0d1_2.*t350;
t408 = R0d1_3.*t349;
t409 = R0d2_1.*t351;
t410 = R0d2_2.*t350;
t411 = R0d2_3.*t349;
t412 = R0d3_1.*t351;
t413 = R0d3_2.*t350;
t414 = R0d3_3.*t349;
t433 = t131+t295;
t434 = t133+t296;
t435 = t135+t297;
t436 = t137+t298;
t437 = t139+t299;
t438 = t141+t300;
t439 = t143+t301;
t440 = t145+t302;
t441 = t147+t303;
t442 = t149+t304;
t443 = t151+t305;
t444 = t153+t306;
t445 = t155+t307;
t446 = t157+t308;
t447 = t159+t309;
t448 = t161+t310;
t449 = t163+t311;
t450 = t165+t312;
t451 = t167+t313;
t452 = t169+t314;
t453 = t171+t315;
t454 = t173+t316;
t455 = t175+t317;
t456 = t177+t318;
t415 = R0d1_1.*t336;
t416 = -t337;
t417 = R0d1_2.*t334;
t418 = -t338;
t419 = R0d1_3.*t335;
t420 = -t339;
t421 = R0d2_1.*t336;
t422 = -t340;
t423 = R0d2_2.*t334;
t424 = -t341;
t425 = R0d2_3.*t335;
t426 = -t342;
t427 = R0d3_1.*t336;
t428 = -t343;
t429 = R0d3_2.*t334;
t430 = -t344;
t431 = R0d3_3.*t335;
t432 = -t345;
t476 = mui1_1.*t361;
t477 = mui1_2.*t362;
t478 = mui1_3.*t363;
t479 = mui1_4.*t364;
t480 = mui1_1.*t365;
t481 = mui1_2.*t366;
t482 = mui1_3.*t367;
t483 = mui1_4.*t368;
t484 = mui1_1.*t370;
t485 = mui1_2.*t372;
t486 = mui1_3.*t374;
t487 = mui2_1.*t369;
t488 = mui1_4.*t376;
t489 = mui2_2.*t371;
t490 = mui2_3.*t373;
t491 = mui2_4.*t375;
t492 = mui2_1.*t377;
t493 = mui2_2.*t378;
t494 = mui2_3.*t379;
t495 = mui2_4.*t380;
t496 = mui2_1.*t382;
t497 = mui2_2.*t384;
t498 = mui2_3.*t386;
t499 = mui3_1.*t381;
t500 = mui2_4.*t388;
t501 = mui3_2.*t383;
t502 = mui3_3.*t385;
t503 = mui3_4.*t387;
t504 = mui3_1.*t389;
t505 = mui3_2.*t390;
t506 = mui3_3.*t391;
t507 = mui3_4.*t392;
t508 = mui3_1.*t393;
t509 = mui3_2.*t394;
t510 = mui3_3.*t395;
t511 = mui3_4.*t396;
t512 = -t397;
t513 = -t398;
t514 = -t399;
t515 = -t400;
t516 = -t401;
t517 = -t402;
t518 = -t403;
t519 = -t404;
t520 = -t405;
t521 = kqi.*t433;
t522 = kqi.*t434;
t523 = kqi.*t435;
t524 = kqi.*t436;
t525 = kqi.*t437;
t526 = kqi.*t438;
t527 = kqi.*t439;
t528 = kqi.*t440;
t529 = kqi.*t441;
t530 = kqi.*t442;
t531 = kqi.*t443;
t532 = kqi.*t444;
t550 = t354.*t460;
t551 = t353.*t462;
t552 = t352.*t465;
t553 = t354.*t461;
t554 = t353.*t464;
t555 = t352.*t468;
t556 = t354.*t463;
t557 = t353.*t467;
t558 = t352.*t470;
t559 = t354.*t466;
t560 = t353.*t469;
t561 = t352.*t471;
t562 = t357.*t460;
t563 = t356.*t462;
t564 = t355.*t465;
t565 = t357.*t461;
t566 = t356.*t464;
t567 = t355.*t468;
t568 = t357.*t463;
t569 = t356.*t467;
t570 = t355.*t470;
t571 = t357.*t466;
t572 = t356.*t469;
t573 = t355.*t471;
t574 = t360.*t460;
t575 = t359.*t462;
t576 = t358.*t465;
t577 = t360.*t461;
t578 = t359.*t464;
t579 = t358.*t468;
t580 = t360.*t463;
t581 = t359.*t467;
t582 = t358.*t470;
t583 = t360.*t466;
t584 = t359.*t469;
t585 = t358.*t471;
t533 = -t521;
t534 = -t522;
t535 = -t523;
t536 = -t524;
t537 = -t529;
t538 = -t530;
t539 = -t531;
t540 = -t532;
t541 = t211+t213+t512;
t542 = t209+t214+t513;
t543 = t210+t212+t514;
t544 = t217+t219+t515;
t545 = t215+t220+t516;
t546 = t216+t218+t517;
t547 = t223+t225+t518;
t548 = t221+t226+t519;
t549 = t222+t224+t520;
t586 = -t551;
t587 = -t554;
t588 = -t557;
t589 = -t560;
t590 = -t563;
t591 = -t566;
t592 = -t569;
t593 = -t572;
t594 = -t575;
t595 = -t578;
t596 = -t581;
t597 = -t584;
t634 = t408+t416+t417;
t635 = t407+t415+t420;
t636 = t406+t418+t419;
t637 = t411+t422+t423;
t638 = t410+t421+t426;
t639 = t409+t424+t425;
t640 = t414+t428+t429;
t641 = t413+t427+t432;
t642 = t412+t430+t431;
t835 = t227+t319+t476+t477+t478+t479+t487+t489+t490+t491+t499+t501+t502+t503;
t836 = t228+t320+t480+t481+t482+t483+t492+t493+t494+t495+t504+t505+t506+t507;
t837 = t230+t321+t484+t485+t486+t488+t496+t497+t498+t500+t508+t509+t510+t511;
t598 = rho1_1.*t541;
t599 = rho1_2.*t541;
t600 = rho1_3.*t541;
t601 = rho1_4.*t541;
t602 = rho2_1.*t542;
t603 = rho2_2.*t542;
t604 = rho2_3.*t542;
t605 = rho2_4.*t542;
t606 = rho3_1.*t543;
t607 = rho3_2.*t543;
t608 = rho3_3.*t543;
t609 = rho3_4.*t543;
t610 = rho1_1.*t544;
t611 = rho1_2.*t544;
t612 = rho1_3.*t544;
t613 = rho1_4.*t544;
t614 = rho2_1.*t545;
t615 = rho2_2.*t545;
t616 = rho2_3.*t545;
t617 = rho2_4.*t545;
t618 = rho3_1.*t546;
t619 = rho3_2.*t546;
t620 = rho3_3.*t546;
t621 = rho3_4.*t546;
t622 = rho1_1.*t547;
t623 = rho1_2.*t547;
t624 = rho1_3.*t547;
t625 = rho1_4.*t547;
t626 = rho2_1.*t548;
t627 = rho2_2.*t548;
t628 = rho2_3.*t548;
t629 = rho2_4.*t548;
t630 = rho3_1.*t549;
t631 = rho3_2.*t549;
t632 = rho3_3.*t549;
t633 = rho3_4.*t549;
t643 = R0d1_1.*t636;
t644 = R0d1_2.*t635;
t645 = R0d1_3.*t634;
t646 = R0d2_1.*t636;
t647 = R0d2_2.*t635;
t648 = R0d2_3.*t634;
t649 = R0d3_1.*t636;
t650 = R0d3_2.*t635;
t651 = R0d3_3.*t634;
t652 = R0d1_1.*t639;
t653 = R0d1_2.*t638;
t654 = R0d1_3.*t637;
t655 = R0d2_1.*t639;
t656 = R0d2_2.*t638;
t657 = R0d2_3.*t637;
t658 = R0d3_1.*t639;
t659 = R0d3_2.*t638;
t660 = R0d3_3.*t637;
t661 = R0d1_1.*t642;
t662 = R0d1_2.*t641;
t663 = R0d1_3.*t640;
t664 = R0d2_1.*t642;
t665 = R0d2_2.*t641;
t666 = R0d2_3.*t640;
t667 = R0d3_1.*t642;
t668 = R0d3_2.*t641;
t669 = R0d3_3.*t640;
t670 = t550+t552+t586;
t671 = t553+t555+t587;
t672 = t556+t558+t588;
t673 = t559+t561+t589;
t674 = t562+t564+t590;
t675 = t565+t567+t591;
t676 = t568+t570+t592;
t677 = t571+t573+t593;
t678 = t574+t576+t594;
t679 = t577+t579+t595;
t680 = t580+t582+t596;
t681 = t583+t585+t597;
t838 = t235.*t361.*t835;
t839 = t235.*t362.*t835;
t840 = t235.*t363.*t835;
t841 = t235.*t364.*t835;
t842 = t235.*t369.*t835;
t843 = t235.*t371.*t835;
t844 = t235.*t373.*t835;
t845 = t235.*t375.*t835;
t846 = t235.*t381.*t835;
t847 = t235.*t383.*t835;
t848 = t235.*t385.*t835;
t849 = t235.*t387.*t835;
t850 = t234.*t365.*t836;
t851 = t234.*t366.*t836;
t852 = t234.*t367.*t836;
t853 = t234.*t368.*t836;
t854 = t234.*t377.*t836;
t855 = t234.*t378.*t836;
t856 = t234.*t379.*t836;
t857 = t234.*t380.*t836;
t858 = t234.*t389.*t836;
t859 = t234.*t390.*t836;
t860 = t234.*t391.*t836;
t861 = t234.*t392.*t836;
t862 = t233.*t370.*t837;
t863 = t233.*t372.*t837;
t864 = t233.*t374.*t837;
t865 = t233.*t376.*t837;
t866 = t233.*t382.*t837;
t867 = t233.*t384.*t837;
t868 = t233.*t386.*t837;
t869 = t233.*t388.*t837;
t870 = t233.*t393.*t837;
t871 = t233.*t394.*t837;
t872 = t233.*t395.*t837;
t873 = t233.*t396.*t837;
t682 = qid2_1.*t670;
t683 = qid2_2.*t671;
t684 = qid3_1.*t670;
t685 = qid2_3.*t672;
t686 = qid3_2.*t671;
t687 = qid2_4.*t673;
t688 = qid3_3.*t672;
t689 = qid3_4.*t673;
t690 = qid1_1.*t674;
t691 = qid1_2.*t675;
t692 = qid1_3.*t676;
t693 = qid3_1.*t674;
t694 = qid1_4.*t677;
t695 = qid3_2.*t675;
t696 = qid3_3.*t676;
t697 = qid3_4.*t677;
t698 = qid1_1.*t678;
t699 = qid1_2.*t679;
t700 = qid2_1.*t678;
t701 = qid1_3.*t680;
t702 = qid2_2.*t679;
t703 = qid1_4.*t681;
t704 = qid2_3.*t680;
t705 = qid2_4.*t681;
t718 = t643+t644+t645;
t719 = t646+t647+t648;
t720 = t649+t650+t651;
t721 = t652+t653+t654;
t722 = t655+t656+t657;
t723 = t658+t659+t660;
t724 = t661+t662+t663;
t725 = t664+t665+t666;
t726 = t667+t668+t669;
t970 = t457+t598+t602+t606+t838+t850+t862;
t971 = t457+t599+t603+t607+t839+t851+t863;
t972 = t457+t600+t604+t608+t840+t852+t864;
t973 = t457+t601+t605+t609+t841+t853+t865;
t974 = t458+t610+t614+t618+t842+t854+t866;
t975 = t458+t611+t615+t619+t843+t855+t867;
t976 = t458+t612+t616+t620+t844+t856+t868;
t977 = t458+t613+t617+t621+t845+t857+t869;
t978 = t459+t622+t626+t630+t846+t858+t870;
t979 = t459+t623+t627+t631+t847+t859+t871;
t980 = t459+t624+t628+t632+t848+t860+t872;
t981 = t459+t625+t629+t633+t849+t861+t873;
t706 = -t690;
t707 = -t691;
t708 = -t692;
t709 = -t694;
t710 = -t698;
t711 = -t699;
t712 = -t700;
t713 = -t701;
t714 = -t702;
t715 = -t703;
t716 = -t704;
t717 = -t705;
t727 = qid1_1.*t718;
t728 = qid1_2.*t718;
t729 = qid1_3.*t718;
t730 = qid1_4.*t718;
t731 = qid2_1.*t719;
t732 = qid2_2.*t719;
t733 = qid2_3.*t719;
t734 = qid2_4.*t719;
t735 = qid3_1.*t720;
t736 = qid3_2.*t720;
t737 = qid3_3.*t720;
t738 = qid3_4.*t720;
t739 = qid1_1.*t721;
t740 = qid1_2.*t721;
t741 = qid1_3.*t721;
t742 = qid1_4.*t721;
t743 = qid2_1.*t722;
t744 = qid2_2.*t722;
t745 = qid2_3.*t722;
t746 = qid2_4.*t722;
t747 = qid3_1.*t723;
t748 = qid3_2.*t723;
t749 = qid3_3.*t723;
t750 = qid3_4.*t723;
t751 = qid1_1.*t724;
t752 = qid1_2.*t724;
t753 = qid1_3.*t724;
t754 = qid1_4.*t724;
t755 = qid2_1.*t725;
t756 = qid2_2.*t725;
t757 = qid2_3.*t725;
t758 = qid2_4.*t725;
t759 = qid3_1.*t726;
t760 = qid3_2.*t726;
t761 = qid3_3.*t726;
t762 = qid3_4.*t726;
t763 = t682+t706;
t764 = t683+t707;
t765 = t685+t708;
t766 = t687+t709;
t767 = t684+t710;
t768 = t686+t711;
t769 = t688+t713;
t770 = t689+t715;
t771 = t693+t712;
t772 = t695+t714;
t773 = t696+t716;
t774 = t697+t717;
t874 = t727+t731+t735;
t875 = t728+t732+t736;
t876 = t729+t733+t737;
t877 = t730+t734+t738;
t878 = t739+t743+t747;
t879 = t740+t744+t748;
t880 = t741+t745+t749;
t881 = t742+t746+t750;
t882 = t751+t755+t759;
t883 = t752+t756+t760;
t884 = t753+t757+t761;
t885 = t754+t758+t762;
t775 = qi3_1.*t763;
t776 = qi3_2.*t764;
t777 = qi3_3.*t765;
t778 = qi2_1.*t767;
t779 = qi3_4.*t766;
t780 = qi2_2.*t768;
t781 = qi2_3.*t769;
t782 = qi1_1.*t771;
t783 = qi2_4.*t770;
t784 = qi1_2.*t772;
t785 = qi1_3.*t773;
t786 = qi1_4.*t774;
t823 = t322.*t763;
t824 = t323.*t764;
t825 = t324.*t765;
t826 = t325.*t766;
t827 = t326.*t767;
t828 = t327.*t768;
t829 = t328.*t769;
t830 = t329.*t770;
t831 = t330.*t771;
t832 = t331.*t772;
t833 = t332.*t773;
t834 = t333.*t774;
t886 = qid2_1.*t874;
t887 = qid2_2.*t875;
t888 = qid2_3.*t876;
t889 = qid3_1.*t874;
t890 = qid2_4.*t877;
t891 = qid3_2.*t875;
t892 = qid3_3.*t876;
t893 = qid3_4.*t877;
t894 = qid1_1.*t878;
t895 = qid1_2.*t879;
t896 = qid1_3.*t880;
t897 = qid1_4.*t881;
t898 = qid3_1.*t878;
t899 = qid3_2.*t879;
t900 = qid3_3.*t880;
t901 = qid3_4.*t881;
t902 = qid1_1.*t882;
t903 = qid1_2.*t883;
t904 = qid1_3.*t884;
t905 = qid2_1.*t882;
t906 = qid1_4.*t885;
t907 = qid2_2.*t883;
t908 = qid2_3.*t884;
t909 = qid2_4.*t885;
t787 = qi1_1.*t775;
t788 = qi2_1.*t775;
t789 = qi1_2.*t776;
t790 = qi2_2.*t776;
t791 = qi1_3.*t777;
t792 = qi2_3.*t777;
t793 = qi1_1.*t778;
t794 = qi1_4.*t779;
t795 = qi2_4.*t779;
t796 = qi3_1.*t778;
t797 = qi1_2.*t780;
t798 = qi3_2.*t780;
t799 = qi1_3.*t781;
t800 = qi3_3.*t781;
t801 = qi1_4.*t783;
ui = ft_2({kwi,li1,li2,li3,li4,mi1,mi2,mi3,mi4,mui1_1,mui1_2,mui1_3,mui1_4,mui2_1,mui2_2,mui2_3,mui2_4,mui3_1,mui3_2,mui3_3,mui3_4,qi1_1,qi1_2,qi1_3,qi1_4,qi2_1,qi2_2,qi2_3,qi2_4,qi3_1,qi3_2,qi3_3,qi3_4,t185,t186,t187,t188,t189,t190,t191,t192,t193,t194,t195,t196,t237,t238,t239,t240,t322,t323,t324,t325,t326,t327,t328,t329,t330,t331,t332,t333,t445,t446,t447,t448,t449,t450,t451,t452,t453,t454,t455,t456,t472,t473,t474,t475,t521,t522,t523,t524,t525,t526,t527,t528,t529,t530,t531,t532,t533,t534,t535,t536,t537,t538,t539,t540,t775,t776,t777,t778,t779,t780,t781,t782,t783,t784,t785,t786,t787,t788,t789,t790,t791,t792,t793,t794,t795,t796,t797,t798,t799,t800,t801,t823,t824,t825,t826,t827,t828,t829,t830,t831,t832,t833,t834,t886,t887,t888,t889,t890,t891,t892,t893,t894,t895,t896,t897,t898,t899,t900,t901,t902,t903,t904,t905,t906,t907,t908,t909,t970,t971,t972,t973,t974,t975,t976,t977,t978,t979,t980,t981,wi1_1,wi1_2,wi1_3,wi1_4,wi3_1,wi3_2,wi3_3,wi3_4});
end
function ui = ft_2(ct)
kwi = ct{1};
li1 = ct{2};
li2 = ct{3};
li3 = ct{4};
li4 = ct{5};
mi1 = ct{6};
mi2 = ct{7};
mi3 = ct{8};
mi4 = ct{9};
mui1_1 = ct{10};
mui1_2 = ct{11};
mui1_3 = ct{12};
mui1_4 = ct{13};
mui2_1 = ct{14};
mui2_2 = ct{15};
mui2_3 = ct{16};
mui2_4 = ct{17};
mui3_1 = ct{18};
mui3_2 = ct{19};
mui3_3 = ct{20};
mui3_4 = ct{21};
qi1_1 = ct{22};
qi1_2 = ct{23};
qi1_3 = ct{24};
qi1_4 = ct{25};
qi2_1 = ct{26};
qi2_2 = ct{27};
qi2_3 = ct{28};
qi2_4 = ct{29};
qi3_1 = ct{30};
qi3_2 = ct{31};
qi3_3 = ct{32};
qi3_4 = ct{33};
t185 = ct{34};
t186 = ct{35};
t187 = ct{36};
t188 = ct{37};
t189 = ct{38};
t190 = ct{39};
t191 = ct{40};
t192 = ct{41};
t193 = ct{42};
t194 = ct{43};
t195 = ct{44};
t196 = ct{45};
t237 = ct{46};
t238 = ct{47};
t239 = ct{48};
t240 = ct{49};
t322 = ct{50};
t323 = ct{51};
t324 = ct{52};
t325 = ct{53};
t326 = ct{54};
t327 = ct{55};
t328 = ct{56};
t329 = ct{57};
t330 = ct{58};
t331 = ct{59};
t332 = ct{60};
t333 = ct{61};
t445 = ct{62};
t446 = ct{63};
t447 = ct{64};
t448 = ct{65};
t449 = ct{66};
t450 = ct{67};
t451 = ct{68};
t452 = ct{69};
t453 = ct{70};
t454 = ct{71};
t455 = ct{72};
t456 = ct{73};
t472 = ct{74};
t473 = ct{75};
t474 = ct{76};
t475 = ct{77};
t521 = ct{78};
t522 = ct{79};
t523 = ct{80};
t524 = ct{81};
t525 = ct{82};
t526 = ct{83};
t527 = ct{84};
t528 = ct{85};
t529 = ct{86};
t530 = ct{87};
t531 = ct{88};
t532 = ct{89};
t533 = ct{90};
t534 = ct{91};
t535 = ct{92};
t536 = ct{93};
t537 = ct{94};
t538 = ct{95};
t539 = ct{96};
t540 = ct{97};
t775 = ct{98};
t776 = ct{99};
t777 = ct{100};
t778 = ct{101};
t779 = ct{102};
t780 = ct{103};
t781 = ct{104};
t782 = ct{105};
t783 = ct{106};
t784 = ct{107};
t785 = ct{108};
t786 = ct{109};
t787 = ct{110};
t788 = ct{111};
t789 = ct{112};
t790 = ct{113};
t791 = ct{114};
t792 = ct{115};
t793 = ct{116};
t794 = ct{117};
t795 = ct{118};
t796 = ct{119};
t797 = ct{120};
t798 = ct{121};
t799 = ct{122};
t800 = ct{123};
t801 = ct{124};
t823 = ct{125};
t824 = ct{126};
t825 = ct{127};
t826 = ct{128};
t827 = ct{129};
t828 = ct{130};
t829 = ct{131};
t830 = ct{132};
t831 = ct{133};
t832 = ct{134};
t833 = ct{135};
t834 = ct{136};
t886 = ct{137};
t887 = ct{138};
t888 = ct{139};
t889 = ct{140};
t890 = ct{141};
t891 = ct{142};
t892 = ct{143};
t893 = ct{144};
t894 = ct{145};
t895 = ct{146};
t896 = ct{147};
t897 = ct{148};
t898 = ct{149};
t899 = ct{150};
t900 = ct{151};
t901 = ct{152};
t902 = ct{153};
t903 = ct{154};
t904 = ct{155};
t905 = ct{156};
t906 = ct{157};
t907 = ct{158};
t908 = ct{159};
t909 = ct{160};
t970 = ct{161};
t971 = ct{162};
t972 = ct{163};
t973 = ct{164};
t974 = ct{165};
t975 = ct{166};
t976 = ct{167};
t977 = ct{168};
t978 = ct{169};
t979 = ct{170};
t980 = ct{171};
t981 = ct{172};
wi1_1 = ct{173};
wi1_2 = ct{174};
wi1_3 = ct{175};
wi1_4 = ct{176};
wi3_1 = ct{177};
wi3_2 = ct{178};
wi3_3 = ct{179};
wi3_4 = ct{180};
t802 = qi2_1.*t782;
t803 = qi3_1.*t782;
t804 = qi3_4.*t783;
t805 = qi2_2.*t784;
t806 = qi3_2.*t784;
t807 = qi2_3.*t785;
t808 = qi3_3.*t785;
t809 = qi2_4.*t786;
t810 = qi3_4.*t786;
t811 = -t778;
t812 = -t780;
t813 = -t781;
t814 = -t783;
t910 = -t894;
t911 = -t895;
t912 = -t896;
t913 = -t897;
t914 = -t902;
t915 = -t903;
t916 = -t904;
t917 = -t905;
t918 = -t906;
t919 = -t907;
t920 = -t908;
t921 = -t909;
t815 = -t787;
t816 = -t789;
t817 = -t791;
t818 = -t794;
t819 = -t803;
t820 = -t806;
t821 = -t808;
t822 = -t810;
t922 = t775+t782+t811;
t923 = t776+t784+t812;
t924 = t777+t785+t813;
t925 = t779+t786+t814;
t934 = t237+t788+t802+t827;
t935 = t238+t790+t805+t828;
t936 = t239+t792+t807+t829;
t937 = t240+t795+t809+t830;
t982 = t886+t910;
t983 = t887+t911;
t984 = t888+t912;
t985 = t890+t913;
t986 = t889+t914;
t987 = t891+t915;
t988 = t892+t916;
t989 = t893+t918;
t990 = t898+t917;
t991 = t899+t919;
t992 = t900+t920;
t993 = t901+t921;
t926 = t793+t815+t831+wi1_1;
t927 = t796+t819+t823+wi3_1;
t928 = t797+t816+t832+wi1_2;
t929 = t798+t820+t824+wi3_2;
t930 = t799+t817+t833+wi1_3;
t931 = t800+t821+t825+wi3_3;
t932 = t801+t818+t834+wi1_4;
t933 = t804+t822+t826+wi3_4;
t938 = t445.*t922;
t939 = t449.*t922;
t940 = t453.*t922;
t941 = t446.*t923;
t942 = t450.*t923;
t943 = t454.*t923;
t944 = t447.*t924;
t945 = t451.*t924;
t946 = t455.*t924;
t947 = t448.*t925;
t948 = t452.*t925;
t949 = t456.*t925;
t958 = kwi.*t934;
t959 = kwi.*t935;
t960 = kwi.*t936;
t961 = kwi.*t937;
t994 = qi1_1.*qi3_1.*t982;
t995 = qi1_2.*qi3_2.*t983;
t996 = qi2_1.*qi3_1.*t982;
t997 = qi1_3.*qi3_3.*t984;
t998 = qi2_2.*qi3_2.*t983;
t999 = qi1_4.*qi3_4.*t985;
t1000 = qi2_3.*qi3_3.*t984;
t1001 = qi2_4.*qi3_4.*t985;
t1002 = qi1_1.*qi2_1.*t986;
t1003 = qi1_2.*qi2_2.*t987;
t1004 = qi1_3.*qi2_3.*t988;
t1005 = qi2_1.*qi3_1.*t986;
t1006 = qi1_4.*qi2_4.*t989;
t1007 = qi2_2.*qi3_2.*t987;
t1008 = qi2_3.*qi3_3.*t988;
t1009 = qi2_4.*qi3_4.*t989;
t1010 = qi1_1.*qi2_1.*t990;
t1011 = qi1_1.*qi3_1.*t990;
t1012 = qi1_2.*qi2_2.*t991;
t1013 = qi1_2.*qi3_2.*t991;
t1014 = qi1_3.*qi2_3.*t992;
t1015 = qi1_3.*qi3_3.*t992;
t1016 = qi1_4.*qi2_4.*t993;
t1017 = qi1_4.*qi3_4.*t993;
t1026 = t322.*t982;
t1027 = t323.*t983;
t1028 = t324.*t984;
t1029 = t325.*t985;
t1030 = t326.*t986;
t1031 = t327.*t987;
t1032 = t328.*t988;
t1033 = t329.*t989;
t1034 = t330.*t990;
t1035 = t331.*t991;
t1036 = t332.*t992;
t1037 = t333.*t993;
t950 = kwi.*t926;
t951 = kwi.*t927;
t952 = kwi.*t928;
t953 = kwi.*t929;
t954 = kwi.*t930;
t955 = kwi.*t931;
t956 = kwi.*t932;
t957 = kwi.*t933;
t962 = -t939;
t963 = -t942;
t964 = -t945;
t965 = -t948;
t966 = -t958;
t967 = -t959;
t968 = -t960;
t969 = -t961;
t1018 = -t1002;
t1019 = -t1003;
t1020 = -t1004;
t1021 = -t1005;
t1022 = -t1006;
t1023 = -t1007;
t1024 = -t1008;
t1025 = -t1009;
t1038 = -t1026;
t1039 = -t1027;
t1040 = -t1028;
t1041 = -t1029;
t1042 = -t1034;
t1043 = -t1035;
t1044 = -t1036;
t1045 = -t1037;
t1046 = t525+t962+t966+t996+t1010+t1030;
t1047 = t526+t963+t967+t998+t1012+t1031;
t1048 = t527+t964+t968+t1000+t1014+t1032;
t1049 = t528+t965+t969+t1001+t1016+t1033;
t1050 = t533+t938+t951+t1011+t1021+t1038;
t1051 = t537+t940+t950+t994+t1018+t1042;
t1052 = t534+t941+t953+t1013+t1023+t1039;
t1053 = t538+t943+t952+t995+t1019+t1043;
t1054 = t535+t944+t955+t1015+t1024+t1040;
t1055 = t539+t946+t954+t997+t1020+t1044;
t1056 = t536+t947+t957+t1017+t1025+t1041;
t1057 = t540+t949+t956+t999+t1022+t1045;
mt1 = [mui1_1+mi1.*t185.*t970+mi1.*t330.*t970+li1.*mi1.*qi1_1.*t472+li1.*mi1.*qi3_1.*t1046+li1.*mi1.*qi2_1.*(t521-t938-t951+t1005-t1011+t1026),mui2_1+mi1.*t189.*t974+mi1.*t326.*t974+li1.*mi1.*qi2_1.*t472-li1.*mi1.*qi1_1.*(t521-t938-t951+t1005-t1011+t1026)+li1.*mi1.*qi3_1.*(t529-t940-t950-t994+t1002+t1034),mui3_1+mi1.*t193.*t978+mi1.*t322.*t978+li1.*mi1.*qi3_1.*t472-li1.*mi1.*qi1_1.*t1046-li1.*mi1.*qi2_1.*(t529-t940-t950-t994+t1002+t1034),mui1_2+mi2.*t186.*t971+mi2.*t331.*t971+li2.*mi2.*qi1_2.*t473+li2.*mi2.*qi3_2.*t1047+li2.*mi2.*qi2_2.*(t522-t941-t953+t1007-t1013+t1027)];
mt2 = [mui2_2+mi2.*t190.*t975+mi2.*t327.*t975+li2.*mi2.*qi2_2.*t473-li2.*mi2.*qi1_2.*(t522-t941-t953+t1007-t1013+t1027)+li2.*mi2.*qi3_2.*(t530-t943-t952-t995+t1003+t1035),mui3_2+mi2.*t194.*t979+mi2.*t323.*t979+li2.*mi2.*qi3_2.*t473-li2.*mi2.*qi1_2.*t1047-li2.*mi2.*qi2_2.*(t530-t943-t952-t995+t1003+t1035),mui1_3+mi3.*t187.*t972+mi3.*t332.*t972+li3.*mi3.*qi1_3.*t474+li3.*mi3.*qi3_3.*t1048+li3.*mi3.*qi2_3.*(t523-t944-t955+t1008-t1015+t1028),mui2_3+mi3.*t191.*t976+mi3.*t328.*t976+li3.*mi3.*qi2_3.*t474-li3.*mi3.*qi1_3.*(t523-t944-t955+t1008-t1015+t1028)+li3.*mi3.*qi3_3.*(t531-t946-t954-t997+t1004+t1036)];
mt3 = [mui3_3+mi3.*t195.*t980+mi3.*t324.*t980+li3.*mi3.*qi3_3.*t474-li3.*mi3.*qi1_3.*t1048-li3.*mi3.*qi2_3.*(t531-t946-t954-t997+t1004+t1036),mui1_4+mi4.*t188.*t973+mi4.*t333.*t973+li4.*mi4.*qi1_4.*t475+li4.*mi4.*qi3_4.*t1049+li4.*mi4.*qi2_4.*(t524-t947-t957+t1009-t1017+t1029),mui2_4+mi4.*t192.*t977+mi4.*t329.*t977+li4.*mi4.*qi2_4.*t475-li4.*mi4.*qi1_4.*(t524-t947-t957+t1009-t1017+t1029)+li4.*mi4.*qi3_4.*(t532-t949-t956-t999+t1006+t1037),mui3_4+mi4.*t196.*t981+mi4.*t325.*t981+li4.*mi4.*qi3_4.*t475-li4.*mi4.*qi1_4.*t1049-li4.*mi4.*qi2_4.*(t532-t949-t956-t999+t1006+t1037)];
ui = reshape([mt1,mt2,mt3],3,4);
end
