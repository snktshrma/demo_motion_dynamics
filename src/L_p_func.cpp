#include "demo_motion_dynamics/L_p_func.h"
#include <cmath>

// Function Definitions
//
// L_p_func
//     L_p = L_p_func(DELTA_1,DELTA_2,DELTA_3,DELTA_4)
//
// Arguments    : double delta_1
//                double delta_2
//                double delta_3
//                double delta_4
//                double L_p[12]
// Return Type  : void
//
void L_p_func(double delta_1, double delta_2, double delta_3, double delta_4,
              double L_p[12])
{
  double b_t210_tmp;
  double b_t211_tmp;
  double b_t215_tmp;
  double b_t92_tmp;
  double b_t94_tmp;
  double c_t210_tmp;
  double c_t211_tmp;
  double c_t215_tmp;
  double d_t210_tmp;
  double d_t211_tmp;
  double d_t215_tmp;
  double e_t211_tmp;
  double t10;
  double t107;
  double t107_tmp;
  double t108;
  double t108_tmp;
  double t109;
  double t109_tmp;
  double t11;
  double t110;
  double t110_tmp;
  double t111_tmp;
  double t112_tmp;
  double t12;
  double t13;
  double t130;
  double t131;
  double t132;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t18_tmp;
  double t19;
  double t19_tmp;
  double t2;
  double t20;
  double t20_tmp;
  double t21;
  double t210;
  double t210_tmp;
  double t211;
  double t211_tmp;
  double t212;
  double t212_tmp;
  double t213;
  double t215;
  double t215_tmp;
  double t217;
  double t21_tmp;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t76_tmp;
  double t78_tmp;
  double t8;
  double t80_tmp;
  double t82_tmp;
  double t84;
  double t85;
  double t86;
  double t87;
  double t9;
  double t92_tmp;
  double t94_tmp;

  t2 = std::cos(delta_1);
  t3 = std::cos(delta_2);
  t4 = std::cos(delta_3);
  t5 = std::cos(delta_4);
  t6 = std::sin(delta_1);
  t7 = std::sin(delta_2);
  t8 = std::sin(delta_3);
  t9 = std::sin(delta_4);
  t10 = t2 * t2;
  t11 = t3 * t3;
  t12 = t4 * t4;
  t13 = t5 * t5;
  t14 = t6 * t6;
  t15 = t7 * t7;
  t16 = t8 * t8;
  t17 = t9 * t9;
  t107_tmp = t2 * t3 * t6 * t7;
  t107 = t107_tmp * 8.2413594426779614E+44;
  t108_tmp = t2 * t5 * t6 * t9;
  t108 = t108_tmp * 8.2413594426779614E+44;
  t109_tmp = t3 * t4 * t7 * t8;
  t109 = t109_tmp * 8.2413594426779614E+44;
  t110_tmp = t4 * t5 * t8 * t9;
  t110 = t110_tmp * 8.2413594426779614E+44;
  t18_tmp = t10 * t11;
  t18 = t18_tmp * 4.2266723167125741E+29;
  t19_tmp = t10 * t13;
  t19 = t19_tmp * 4.2266723167125741E+29;
  t20_tmp = t11 * t12;
  t20 = t20_tmp * 4.2266723167125741E+29;
  t21_tmp = t12 * t13;
  t21 = t21_tmp * 4.2266723167125741E+29;
  t217 = t2 * t6;
  t76_tmp = t217 * t11;
  t78_tmp = t217 * t13;
  t213 = t4 * t8;
  t80_tmp = t213 * t11;
  t82_tmp = t213 * t13;
  t84 = t18_tmp * 2.7478806701289172E+44;
  t85 = t19_tmp * 2.7478806701289172E+44;
  t86 = t20_tmp * 2.7478806701289172E+44;
  t87 = t21_tmp * 2.7478806701289172E+44;
  t92_tmp = t3 * t7;
  b_t92_tmp = t92_tmp * t13;
  t94_tmp = t5 * t9;
  b_t94_tmp = t94_tmp * t11;
  t111_tmp = t10 * t16;
  t112_tmp = t12 * t14;
  t130 = t18_tmp * 1.7864758872864441E+59;
  t131 = t19_tmp * 1.7864758872864441E+59;
  t132 = t20_tmp * 1.7864758872864441E+59;
  t215_tmp = t2 * t4 * t6 * t8;
  b_t215_tmp = t3 * t5 * t7 * t9;
  c_t215_tmp = t11 * t17;
  d_t215_tmp = t13 * t15;
  t215 = ((((((((((((((((t111_tmp * 5.3579436992487363E+59 +
                         t112_tmp * 5.3579436992487363E+59) +
                        c_t215_tmp * 5.3579436992487363E+59) +
                       d_t215_tmp * 5.3579436992487363E+59) +
                      t130) +
                     t131) +
                    t132) +
                   t21_tmp * 1.7864758872864441E+59) +
                  t107_tmp * 1.0715887398497471E+60) +
                 t108_tmp * 1.0715887398497471E+60) +
                t109_tmp * 1.0715887398497471E+60) +
               t110_tmp * 1.0715887398497471E+60) +
              t14 * t15 * 1.60693804425899E+60) +
             t14 * t17 * 1.60693804425899E+60) +
            t15 * t16 * 1.60693804425899E+60) +
           t16 * t17 * 1.60693804425899E+60) -
          t215_tmp * 1.0715887398497471E+60) -
         b_t215_tmp * 1.0715887398497471E+60;
  t210_tmp = ((t18 + t19) + t20) + t21;
  b_t210_tmp = t11 * t14;
  c_t210_tmp = t13 * t14;
  d_t210_tmp = t11 * t16;
  t210 = (((((((((((t210_tmp + t78_tmp * 1.463959521195683E+30) +
                   t80_tmp * 1.463959521195683E+30) +
                  t215_tmp * 2.5353012004564588E+30) +
                 t11 * t13 * 1.69066892668503E+30) +
                b_t210_tmp * 1.267650600228229E+30) +
               t111_tmp * 1.267650600228229E+30) +
              t112_tmp * 1.267650600228229E+30) +
             d_t210_tmp * 1.267650600228229E+30) +
            c_t210_tmp * 1.267650600228229E+30) +
           t13 * t16 * 1.267650600228229E+30) -
          t76_tmp * 1.463959521195683E+30) -
         t82_tmp * 1.463959521195683E+30;
  t211_tmp = t92_tmp * t10;
  b_t211_tmp = t92_tmp * t12;
  c_t211_tmp = t94_tmp * t10;
  d_t211_tmp = t94_tmp * t12;
  e_t211_tmp = t10 * t15;
  t211 = (((((((((((t210_tmp + t211_tmp * 1.463959521195683E+30) +
                   d_t211_tmp * 1.463959521195683E+30) +
                  b_t215_tmp * 2.5353012004564588E+30) +
                 t10 * t12 * 1.69066892668503E+30) +
                e_t211_tmp * 1.267650600228229E+30) +
               t10 * t17 * 1.267650600228229E+30) +
              t12 * t15 * 1.267650600228229E+30) +
             c_t215_tmp * 1.267650600228229E+30) +
            d_t215_tmp * 1.267650600228229E+30) +
           t12 * t17 * 1.267650600228229E+30) -
          b_t211_tmp * 1.463959521195683E+30) -
         c_t211_tmp * 1.463959521195683E+30;
  t210_tmp = t217 * t12;
  t212_tmp = t213 * t10;
  t212 = ((((((((((((((((((t18 + t21) + t211_tmp * 7.3197976059784166E+29) +
                         b_t211_tmp * 7.3197976059784166E+29) +
                        b_t92_tmp * 1.463959521195683E+30) +
                       c_t211_tmp * 7.3197976059784166E+29) +
                      b_t94_tmp * 1.463959521195683E+30) +
                     d_t211_tmp * 7.3197976059784166E+29) -
                    t19) -
                   t20) +
                  t108_tmp * 1.267650600228229E+30) +
                 t109_tmp * 1.267650600228229E+30) -
                t76_tmp * 7.3197976059784166E+29) -
               t210_tmp * 1.463959521195683E+30) -
              t78_tmp * 7.3197976059784166E+29) -
             t212_tmp * 1.463959521195683E+30) -
            t80_tmp * 7.3197976059784166E+29) -
           t82_tmp * 7.3197976059784166E+29) -
          t107_tmp * 1.267650600228229E+30) -
         t110_tmp * 1.267650600228229E+30;
  t213 = ((((((((((((((((((t211_tmp * 4.7588099676409849E+44 +
                           c_t211_tmp * 4.7588099676409849E+44) +
                          t84) +
                         t86) +
                        t210_tmp * 9.5176199352819714E+44) -
                       b_t211_tmp * 4.7588099676409849E+44) -
                      d_t211_tmp * 4.7588099676409849E+44) +
                     t217 * t15 * 1.42724769270596E+45) +
                    t217 * t17 * 1.42724769270596E+45) -
                   t85) -
                  t87) +
                 t107) +
                t109) +
               c_t215_tmp * 8.2413594426779614E+44) -
              t212_tmp * 9.5176199352819714E+44) -
             t213 * t15 * 1.42724769270596E+45) -
            t213 * t17 * 1.42724769270596E+45) -
           t108) -
          t110) -
         d_t215_tmp * 8.2413594426779614E+44;
    t217 = 1.0 / (((((((((((((((((((((((((((((((((((((((((((((((((((t12 * t130 + t13 * t130) + t12 * t131) + t13 * t132) + t210_tmp * t13 * 3.093838590512795E+59) + t211_tmp * t13 * 3.093838590512795E+59) + t212_tmp * t11 * 3.093838590512795E+59) + b_t94_tmp * t12 * 3.093838590512795E+59) + t78_tmp * t15 * 4.6394729143838482E+59) + t211_tmp * t16 * 4.6394729143838482E+59) + t80_tmp * t17 * 4.6394729143838482E+59) + d_t211_tmp * t14 * 4.6394729143838482E+59) + t107_tmp * t12 * 5.3579436992487363E+59) + t107_tmp * t13 * 5.3579436992487363E+59) + t109_tmp * t10 * 5.3579436992487363E+59) + t108_tmp * t11 * 5.3579436992487363E+59) + t108_tmp * t12 * 5.3579436992487363E+59) + t109_tmp * t13 * 5.3579436992487363E+59) + t110_tmp * t10 * 5.3579436992487363E+59) + t110_tmp * t11 * 5.3579436992487363E+59) + t18_tmp * t16 * 1.3394859248121841E+59) + t20_tmp * t14 * 1.3394859248121841E+59) + t18_tmp * t17 * 1.3394859248121841E+59) + t19_tmp * t15 * 1.3394859248121841E+59) + t19_tmp * t16 * 1.3394859248121841E+59) + t21_tmp * t14 * 1.3394859248121841E+59) + t20_tmp * t17 * 1.3394859248121841E+59) + t21_tmp * t15 * 1.3394859248121841E+59) + e_t211_tmp * t16 * 4.0173451106474757E+59) + t112_tmp * t15 * 4.0173451106474757E+59) + b_t210_tmp * t17 * 4.0173451106474757E+59) + c_t210_tmp * t15 * 4.0173451106474757E+59) + t111_tmp * t17 * 4.0173451106474757E+59) + t112_tmp * t17 * 4.0173451106474757E+59) + d_t210_tmp * t17 * 4.0173451106474757E+59) + d_t215_tmp * t16 * 4.0173451106474757E+59) - t76_tmp * t12 * 3.093838590512795E+59) - b_t211_tmp * t13 * 3.093838590512795E+59) - t212_tmp * t13 * 3.093838590512795E+59) - c_t211_tmp * t11 * 3.093838590512795E+59) - t76_tmp * t17 * 4.6394729143838482E+59) - b_t211_tmp * t14 * 4.6394729143838482E+59) - t82_tmp * t15 * 4.6394729143838482E+59) - c_t211_tmp * t16 * 4.6394729143838482E+59) - t215_tmp * t11 * 2.6789718496243682E+59) - t215_tmp * t13 * 2.6789718496243682E+59) - b_t215_tmp * t10 * 2.6789718496243682E+59) - b_t215_tmp * t12 * 2.6789718496243682E+59) + t215_tmp * t15 * 8.0346902212949514E+59) + t215_tmp * t17 * 8.0346902212949514E+59) + b_t215_tmp * t14 * 8.0346902212949514E+59) + b_t215_tmp * t16 * 8.0346902212949514E+59);
    t19 = ((((((((((((((((((t76_tmp * 4.7588099676409849E+44 -
                            t78_tmp * 4.7588099676409849E+44) +
                           t80_tmp * 4.7588099676409849E+44) -
                          t82_tmp * 4.7588099676409849E+44) -
                         t84) +
                        t86) +
                       t87) +
                      b_t92_tmp * 9.5176199352819714E+44) -
                     b_t94_tmp * 9.5176199352819714E+44) +
                    t92_tmp * t14 * 1.42724769270596E+45) +
                   t92_tmp * t16 * 1.42724769270596E+45) -
                  t94_tmp * t14 * 1.42724769270596E+45) -
                 t94_tmp * t16 * 1.42724769270596E+45) -
                t85) -
               t107) +
              t109) +
             t110) -
            t111_tmp * 8.2413594426779614E+44) +
           t112_tmp * 8.2413594426779614E+44) -
          t108;
    t20 = t2 * t217 * t19;
    L_p[0] = (t20 * -2.81474976710656E+13 -
              t2 * t210 * t217 * 1.829949401494604E+28) -
             t6 * t212 * t217 * 3.1691265005705741E+28;
    L_p[1] = (t3 * t217 * t19 * -2.81474976710656E+13 -
              t3 * t212 * t217 * 1.829949401494604E+28) +
             t7 * t210 * t217 * 3.1691265005705741E+28;
    t21 = t4 * t217 * t19;
    L_p[2] = (t21 * -2.81474976710656E+13 +
              t4 * t210 * t217 * 1.829949401494604E+28) +
             t8 * t212 * t217 * 3.1691265005705741E+28;
    L_p[3] = (t5 * t217 * t19 * -2.81474976710656E+13 +
              t5 * t212 * t217 * 1.829949401494604E+28) -
             t9 * t210 * t217 * 3.1691265005705741E+28;
    L_p[4] = (t2 * t212 * t217 * -1.829949401494604E+28 +
              t2 * t213 * t217 * 2.81474976710656E+13) -
             t6 * t211 * t217 * 3.1691265005705741E+28;
    t210_tmp = t3 * t213 * t217;
    L_p[5] = (t3 * t211 * t217 * -1.829949401494604E+28 +
              t210_tmp * 2.81474976710656E+13) +
             t7 * t212 * t217 * 3.1691265005705741E+28;
    L_p[6] = (t4 * t212 * t217 * 1.829949401494604E+28 +
              t4 * t213 * t217 * 2.81474976710656E+13) +
             t8 * t211 * t217 * 3.1691265005705741E+28;
    t18 = t5 * t213 * t217;
    L_p[7] = (t5 * t211 * t217 * 1.829949401494604E+28 +
              t18 * 2.81474976710656E+13) -
             t9 * t212 * t217 * 3.1691265005705741E+28;
    L_p[8] = (t20 * 1.9907421004390531E+13 +
              t2 * t215 * t217 * 0.030620742076275222) -
             t6 * t213 * t217 * 3.4475890651130281E+13;
    L_p[9] = (t7 * t217 * t19 * -3.4475890651130281E+13 -
              t210_tmp * 1.9907421004390531E+13) +
             t3 * t215 * t217 * 0.030620742076275222;
    L_p[10] = (t21 * -1.9907421004390531E+13 +
               t4 * t215 * t217 * 0.030620742076275222) +
              t8 * t213 * t217 * 3.4475890651130281E+13;
    L_p[11] = (t9 * t217 * t19 * 3.4475890651130281E+13 +
               t18 * 1.9907421004390531E+13) +
              t5 * t215 * t217 * 0.030620742076275222;
}
