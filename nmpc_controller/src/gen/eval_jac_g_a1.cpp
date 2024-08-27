/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
#define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
#define _CASADI_NAMESPACE_CONCAT(NS, ID) NS##ID
#define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
#define CASADI_PREFIX(ID) eval_jac_g_a1_##ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
#if defined(STATIC_LINKED)
#define CASADI_SYMBOL_EXPORT
#else
#define CASADI_SYMBOL_EXPORT __declspec(dllexport)
#endif
#elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
#define CASADI_SYMBOL_EXPORT __attribute__((visibility("default")))
#else
#define CASADI_SYMBOL_EXPORT
#endif
#endif

casadi_real casadi_sq(casadi_real x) { return x * x; }

static const casadi_int casadi_s0[40] = {
    36, 1,  0,  36, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
static const casadi_int casadi_s1[18] = {14, 1, 0, 14, 0, 1,  2,  3,  4,
                                         5,  6, 7, 8,  9, 10, 11, 12, 13};
static const casadi_int casadi_s2[171] = {
    28,  36,  0,   1,   2,   3,   8,   13,  16,  18,  20,  22,  26,  32,  38,
    44,  50,  57,  63,  69,  76,  82,  88,  95,  101, 107, 114, 115, 116, 117,
    118, 119, 120, 121, 122, 123, 126, 129, 132, 0,   1,   2,   3,   4,   5,
    10,  11,  3,   4,   5,   9,   11,  5,   9,   10,  0,   6,   1,   7,   2,
    8,   3,   9,   10,  11,  3,   4,   5,   9,   10,  11,  3,   4,   5,   9,
    10,  11,  6,   9,   10,  11,  12,  13,  7,   9,   10,  11,  14,  15,  8,
    9,   10,  12,  13,  14,  15,  6,   9,   10,  11,  16,  17,  7,   9,   10,
    11,  18,  19,  8,   9,   10,  16,  17,  18,  19,  6,   9,   10,  11,  20,
    21,  7,   9,   10,  11,  22,  23,  8,   9,   10,  20,  21,  22,  23,  6,
    9,   10,  11,  24,  25,  7,   9,   10,  11,  26,  27,  8,   9,   10,  24,
    25,  26,  27,  0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,
    9,   10,  11,  9,   10,  11};

/* eval_jac_g_a1:(w[36],p[14])->(jac_g[28x36,132nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw,
                     casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108,
      a109, a11, a110, a111, a112, a113, a12, a13, a14, a15, a16, a17, a18, a19,
      a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32,
      a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46,
      a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6,
      a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73,
      a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87,
      a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0 = -1.;
  if (res[0] != 0) res[0][0] = a0;
  if (res[0] != 0) res[0][1] = a0;
  if (res[0] != 0) res[0][2] = a0;
  a1 = arg[0] ? arg[0][3] : 0;
  a2 = sin(a1);
  a3 = arg[0] ? arg[0][11] : 0;
  a4 = arg[0] ? arg[0][4] : 0;
  a5 = sin(a4);
  a6 = arg[1] ? arg[1][0] : 0;
  a7 = cos(a4);
  a8 = (a6 / a7);
  a9 = (a5 * a8);
  a10 = (a3 * a9);
  a10 = (a2 * a10);
  a11 = cos(a1);
  a12 = arg[0] ? arg[0][10] : 0;
  a13 = (a5 * a8);
  a14 = (a12 * a13);
  a14 = (a11 * a14);
  a10 = (a10 - a14);
  a10 = (a10 + a0);
  if (res[0] != 0) res[0][3] = a10;
  a10 = (a3 * a6);
  a10 = (a11 * a10);
  a14 = (a12 * a6);
  a14 = (a2 * a14);
  a10 = (a10 + a14);
  if (res[0] != 0) res[0][4] = a10;
  a10 = (a6 / a7);
  a14 = (a3 * a10);
  a14 = (a2 * a14);
  a15 = (a12 * a10);
  a15 = (a11 * a15);
  a14 = (a14 - a15);
  if (res[0] != 0) res[0][5] = a14;
  a14 = arg[0] ? arg[0][9] : 0;
  a15 = (a14 * a3);
  a16 = 2.7500000000000001e-005;
  a17 = (a16 * a6);
  a15 = (a15 * a17);
  a18 = (a12 * a3);
  a19 = 6.1099999999999994e-005;
  a20 = (a19 * a6);
  a18 = (a18 * a20);
  a15 = (a15 - a18);
  a18 = (a14 * a12);
  a21 = 2.5702007500000001e-001;
  a22 = (a21 * a6);
  a18 = (a18 * a22);
  a15 = (a15 - a18);
  a18 = casadi_sq(a12);
  a23 = 3.6600000000000002e-005;
  a24 = (a23 * a6);
  a25 = (a18 * a24);
  a15 = (a15 - a25);
  a25 = casadi_sq(a14);
  a26 = (a23 * a6);
  a27 = (a25 * a26);
  a15 = (a15 + a27);
  a27 = 3.1492299499999998e-001;
  a28 = arg[0] ? arg[0][35] : 0;
  a28 = (a28 - a3);
  a29 = (a27 * a28);
  a15 = (a15 - a29);
  a29 = arg[0] ? arg[0][34] : 0;
  a29 = (a29 - a12);
  a30 = (a16 * a29);
  a15 = (a15 + a30);
  a30 = arg[0] ? arg[0][33] : 0;
  a30 = (a30 - a14);
  a31 = (a19 * a30);
  a15 = (a15 + a31);
  a15 = (a11 * a15);
  a31 = (a14 * a12);
  a32 = (a16 * a6);
  a31 = (a31 * a32);
  a33 = (a12 * a3);
  a34 = (a23 * a6);
  a33 = (a33 * a34);
  a35 = (a14 * a3);
  a36 = 2.8197203500000001e-001;
  a37 = (a36 * a6);
  a35 = (a35 * a37);
  a33 = (a33 + a35);
  a31 = (a31 - a33);
  a33 = casadi_sq(a3);
  a35 = (a19 * a6);
  a38 = (a33 * a35);
  a31 = (a31 - a38);
  a38 = (a19 * a6);
  a39 = (a25 * a38);
  a31 = (a31 + a39);
  a39 = -2.7500000000000001e-005;
  a40 = (a39 * a28);
  a31 = (a31 + a40);
  a40 = 2.8997103499999999e-001;
  a41 = (a40 * a29);
  a31 = (a31 + a41);
  a41 = -3.6600000000000002e-005;
  a42 = (a41 * a30);
  a31 = (a31 + a42);
  a31 = (a2 * a31);
  a15 = (a15 - a31);
  if (res[0] != 0) res[0][6] = a15;
  a15 = (a14 * a12);
  a31 = (a15 * a7);
  a42 = (a16 * a6);
  a31 = (a31 * a42);
  a43 = (a12 * a3);
  a44 = (a43 * a7);
  a45 = (a23 * a6);
  a44 = (a44 * a45);
  a46 = (a14 * a3);
  a47 = (a46 * a7);
  a36 = (a36 * a6);
  a47 = (a47 * a36);
  a44 = (a44 + a47);
  a31 = (a31 - a44);
  a44 = (a19 * a6);
  a47 = (a33 * a44);
  a48 = (a7 * a47);
  a31 = (a31 - a48);
  a48 = (a19 * a6);
  a49 = (a25 * a48);
  a50 = (a7 * a49);
  a31 = (a31 + a50);
  a50 = (a16 * a28);
  a51 = (a7 * a50);
  a31 = (a31 - a51);
  a51 = (a40 * a29);
  a52 = (a7 * a51);
  a31 = (a31 + a52);
  a52 = (a23 * a30);
  a53 = (a7 * a52);
  a31 = (a31 - a53);
  a11 = (a11 * a31);
  a31 = (a12 * a3);
  a53 = (a19 * a6);
  a54 = (a7 * a53);
  a55 = (a31 * a54);
  a56 = (a14 * a3);
  a57 = (a16 * a6);
  a58 = (a7 * a57);
  a59 = (a56 * a58);
  a55 = (a55 - a59);
  a59 = (a14 * a12);
  a21 = (a21 * a6);
  a60 = (a7 * a21);
  a61 = (a59 * a60);
  a55 = (a55 + a61);
  a61 = (a23 * a6);
  a62 = (a18 * a61);
  a63 = (a7 * a62);
  a55 = (a55 + a63);
  a63 = (a23 * a6);
  a25 = (a25 * a63);
  a64 = (a7 * a25);
  a55 = (a55 - a64);
  a64 = (a27 * a28);
  a65 = (a7 * a64);
  a55 = (a55 + a65);
  a65 = (a16 * a29);
  a66 = (a7 * a65);
  a55 = (a55 - a66);
  a66 = (a19 * a30);
  a67 = (a7 * a66);
  a55 = (a55 - a67);
  a2 = (a2 * a55);
  a11 = (a11 - a2);
  if (res[0] != 0) res[0][7] = a11;
  a11 = cos(a4);
  a2 = sin(a1);
  a55 = (a12 * a2);
  a67 = (a55 * a8);
  a1 = cos(a1);
  a68 = (a3 * a1);
  a69 = (a68 * a8);
  a67 = (a67 + a69);
  a67 = (a11 * a67);
  a4 = sin(a4);
  a69 = (a14 * a7);
  a68 = (a68 * a5);
  a69 = (a69 + a68);
  a55 = (a55 * a5);
  a69 = (a69 + a55);
  a69 = (a69 / a7);
  a69 = (a69 / a7);
  a69 = (a69 * a6);
  a8 = (a14 * a8);
  a69 = (a69 - a8);
  a69 = (a4 * a69);
  a67 = (a67 + a69);
  a67 = (-a67);
  if (res[0] != 0) res[0][8] = a67;
  if (res[0] != 0) res[0][9] = a0;
  a67 = (a3 * a1);
  a69 = (a12 * a2);
  a67 = (a67 + a69);
  a67 = (a67 / a7);
  a67 = (a67 / a7);
  a67 = (a67 * a6);
  a67 = (a4 * a67);
  a67 = (-a67);
  if (res[0] != 0) res[0][10] = a67;
  a67 = arg[1] ? arg[1][12] : 0;
  a69 = arg[0] ? arg[0][21] : 0;
  a8 = (a69 * a6);
  a55 = (a67 * a8);
  a68 = arg[1] ? arg[1][11] : 0;
  a70 = arg[0] ? arg[0][22] : 0;
  a71 = (a70 * a6);
  a72 = (a68 * a71);
  a55 = (a55 - a72);
  a72 = arg[1] ? arg[1][8] : 0;
  a73 = arg[0] ? arg[0][19] : 0;
  a74 = (a73 * a6);
  a75 = (a72 * a74);
  a55 = (a55 - a75);
  a75 = arg[1] ? arg[1][9] : 0;
  a76 = arg[0] ? arg[0][18] : 0;
  a77 = (a76 * a6);
  a78 = (a75 * a77);
  a55 = (a55 + a78);
  a78 = arg[1] ? arg[1][5] : 0;
  a79 = arg[0] ? arg[0][16] : 0;
  a80 = (a79 * a6);
  a81 = (a78 * a80);
  a55 = (a55 - a81);
  a81 = arg[1] ? arg[1][6] : 0;
  a82 = arg[0] ? arg[0][15] : 0;
  a83 = (a82 * a6);
  a84 = (a81 * a83);
  a55 = (a55 + a84);
  a84 = arg[1] ? arg[1][3] : 0;
  a85 = arg[0] ? arg[0][12] : 0;
  a86 = (a85 * a6);
  a87 = (a84 * a86);
  a55 = (a55 + a87);
  a87 = arg[1] ? arg[1][2] : 0;
  a88 = arg[0] ? arg[0][13] : 0;
  a89 = (a88 * a6);
  a90 = (a87 * a89);
  a55 = (a55 - a90);
  a55 = (a11 * a55);
  a90 = arg[0] ? arg[0][5] : 0;
  a91 = cos(a90);
  a92 = (a67 * a91);
  a93 = sin(a90);
  a94 = (a68 * a93);
  a92 = (a92 - a94);
  a94 = arg[0] ? arg[0][23] : 0;
  a95 = (a94 * a6);
  a96 = (a92 * a95);
  a97 = arg[1] ? arg[1][13] : 0;
  a98 = (a91 * a71);
  a98 = (a97 * a98);
  a96 = (a96 - a98);
  a98 = (a93 * a8);
  a98 = (a97 * a98);
  a96 = (a96 + a98);
  a98 = (a75 * a91);
  a99 = (a72 * a93);
  a98 = (a98 - a99);
  a99 = arg[0] ? arg[0][20] : 0;
  a100 = (a99 * a6);
  a101 = (a98 * a100);
  a96 = (a96 + a101);
  a101 = arg[1] ? arg[1][10] : 0;
  a102 = (a91 * a74);
  a102 = (a101 * a102);
  a96 = (a96 - a102);
  a102 = (a93 * a77);
  a102 = (a101 * a102);
  a96 = (a96 + a102);
  a102 = (a81 * a91);
  a103 = (a78 * a93);
  a102 = (a102 - a103);
  a103 = arg[0] ? arg[0][17] : 0;
  a104 = (a103 * a6);
  a105 = (a102 * a104);
  a96 = (a96 + a105);
  a105 = arg[1] ? arg[1][7] : 0;
  a106 = (a91 * a80);
  a106 = (a105 * a106);
  a96 = (a96 - a106);
  a106 = (a93 * a83);
  a106 = (a105 * a106);
  a96 = (a96 + a106);
  a106 = (a84 * a91);
  a107 = (a87 * a93);
  a106 = (a106 - a107);
  a107 = arg[0] ? arg[0][14] : 0;
  a108 = (a107 * a6);
  a109 = (a106 * a108);
  a96 = (a96 + a109);
  a109 = arg[1] ? arg[1][4] : 0;
  a110 = (a93 * a86);
  a110 = (a109 * a110);
  a96 = (a96 + a110);
  a110 = (a91 * a89);
  a110 = (a109 * a110);
  a96 = (a96 - a110);
  a96 = (a4 * a96);
  a55 = (a55 - a96);
  if (res[0] != 0) res[0][11] = a55;
  a55 = (a14 * a12);
  a96 = (a19 * a6);
  a55 = (a55 * a96);
  a110 = (a12 * a3);
  a111 = 2.4951959999999999e-002;
  a111 = (a111 * a6);
  a110 = (a110 * a111);
  a112 = (a14 * a3);
  a113 = (a23 * a6);
  a112 = (a112 * a113);
  a110 = (a110 + a112);
  a55 = (a55 - a110);
  a110 = (a16 * a6);
  a33 = (a33 * a110);
  a55 = (a55 - a33);
  a33 = (a16 * a6);
  a18 = (a18 * a33);
  a55 = (a55 + a18);
  a28 = (a19 * a28);
  a55 = (a55 + a28);
  a29 = (a23 * a29);
  a55 = (a55 + a29);
  a29 = -3.2950960000000001e-002;
  a30 = (a29 * a30);
  a55 = (a55 + a30);
  a11 = (a11 * a55);
  a42 = (a2 * a42);
  a15 = (a15 * a42);
  a45 = (a2 * a45);
  a43 = (a43 * a45);
  a36 = (a2 * a36);
  a46 = (a46 * a36);
  a43 = (a43 + a46);
  a15 = (a15 - a43);
  a31 = (a31 * a1);
  a31 = (a31 * a53);
  a15 = (a15 + a31);
  a56 = (a56 * a1);
  a56 = (a56 * a57);
  a15 = (a15 - a56);
  a59 = (a59 * a1);
  a59 = (a59 * a21);
  a15 = (a15 + a59);
  a47 = (a2 * a47);
  a15 = (a15 - a47);
  a49 = (a2 * a49);
  a15 = (a15 + a49);
  a62 = (a1 * a62);
  a15 = (a15 + a62);
  a25 = (a1 * a25);
  a15 = (a15 - a25);
  a50 = (a2 * a50);
  a15 = (a15 - a50);
  a64 = (a1 * a64);
  a15 = (a15 + a64);
  a51 = (a2 * a51);
  a15 = (a15 + a51);
  a65 = (a1 * a65);
  a15 = (a15 - a65);
  a52 = (a2 * a52);
  a15 = (a15 - a52);
  a66 = (a1 * a66);
  a15 = (a15 - a66);
  a4 = (a4 * a15);
  a11 = (a11 - a4);
  if (res[0] != 0) res[0][12] = a11;
  if (res[0] != 0) res[0][13] = a0;
  a11 = cos(a90);
  a4 = (a97 * a7);
  a8 = (a4 * a8);
  a95 = (a7 * a95);
  a15 = (a68 * a95);
  a8 = (a8 - a15);
  a100 = (a7 * a100);
  a15 = (a72 * a100);
  a8 = (a8 - a15);
  a15 = (a101 * a7);
  a77 = (a15 * a77);
  a8 = (a8 + a77);
  a104 = (a7 * a104);
  a77 = (a78 * a104);
  a8 = (a8 - a77);
  a77 = (a105 * a7);
  a83 = (a77 * a83);
  a8 = (a8 + a83);
  a108 = (a7 * a108);
  a83 = (a87 * a108);
  a8 = (a8 - a83);
  a83 = (a109 * a7);
  a86 = (a83 * a86);
  a8 = (a8 + a86);
  a8 = (a11 * a8);
  a90 = sin(a90);
  a95 = (a67 * a95);
  a86 = (a97 * a7);
  a71 = (a86 * a71);
  a95 = (a95 - a71);
  a100 = (a75 * a100);
  a95 = (a95 + a100);
  a100 = (a101 * a7);
  a74 = (a100 * a74);
  a95 = (a95 - a74);
  a104 = (a81 * a104);
  a95 = (a95 + a104);
  a104 = (a105 * a7);
  a80 = (a104 * a80);
  a95 = (a95 - a80);
  a108 = (a84 * a108);
  a95 = (a95 + a108);
  a108 = (a109 * a7);
  a89 = (a108 * a89);
  a95 = (a95 - a89);
  a95 = (a90 * a95);
  a8 = (a8 - a95);
  if (res[0] != 0) res[0][14] = a8;
  a70 = (a70 * a6);
  a70 = (a97 * a70);
  a94 = (a94 * a6);
  a8 = (a67 * a94);
  a70 = (a70 - a8);
  a99 = (a99 * a6);
  a8 = (a75 * a99);
  a70 = (a70 - a8);
  a73 = (a73 * a6);
  a73 = (a101 * a73);
  a70 = (a70 + a73);
  a103 = (a103 * a6);
  a73 = (a81 * a103);
  a70 = (a70 - a73);
  a79 = (a79 * a6);
  a79 = (a105 * a79);
  a70 = (a70 + a79);
  a88 = (a88 * a6);
  a88 = (a109 * a88);
  a70 = (a70 + a88);
  a107 = (a107 * a6);
  a88 = (a84 * a107);
  a70 = (a70 - a88);
  a11 = (a11 * a70);
  a69 = (a69 * a6);
  a69 = (a97 * a69);
  a94 = (a68 * a94);
  a69 = (a69 - a94);
  a99 = (a72 * a99);
  a69 = (a69 - a99);
  a76 = (a76 * a6);
  a76 = (a101 * a76);
  a69 = (a69 + a76);
  a103 = (a78 * a103);
  a69 = (a69 - a103);
  a82 = (a82 * a6);
  a82 = (a105 * a82);
  a69 = (a69 + a82);
  a85 = (a85 * a6);
  a85 = (a109 * a85);
  a69 = (a69 + a85);
  a107 = (a87 * a107);
  a69 = (a69 - a107);
  a90 = (a90 * a69);
  a11 = (a11 - a90);
  if (res[0] != 0) res[0][15] = a11;
  a11 = (-a6);
  if (res[0] != 0) res[0][16] = a11;
  a11 = -1.3740000000000000e+001;
  if (res[0] != 0) res[0][17] = a11;
  a90 = (-a6);
  if (res[0] != 0) res[0][18] = a90;
  if (res[0] != 0) res[0][19] = a11;
  a90 = (-a6);
  if (res[0] != 0) res[0][20] = a90;
  if (res[0] != 0) res[0][21] = a11;
  a11 = (-a6);
  if (res[0] != 0) res[0][22] = a11;
  a11 = (a3 * a6);
  a90 = (a23 * a11);
  a69 = (a12 * a6);
  a107 = (a19 * a69);
  a90 = (a90 - a107);
  a90 = (a90 + a29);
  if (res[0] != 0) res[0][23] = a90;
  a17 = (a2 * a17);
  a90 = (a3 * a17);
  a22 = (a2 * a22);
  a107 = (a12 * a22);
  a90 = (a90 - a107);
  a37 = (a1 * a37);
  a107 = (a3 * a37);
  a90 = (a90 - a107);
  a32 = (a1 * a32);
  a107 = (a12 * a32);
  a90 = (a90 + a107);
  a107 = (a14 + a14);
  a26 = (a2 * a26);
  a38 = (a1 * a38);
  a26 = (a26 + a38);
  a26 = (a107 * a26);
  a90 = (a90 + a26);
  a26 = (a41 * a1);
  a38 = (a19 * a2);
  a26 = (a26 + a38);
  a90 = (a90 - a26);
  if (res[0] != 0) res[0][24] = a90;
  a42 = (a7 * a42);
  a90 = (a12 * a42);
  a36 = (a7 * a36);
  a38 = (a3 * a36);
  a90 = (a90 - a38);
  a58 = (a1 * a58);
  a38 = (a3 * a58);
  a90 = (a90 - a38);
  a60 = (a1 * a60);
  a38 = (a12 * a60);
  a90 = (a90 + a38);
  a113 = (a5 * a113);
  a38 = (a3 * a113);
  a90 = (a90 - a38);
  a96 = (a5 * a96);
  a38 = (a12 * a96);
  a90 = (a90 + a38);
  a38 = (a7 * a2);
  a38 = (a38 * a48);
  a48 = (a1 * a7);
  a48 = (a48 * a63);
  a38 = (a38 - a48);
  a107 = (a107 * a38);
  a90 = (a90 + a107);
  a29 = (a29 * a5);
  a107 = (a1 * a7);
  a107 = (a19 * a107);
  a29 = (a29 - a107);
  a107 = (a7 * a2);
  a107 = (a23 * a107);
  a29 = (a29 - a107);
  a90 = (a90 - a29);
  if (res[0] != 0) res[0][25] = a90;
  a13 = (a2 * a13);
  a13 = (-a13);
  if (res[0] != 0) res[0][26] = a13;
  a13 = (a1 * a6);
  a13 = (-a13);
  if (res[0] != 0) res[0][27] = a13;
  a13 = (a2 * a10);
  a13 = (-a13);
  if (res[0] != 0) res[0][28] = a13;
  a13 = (a19 * a14);
  a90 = (a16 * a12);
  a13 = (a13 + a90);
  a90 = (a27 * a3);
  a13 = (a13 - a90);
  a13 = (a13 * a6);
  a90 = (a16 * a69);
  a13 = (a13 + a90);
  a90 = (a40 * a11);
  a13 = (a13 + a90);
  a13 = (a23 - a13);
  if (res[0] != 0) res[0][29] = a13;
  a32 = (a14 * a32);
  a20 = (a2 * a20);
  a13 = (a3 * a20);
  a22 = (a14 * a22);
  a13 = (a13 + a22);
  a34 = (a1 * a34);
  a22 = (a3 * a34);
  a13 = (a13 + a22);
  a32 = (a32 - a13);
  a13 = (a12 + a12);
  a24 = (a2 * a24);
  a24 = (a13 * a24);
  a32 = (a32 - a24);
  a24 = (a40 * a1);
  a22 = (a16 * a2);
  a24 = (a24 + a22);
  a32 = (a32 - a24);
  if (res[0] != 0) res[0][30] = a32;
  a42 = (a14 * a42);
  a45 = (a7 * a45);
  a32 = (a3 * a45);
  a42 = (a42 - a32);
  a54 = (a1 * a54);
  a32 = (a3 * a54);
  a42 = (a42 + a32);
  a60 = (a14 * a60);
  a42 = (a42 + a60);
  a111 = (a5 * a111);
  a60 = (a3 * a111);
  a42 = (a42 - a60);
  a96 = (a14 * a96);
  a42 = (a42 + a96);
  a96 = (a1 * a7);
  a96 = (a96 * a61);
  a33 = (a5 * a33);
  a96 = (a96 + a33);
  a13 = (a13 * a96);
  a42 = (a42 + a13);
  a13 = (a23 * a5);
  a96 = (a1 * a7);
  a96 = (a16 * a96);
  a13 = (a13 - a96);
  a96 = (a7 * a2);
  a96 = (a40 * a96);
  a13 = (a13 + a96);
  a42 = (a42 - a13);
  if (res[0] != 0) res[0][31] = a42;
  a9 = (a1 * a9);
  a9 = (-a9);
  if (res[0] != 0) res[0][32] = a9;
  a9 = (a2 * a6);
  if (res[0] != 0) res[0][33] = a9;
  a10 = (a1 * a10);
  a10 = (-a10);
  if (res[0] != 0) res[0][34] = a10;
  a69 = (a27 * a69);
  a23 = (a23 * a14);
  a40 = (a40 * a12);
  a23 = (a23 - a40);
  a40 = (a16 * a3);
  a23 = (a23 + a40);
  a23 = (a23 * a6);
  a69 = (a69 + a23);
  a11 = (a16 * a11);
  a69 = (a69 + a11);
  a69 = (a69 + a19);
  if (res[0] != 0) res[0][35] = a69;
  a17 = (a14 * a17);
  a20 = (a12 * a20);
  a17 = (a17 - a20);
  a34 = (a12 * a34);
  a17 = (a17 - a34);
  a37 = (a14 * a37);
  a17 = (a17 - a37);
  a3 = (a3 + a3);
  a35 = (a1 * a35);
  a35 = (a3 * a35);
  a17 = (a17 - a35);
  a39 = (a39 * a1);
  a35 = (a27 * a2);
  a39 = (a39 - a35);
  a17 = (a17 - a39);
  if (res[0] != 0) res[0][36] = a17;
  a54 = (a12 * a54);
  a45 = (a12 * a45);
  a36 = (a14 * a36);
  a45 = (a45 + a36);
  a54 = (a54 - a45);
  a58 = (a14 * a58);
  a54 = (a54 - a58);
  a12 = (a12 * a111);
  a54 = (a54 - a12);
  a14 = (a14 * a113);
  a54 = (a54 - a14);
  a14 = (a7 * a2);
  a14 = (a14 * a44);
  a110 = (a5 * a110);
  a14 = (a14 + a110);
  a3 = (a3 * a14);
  a54 = (a54 - a3);
  a19 = (a19 * a5);
  a1 = (a1 * a7);
  a27 = (a27 * a1);
  a19 = (a19 + a27);
  a2 = (a7 * a2);
  a16 = (a16 * a2);
  a19 = (a19 - a16);
  a54 = (a54 - a19);
  if (res[0] != 0) res[0][37] = a54;
  a54 = (-a6);
  if (res[0] != 0) res[0][38] = a54;
  a16 = (a84 * a5);
  a83 = (a83 * a93);
  a16 = (a16 + a83);
  a16 = (a16 * a6);
  if (res[0] != 0) res[0][39] = a16;
  a16 = (a109 * a91);
  a16 = (a16 * a6);
  if (res[0] != 0) res[0][40] = a16;
  a16 = (a84 * a6);
  a16 = (-a16);
  if (res[0] != 0) res[0][41] = a16;
  a16 = 1.;
  if (res[0] != 0) res[0][42] = a16;
  if (res[0] != 0) res[0][43] = a0;
  a83 = (-a6);
  if (res[0] != 0) res[0][44] = a83;
  a2 = (a87 * a5);
  a108 = (a108 * a91);
  a2 = (a2 + a108);
  a2 = (a2 * a6);
  a2 = (-a2);
  if (res[0] != 0) res[0][45] = a2;
  a109 = (a109 * a93);
  a109 = (a109 * a6);
  if (res[0] != 0) res[0][46] = a109;
  a109 = (a87 * a6);
  if (res[0] != 0) res[0][47] = a109;
  if (res[0] != 0) res[0][48] = a16;
  if (res[0] != 0) res[0][49] = a0;
  a109 = (-a6);
  if (res[0] != 0) res[0][50] = a109;
  a106 = (a7 * a106);
  a106 = (a106 * a6);
  if (res[0] != 0) res[0][51] = a106;
  a87 = (a87 * a91);
  a84 = (a84 * a93);
  a87 = (a87 + a84);
  a87 = (a87 * a6);
  a87 = (-a87);
  if (res[0] != 0) res[0][52] = a87;
  a87 = arg[1] ? arg[1][1] : 0;
  a84 = (-a87);
  if (res[0] != 0) res[0][53] = a84;
  a84 = (-a87);
  if (res[0] != 0) res[0][54] = a84;
  a84 = (-a87);
  if (res[0] != 0) res[0][55] = a84;
  a84 = (-a87);
  if (res[0] != 0) res[0][56] = a84;
  if (res[0] != 0) res[0][57] = a54;
  a84 = (a81 * a5);
  a77 = (a77 * a93);
  a84 = (a84 + a77);
  a84 = (a84 * a6);
  if (res[0] != 0) res[0][58] = a84;
  a84 = (a105 * a91);
  a84 = (a84 * a6);
  if (res[0] != 0) res[0][59] = a84;
  a84 = (a81 * a6);
  a84 = (-a84);
  if (res[0] != 0) res[0][60] = a84;
  if (res[0] != 0) res[0][61] = a16;
  if (res[0] != 0) res[0][62] = a0;
  if (res[0] != 0) res[0][63] = a83;
  a84 = (a78 * a5);
  a104 = (a104 * a91);
  a84 = (a84 + a104);
  a84 = (a84 * a6);
  a84 = (-a84);
  if (res[0] != 0) res[0][64] = a84;
  a105 = (a105 * a93);
  a105 = (a105 * a6);
  if (res[0] != 0) res[0][65] = a105;
  a105 = (a78 * a6);
  if (res[0] != 0) res[0][66] = a105;
  if (res[0] != 0) res[0][67] = a16;
  if (res[0] != 0) res[0][68] = a0;
  if (res[0] != 0) res[0][69] = a109;
  a102 = (a7 * a102);
  a102 = (a102 * a6);
  if (res[0] != 0) res[0][70] = a102;
  a78 = (a78 * a91);
  a81 = (a81 * a93);
  a78 = (a78 + a81);
  a78 = (a78 * a6);
  a78 = (-a78);
  if (res[0] != 0) res[0][71] = a78;
  a78 = (-a87);
  if (res[0] != 0) res[0][72] = a78;
  a78 = (-a87);
  if (res[0] != 0) res[0][73] = a78;
  a78 = (-a87);
  if (res[0] != 0) res[0][74] = a78;
  a78 = (-a87);
  if (res[0] != 0) res[0][75] = a78;
  if (res[0] != 0) res[0][76] = a54;
  a78 = (a75 * a5);
  a15 = (a15 * a93);
  a78 = (a78 + a15);
  a78 = (a78 * a6);
  if (res[0] != 0) res[0][77] = a78;
  a78 = (a101 * a91);
  a78 = (a78 * a6);
  if (res[0] != 0) res[0][78] = a78;
  a78 = (a75 * a6);
  a78 = (-a78);
  if (res[0] != 0) res[0][79] = a78;
  if (res[0] != 0) res[0][80] = a16;
  if (res[0] != 0) res[0][81] = a0;
  if (res[0] != 0) res[0][82] = a83;
  a78 = (a72 * a5);
  a100 = (a100 * a91);
  a78 = (a78 + a100);
  a78 = (a78 * a6);
  a78 = (-a78);
  if (res[0] != 0) res[0][83] = a78;
  a101 = (a101 * a93);
  a101 = (a101 * a6);
  if (res[0] != 0) res[0][84] = a101;
  a101 = (a72 * a6);
  if (res[0] != 0) res[0][85] = a101;
  if (res[0] != 0) res[0][86] = a16;
  if (res[0] != 0) res[0][87] = a0;
  if (res[0] != 0) res[0][88] = a109;
  a98 = (a7 * a98);
  a98 = (a98 * a6);
  if (res[0] != 0) res[0][89] = a98;
  a72 = (a72 * a91);
  a75 = (a75 * a93);
  a72 = (a72 + a75);
  a72 = (a72 * a6);
  a72 = (-a72);
  if (res[0] != 0) res[0][90] = a72;
  a72 = (-a87);
  if (res[0] != 0) res[0][91] = a72;
  a72 = (-a87);
  if (res[0] != 0) res[0][92] = a72;
  a72 = (-a87);
  if (res[0] != 0) res[0][93] = a72;
  a72 = (-a87);
  if (res[0] != 0) res[0][94] = a72;
  if (res[0] != 0) res[0][95] = a54;
  a54 = (a67 * a5);
  a4 = (a4 * a93);
  a54 = (a54 + a4);
  a54 = (a54 * a6);
  if (res[0] != 0) res[0][96] = a54;
  a54 = (a97 * a91);
  a54 = (a54 * a6);
  if (res[0] != 0) res[0][97] = a54;
  a54 = (a67 * a6);
  a54 = (-a54);
  if (res[0] != 0) res[0][98] = a54;
  if (res[0] != 0) res[0][99] = a16;
  if (res[0] != 0) res[0][100] = a0;
  if (res[0] != 0) res[0][101] = a83;
  a5 = (a68 * a5);
  a86 = (a86 * a91);
  a5 = (a5 + a86);
  a5 = (a5 * a6);
  a5 = (-a5);
  if (res[0] != 0) res[0][102] = a5;
  a97 = (a97 * a93);
  a97 = (a97 * a6);
  if (res[0] != 0) res[0][103] = a97;
  a97 = (a68 * a6);
  if (res[0] != 0) res[0][104] = a97;
  if (res[0] != 0) res[0][105] = a16;
  if (res[0] != 0) res[0][106] = a0;
  if (res[0] != 0) res[0][107] = a109;
  a7 = (a7 * a92);
  a7 = (a7 * a6);
  if (res[0] != 0) res[0][108] = a7;
  a68 = (a68 * a91);
  a67 = (a67 * a93);
  a68 = (a68 + a67);
  a68 = (a68 * a6);
  a68 = (-a68);
  if (res[0] != 0) res[0][109] = a68;
  a68 = (-a87);
  if (res[0] != 0) res[0][110] = a68;
  a68 = (-a87);
  if (res[0] != 0) res[0][111] = a68;
  a68 = (-a87);
  if (res[0] != 0) res[0][112] = a68;
  a87 = (-a87);
  if (res[0] != 0) res[0][113] = a87;
  if (res[0] != 0) res[0][114] = a16;
  if (res[0] != 0) res[0][115] = a16;
  if (res[0] != 0) res[0][116] = a16;
  if (res[0] != 0) res[0][117] = a16;
  if (res[0] != 0) res[0][118] = a16;
  if (res[0] != 0) res[0][119] = a16;
  a16 = 1.3740000000000000e+001;
  if (res[0] != 0) res[0][120] = a16;
  if (res[0] != 0) res[0][121] = a16;
  if (res[0] != 0) res[0][122] = a16;
  a16 = 3.2950960000000001e-002;
  if (res[0] != 0) res[0][123] = a16;
  if (res[0] != 0) res[0][124] = a26;
  if (res[0] != 0) res[0][125] = a29;
  if (res[0] != 0) res[0][126] = a41;
  if (res[0] != 0) res[0][127] = a24;
  if (res[0] != 0) res[0][128] = a13;
  a13 = -6.1099999999999994e-005;
  if (res[0] != 0) res[0][129] = a13;
  if (res[0] != 0) res[0][130] = a39;
  if (res[0] != 0) res[0][131] = a19;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_a1(const casadi_real** arg,
                                                  casadi_real** res,
                                                  casadi_int* iw,
                                                  casadi_real* w, int mem) {
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_a1_alloc_mem(void) { return 0; }

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_a1_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_a1_free_mem(int mem) {}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_a1_checkout(void) { return 0; }

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_a1_release(int mem) {}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_a1_incref(void) {}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_a1_decref(void) {}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_jac_g_a1_n_in(void) {
  return 2;
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_jac_g_a1_n_out(void) {
  return 1;
}

extern "C" CASADI_SYMBOL_EXPORT casadi_real
eval_jac_g_a1_default_in(casadi_int i) {
  switch (i) {
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_jac_g_a1_name_in(
    casadi_int i) {
  switch (i) {
    case 0:
      return "w";
    case 1:
      return "p";
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_jac_g_a1_name_out(
    casadi_int i) {
  switch (i) {
    case 0:
      return "jac_g";
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_jac_g_a1_sparsity_in(
    casadi_int i) {
  switch (i) {
    case 0:
      return casadi_s0;
    case 1:
      return casadi_s1;
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_jac_g_a1_sparsity_out(
    casadi_int i) {
  switch (i) {
    case 0:
      return casadi_s2;
    default:
      return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_a1_work(casadi_int* sz_arg,
                                                       casadi_int* sz_res,
                                                       casadi_int* sz_iw,
                                                       casadi_int* sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}
