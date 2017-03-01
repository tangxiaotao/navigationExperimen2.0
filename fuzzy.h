#ifndef FUZZY_H
#define FUZZY_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

/* handy macros */
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define NUM_RULES	15

enum {
    /* the membership wedges of distance */
    NLd = 10,
    NSd = 11,
    Zd  = 12,
    PSd = 13,
    PLd = 14,

    /* the membership wedges of velocity */
    Zv = 20,
    Sv = 21,
    Lv = 22
};

typedef struct {
    double value;	/* crisp value */
    int membership_func;    /* the specific function Small, Large ... */
}fuzzy_in;

class fuzzy{
public:
    fuzzy();
    double fuzzyOutput(double d, double v);

    static double Lds[5];
    static int rule[5][3];

    //输入输出权重因子
    double wd;
    double wv;
    double wLd;

private:
    double compute_aggregation(double memb_val_in1, double memb_val_in2);
    double compute_membership(fuzzy_in* in);
};

#endif // FUZZY_H
