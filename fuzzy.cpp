#include "fuzzy.h"



double fuzzy::Lds[5] = {0, 1.0/4, 2.0/4, 3.0/4, 4.0/4};
/*the fuzzy rules
 *Z is the 1
 ×S is the 2
 *M is the 3
 *L is the 4
 */
int fuzzy::rule[5][3] = {{0,1,2},{0,2,3},{0,3,4},{0,2,3},{0,1,2}};

fuzzy::fuzzy() {

    wd = 1.0;
    wv = 1.0/1.5;
    wLd = 6.0;
}

double fuzzy::fuzzyOutput(double d, double v) {
    fuzzy_in *d_in = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
    fuzzy_in *v_in = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
    d_in->value = d*wd;
    v_in->value = v*wv;

    double w[NUM_RULES] = {0.0f};
    double u[NUM_RULES] = {0.0f};

    int k = 0;
    double sum_w = 0.0f, sum_u = 0.0f;
    for (int i=10; i<=14; i++) {
        for (int j=20; j<=22; j++) {
            d_in->membership_func = i;
            v_in->membership_func = j;
            w[k] = compute_aggregation(compute_membership(d_in), compute_membership(v_in));
            u[k] = Lds[rule[i-10][j-20]];
            sum_w += w[k];
            sum_u += w[k]*u[k];
            k++;
        }
    }
    double out = sum_u/sum_w;

    free(d_in);
    free(v_in);
    return out*wLd;
}

double fuzzy::compute_aggregation(double memb_val_in1, double memb_val_in2) {
    return min(memb_val_in1, memb_val_in2);
}

double fuzzy::compute_membership(fuzzy_in *in) {
    double a, b, c;
        double out = 0.0f;
        switch(in->membership_func){
            /* for distance error*/
            case NLd:
                a = -1.5, b = -1, c = -0.5;
            break;
            case NSd:
                a = -1, b = -0.5, c = 0;
            break;
            case Zd:
                a = -0.5, b = 0, c = 0.5;
            break;
            case PSd:
                a = 0, b = 0.5, c = 1;
            break;
            case PLd:
                a = 0.5, b = 1, c = 1.5;
            break;
            case Zv:
                a = -0.5, b = 0, c = 0.5;
            break;
            case Sv:
                a = 0, b = 0.5, c = 1.0;
            break;
            case Lv:
                a = 0.5, b = 1.0, c = 1.5;
            break;
        }

        /*s使用等腰三角形为隶属度函数*/
        out = max(min((in->value - a)/(b-a), (c - in->value)/(c - b)), 0);
        return out;
}
