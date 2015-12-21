#ifndef MATRIX_HEADER
#define MATRIX_HEADER

#include "Vector.h"

namespace mmm {

    //typedef std::vector<Vector> Matrix;
    class Matrix {
    public:
        Matrix(double val) {
            for (int i = 0; i < DIM; ++i) {
                for (int j = 0; j < DIM; ++j) {
                    a[i][j] = val;
                }
            }
        }
        const double *operator[] (int row) const {
            return a[row];
        }
        double *operator[] (int row) {
            return a[row];
        }
        static const size_t DIM = 4;
    private:
        double a[4][4];
    };


    /* Outer product accumulator. */
    void outerProductAcc(const Vector &a, const Vector &b, Matrix &acc);
    Vector innerProduct(const Vector &a, const Matrix &b);

    Matrix operator + (const Matrix &a, const Matrix &b);
}

#endif