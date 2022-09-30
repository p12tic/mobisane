/*  SPDX-License-Identifier: MIT

    The MIT License (MIT)

    Copyright (c) 2017 Rohan Sawhney and Keenan Crane

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
    Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
    PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "DenseMatrix.h"
#include <Eigen/SparseCore>

namespace bff {

class Triplet;

class SparseMatrix {
public:
    using EigenMatrix = Eigen::SparseMatrix<double>;

    // constructor
    SparseMatrix(size_t m = 0, size_t n = 0);
    // constructor
    SparseMatrix(const EigenMatrix& mat) { mat_ = mat; }

    // assignment operators
    SparseMatrix& operator=(EigenMatrix& mat);

    // returns identity
	static SparseMatrix identity(size_t m, size_t n);

    // return sparse diagonal matrix
	static SparseMatrix diag(const DenseMatrix& d);

    // returns transpose
	SparseMatrix transpose() const;

    // returns number of rows
    size_t nRows() const { return mat_.rows(); }

    // returns number of columns
    size_t nCols() const { return mat_.cols(); }

    // returns number of non zeros
    size_t nnz() const { return mat_.nonZeros(); }

    // returns norm. 0: Infinity, 1: 1-norm
    double norm(int norm) const { return mat_.norm(); }

    // extracts submatrix in range [r0, r1) x [c0, c1)
	SparseMatrix submatrix(size_t r0, size_t r1, size_t c0, size_t c1) const;

    // extracts submatrix with specified row and column indices
    SparseMatrix submatrix(const std::vector<int>& r, const std::vector<int>& c) const;

    // returns dense
    DenseMatrix toDense() const { return DenseMatrix(mat_.toDense()); }

    // math
    friend SparseMatrix operator*(const SparseMatrix& A, double s)
    {
        return EigenMatrix(A.mat() * s);
    }
    friend SparseMatrix operator+(const SparseMatrix& A, const SparseMatrix& B)
    {
        return EigenMatrix(A.mat() + B.mat());
    }
    friend SparseMatrix operator-(const SparseMatrix& A, const SparseMatrix& B)
    {
        return EigenMatrix(A.mat() - B.mat());
    }
    friend SparseMatrix operator*(const SparseMatrix& A, const SparseMatrix& B)
    {
        return EigenMatrix(A.mat() * B.mat());
    }

    SparseMatrix& operator*=(double s)
    {
        mat() *= s;
        return *this;
    }
    SparseMatrix& operator+=(const SparseMatrix& B)
    {
        mat() += B.mat();
        return *this;
    }
    SparseMatrix& operator-=(const SparseMatrix& B)
    {
        mat() -= B.mat();
        return *this;
    }

    const EigenMatrix& mat() const { return mat_; }
    EigenMatrix& mat() { return mat_; }

protected:
    // clear
    void clear()
    {
        mat_ = EigenMatrix();
    }

    EigenMatrix mat_;
};

} // namespace bff

#include "SparseMatrix.inl"
