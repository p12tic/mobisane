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

#include <Eigen/Core>
#include <cstdlib>
#include <vector>

namespace bff {

class SparseMatrix;

class DenseMatrix {
public:
    using EigenMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

    // constructor
	DenseMatrix(size_t m = 1, size_t n = 1);

    DenseMatrix(const EigenMatrix& m) : mat_{m} {}

    // returns identity
	static DenseMatrix identity(size_t m, size_t n = 1);

    // returns ones
	static DenseMatrix ones(size_t m, size_t n = 1);

    // returns transpose
	DenseMatrix transpose() const;

    // returns number of rows
    size_t nRows() const { return mat_.rows(); }

    // returns number of columns
    size_t nCols() const { return mat_.cols(); }

    // returns norm. 0: Infinity, 1: 1-norm, 2: 2-norm. Note: 2-norm is only valid for row vectors
    double norm(int norm) const { return mat_.norm(); }

    // returns sum
    double sum() const { return mat_.sum(); }

    // returns mean
    double mean() const { return mat_.mean(); }

    // extracts submatrix in range [r0, r1) x [c0, c1)
	DenseMatrix submatrix(size_t r0, size_t r1, size_t c0 = 0, size_t c1 = 1) const;

    // extracts submatrix with specified row and column indices
	DenseMatrix submatrix(const std::vector<int>& r, const std::vector<int>& c = {0}) const;

    // math
    friend DenseMatrix operator*(const DenseMatrix& A, double s)
    {
        return EigenMatrix(A.mat() * s);
    }
    friend DenseMatrix operator+(const DenseMatrix& A, const DenseMatrix& B)
    {
        return EigenMatrix(A.mat() + B.mat());
    }
    friend DenseMatrix operator-(const DenseMatrix& A, const DenseMatrix& B)
    {
        return EigenMatrix(A.mat() - B.mat());
    }
    friend DenseMatrix operator*(const DenseMatrix& A, const DenseMatrix& B)
    {
        return EigenMatrix(A.mat() * B.mat());
    }
    friend DenseMatrix operator*(const SparseMatrix& A, const DenseMatrix& X) ;
    friend DenseMatrix operator-(const DenseMatrix& A)
    {
        return EigenMatrix(-A.mat());
    }

    DenseMatrix& operator*=(double s)
    {
        mat_ *= s; return *this;
    }
    DenseMatrix& operator+=(const DenseMatrix& B)
    {
        mat_ += B.mat(); return *this;
    }
    DenseMatrix& operator-=(const DenseMatrix& B)
    {
        mat_ -= B.mat(); return *this;
    }

    // access
    double& operator()(size_t r, size_t c = 0) { return mat_(r, c); }
    double operator()(size_t r, size_t c = 0) const { return mat_(r, c); }

    void clean()
    {
        mat_ = EigenMatrix();
    }

    const EigenMatrix& mat() const { return mat_; }
    EigenMatrix& mat() { return mat_; }

protected:

    EigenMatrix mat_;
};

// horizontal concat
DenseMatrix hcat(const DenseMatrix& A, const DenseMatrix& B);

// vertical concat
DenseMatrix vcat(const DenseMatrix& A, const DenseMatrix& B);

} // namespace bff

#include "DenseMatrix.inl"
